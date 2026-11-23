# ============================================================
#  AeroSense Server  (FastAPI)
#
#  What it does now:
#   1. CAMERA   — replays camera frames + YOLO detections from ALL drone types
#                 (Avata, M300, Mavic_2, Mavic_3, Phantom_4) to the dashboard.
#   2. MAVLINK  — listens on UDP 14445 for MAVLink telemetry (lat/lon/alt,
#                 heading, speed). Source can be:
#                   * sim_drone.py (virtual drone flying a QGC .plan), or
#                   * QGroundControl's "MAVLink forwarding" (real drone / PX4 SITL),
#                   * a real autopilot / telemetry radio bridged to UDP.
#   3. GEOFENCE — 3D geofence of FAST-NUCES Islamabad (polygons + altitude bands)
#                 → threat level CLEAR / ADVISORY / WARNING / BREACH / CRITICAL,
#                 fused with the camera detection, logged to SQLite.
#   4. SENSORS  — replays MMAUD radar / LiDAR model outputs (frontend_demo_feed.json)
#                 OR legacy individual sensor files (sensor_adapters.py).
#   5. PLANNING — zones + missions editable from the dashboard, exported as a
#                 QGroundControl .plan file.
#
#  Run:   python server.py --sim        (server + built-in virtual drone)
#         python server.py              (waits for MAVLink from sim_drone.py / QGC)
#  Open:  http://127.0.0.1:8000/dashboard
# ============================================================

import argparse
import asyncio
import base64
import json
import os
import sqlite3
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import JSONResponse, FileResponse, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

import geo
from geofence import GeofenceMonitor, TrackSample, load_zones, save_zones, zones_from_api, LEVELS
from mavlink_lite import MavlinkParser, MAV_TYPE_GCS, MAV_MODE_FLAG_SAFETY_ARMED
from qgc_plan import build_plan, parse_plan
from sensor_adapters import SensorReplay

# ---------- Settings you can change ----------
HERE = Path(__file__).parent
PLAYBACK_FPS = 10
MAVLINK_PORT = 14445                 # QGC's MAVLink-forwarding example port
SITE_FILE = HERE / "data" / "site_config.json"
ZONES_FILE = HERE / "data" / "geofence" / "fast_campus_zones.json"
MISSION_FILE = HERE / "data" / "missions" / "fast_intruder_demo.plan"
DB_FILE = HERE / "data" / "aerosense_events.db"
MMAUD_FEED_FILE = HERE / "data" / "frontend_demo_feed.json"
TELEMETRY_HZ = 5
LINK_TIMEOUT_S = 3.0
RUN_SIM = os.environ.get("AEROSENSE_SIM", "0") == "1"

# Camera: cycle through ALL drone types
DRONE_TYPES = [
    {"name": "Avata",     "frames": "data/test_frames/Avata",     "detections": "data/avata_detections.json"},
    {"name": "M300",      "frames": "data/test_frames/M300",      "detections": "data/M300_detections.json"},
    {"name": "Mavic_2",   "frames": "data/test_frames/Mavic_2",   "detections": "data/Mavic_2_detections.json"},
    {"name": "Mavic_3",   "frames": "data/test_frames/Mavic_3",   "detections": "data/Mavic_3_detections.json"},
    {"name": "Phantom_4", "frames": "data/test_frames/Phantom_4", "detections": "data/Phantom_4_detections.json"},
]
# ---------------------------------------------


# ======================================================== MMAUD feed ====

# Size mapping: Avata = small, M300 = large, Mavic2/3 = medium, Phantom4 = medium
DRONE_SIZES = {"Avata": "small", "M300": "large", "Mavic2": "medium",
               "Mavic3": "medium", "Pham4": "medium", "Phantom_4": "medium"}


class MMAUDFeedReplay:
    """Replays frontend_demo_feed.json from the MMAUD dataset.

    The feed is a list of 'windows' at 5 Hz. Each window has keys for each
    sensor (radar, avia, mid360) and a placeholder_fusion, each with:
      detected: bool
      position: {x,y,z} or [x,y,z] or null
      class_probs: {Avata:0.6, ...} or [0.6, ...] or null
    """

    DRONE_NAMES = ["Avata", "M300", "Mavic2", "Mavic3", "Pham4"]

    def __init__(self, path, site, heading_deg=0.0):
        self.path = Path(path)
        self.site = site
        self.heading = heading_deg
        self.windows = []
        self.duration = 0.0
        self.rate_hz = 5.0
        self.error = ""
        self.load()

    @property
    def available(self):
        return bool(self.windows)

    def load(self):
        self.windows, self.error = [], ""
        if not self.path.exists():
            self.error = f"Place frontend_demo_feed.json in data/ folder"
            return
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            if isinstance(raw, dict) and "windows" in raw:
                self.windows = raw["windows"]
            elif isinstance(raw, list):
                self.windows = raw
            else:
                raise ValueError("Expected a list of windows or {windows:[...]}")
            self.duration = len(self.windows) / self.rate_hz
            print(f"[mmaud] loaded {len(self.windows)} windows from {self.path.name} ({self.duration:.1f} s)")
        except Exception as e:
            self.error = f"could not load MMAUD feed: {e}"
            self.windows = []

    def _parse_position(self, pos):
        """Parse position from various formats: {x,y,z}, [x,y,z], or null."""
        if pos is None:
            return None
        if isinstance(pos, dict):
            return float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0))
        if isinstance(pos, (list, tuple)) and len(pos) >= 3:
            return float(pos[0]), float(pos[1]), float(pos[2])
        return None

    def _parse_class_probs(self, probs):
        """Parse class_probs from dict or list format."""
        if probs is None:
            return {}
        if isinstance(probs, dict):
            return {k: float(v) for k, v in probs.items()}
        if isinstance(probs, (list, tuple)):
            return {self.DRONE_NAMES[i]: float(v) for i, v in enumerate(probs) if i < len(self.DRONE_NAMES)}
        return {}

    def _pos_to_latlon(self, xyz):
        """Convert sensor_xyz (x forward, y left, z up) to lat/lon/alt."""
        if xyz is None:
            return None
        x, y, z = xyz
        s = self.site
        h0 = float(s.get("height_agl", 0.0))
        e, n, u = geo.sensor_xyz_to_enu(x, y, z, self.heading)
        lat, lon, alt = geo.enu_to_geodetic(e, n, u, s["lat"], s["lon"], h0)
        rng, az, el = geo.enu_to_polar(e, n, u)
        return {"lat": lat, "lon": lon, "alt": alt, "range": rng, "azimuth": az, "elevation": el}

    def _classify_size(self, class_probs):
        """Classify drone size from class_probs."""
        if not class_probs:
            return "unknown"
        top = max(class_probs, key=class_probs.get)
        return DRONE_SIZES.get(top, "medium")

    def _top_types(self, class_probs, n=2):
        """Return top-n drone types with percentages."""
        if not class_probs:
            return []
        sorted_probs = sorted(class_probs.items(), key=lambda x: x[1], reverse=True)
        return [{"type": k, "prob": round(v * 100, 1)} for k, v in sorted_probs[:n]]

    def _parse_sensor(self, window, sensor_key):
        """Parse one sensor's data from a window."""
        data = window.get(sensor_key, {})
        if data is None:
            data = {}
        detected = bool(data.get("detected", False))
        raw_pos = self._parse_position(data.get("position"))
        class_probs = self._parse_class_probs(data.get("class_probs"))
        pos_map = self._pos_to_latlon(raw_pos) if detected and raw_pos else None
        result = {
            "detected": detected,
            "position": pos_map,
            "class_probs": class_probs,
            "size": self._classify_size(class_probs) if detected else None,
            "top_types": self._top_types(class_probs) if detected else [],
        }
        return result

    def at(self, elapsed_s):
        """Get the sensor data for the current time (loops)."""
        if not self.windows:
            return None
        t = elapsed_s % self.duration if self.duration > 0 else 0
        idx = int(t * self.rate_hz)
        idx = max(0, min(idx, len(self.windows) - 1))
        window = self.windows[idx]

        result = {
            "window_index": idx,
            "total_windows": len(self.windows),
            "radar": self._parse_sensor(window, "radar"),
            "avia": self._parse_sensor(window, "avia"),
            "mid360": self._parse_sensor(window, "mid360"),
            "fusion": self._parse_sensor(window, "placeholder_fusion"),
        }
        return result

    def status(self):
        return {
            "available": self.available,
            "windows": len(self.windows),
            "duration_s": round(self.duration, 1),
            "message": self.error or f"{len(self.windows)} MMAUD windows loaded ({self.duration:.1f} s)",
        }

    def sensor_detections(self, sensor_key, elapsed_s):
        """Get detections in the contract format for backward compat with map display."""
        if not self.windows:
            return []
        t = elapsed_s % self.duration if self.duration > 0 else 0
        idx = int(t * self.rate_hz)
        idx = max(0, min(idx, len(self.windows) - 1))
        window = self.windows[idx]
        data = window.get(sensor_key, {})
        if not data or not data.get("detected"):
            return []
        raw_pos = self._parse_position(data.get("position"))
        pos_map = self._pos_to_latlon(raw_pos) if raw_pos else None
        if not pos_map:
            return []
        class_probs = self._parse_class_probs(data.get("class_probs"))
        return [{
            "lat": pos_map["lat"], "lon": pos_map["lon"], "alt": pos_map["alt"],
            "range": pos_map["range"], "azimuth": pos_map["azimuth"], "elevation": pos_map["elevation"],
            "confidence": max(class_probs.values()) if class_probs else 0.5,
            "track_id": 1, "label": "drone",
        }]


# ======================================================== fusion engine ====

class FusionEngine:
    """Multi-sensor fusion: associates detections from all sensors,
    computes weighted position, tracks which sensors contributed.
    Generates detection events when state changes."""

    MMAUD_TO_CAMERA = {"Avata": "Avata", "M300": "M300", "Mavic2": "Mavic_2",
                       "Mavic3": "Mavic_3", "Pham4": "Phantom_4"}

    def __init__(self):
        self.track_id = 1
        self.detection_count = 0
        self.last_drone_type = ""
        self.was_detected = False
        self.last_fused = None

    def fuse(self, mmaud_data, camera_state, sensor_enabled):
        """Fuse all available sensor data into a single detection result.
        Returns dict with detection info or None if nothing detected."""
        contributors = []
        positions = []
        weights = []
        class_votes = {}

        if mmaud_data:
            if sensor_enabled.get("radar", True):
                r = mmaud_data.get("radar", {})
                if r.get("detected"):
                    contributors.append("radar")
                    if r.get("position"):
                        positions.append(r["position"])
                        weights.append(0.7)
                    for dtype, prob in r.get("class_probs", {}).items():
                        class_votes[dtype] = class_votes.get(dtype, 0) + prob * 0.5

            if sensor_enabled.get("lidar", True):
                for lkey in ("avia", "mid360"):
                    s = mmaud_data.get(lkey, {})
                    if s.get("detected"):
                        contributors.append(f"lidar_{lkey}")
                        w = 1.0 if lkey == "avia" else 0.9
                        if s.get("position"):
                            positions.append(s["position"])
                            weights.append(w)

        if sensor_enabled.get("camera", True) and camera_state.get("drone"):
            contributors.append("camera")
            conf = camera_state.get("confidence", 0)
            drone_type = camera_state.get("drone_type", "")
            if drone_type:
                mm_name = {v: k for k, v in self.MMAUD_TO_CAMERA.items()}.get(drone_type, drone_type)
                class_votes[mm_name] = class_votes.get(mm_name, 0) + conf * 0.8

        if not contributors:
            result = None
            if self.was_detected:
                self.was_detected = False
                self.last_fused = None
                return {"detected": False, "event": "TRACK_LOST",
                        "drone_type": self.last_drone_type, "track_id": self.track_id}
            self.last_fused = None
            return None

        fused_pos = None
        if positions:
            total_w = sum(weights[:len(positions)])
            if total_w > 0:
                fused_pos = {
                    "lat": round(sum(p["lat"] * w for p, w in zip(positions, weights)) / total_w, 7),
                    "lon": round(sum(p["lon"] * w for p, w in zip(positions, weights)) / total_w, 7),
                    "alt": round(sum(p.get("alt", 0) * w for p, w in zip(positions, weights)) / total_w, 1),
                }

        drone_type = ""
        confidence = 0.0
        if class_votes:
            drone_type = max(class_votes, key=class_votes.get)
            total_v = sum(class_votes.values())
            confidence = round(class_votes[drone_type] / total_v, 3) if total_v > 0 else 0
            drone_type = self.MMAUD_TO_CAMERA.get(drone_type, drone_type)

        self.detection_count += 1
        event = None
        if not self.was_detected:
            event = "NEW_DETECTION"
            self.was_detected = True
        elif drone_type and drone_type != self.last_drone_type and self.last_drone_type:
            event = "TYPE_CHANGE"
        self.last_drone_type = drone_type

        result = {
            "detected": True,
            "track_id": self.track_id,
            "drone_type": drone_type,
            "confidence": confidence,
            "position": fused_pos,
            "contributing_sensors": contributors,
            "sensor_count": len(contributors),
            "class_votes": {k: round(v, 3) for k, v in sorted(class_votes.items(), key=lambda x: -x[1])},
            "event": event,
        }
        self.last_fused = result
        return result


# ======================================================== camera ====

def load_frames_for_type(dtype):
    """Load frame list and detections for a drone type."""
    frames_dir = HERE / dtype["frames"]
    det_file = HERE / dtype["detections"]
    if not frames_dir.exists():
        return [], {}, frames_dir
    exts = (".jpg", ".jpeg", ".png")
    frames = [f for f in sorted(os.listdir(frames_dir)) if f.lower().endswith(exts)]
    dets = {}
    if det_file.exists():
        try:
            with open(det_file, "r") as f:
                dets = json.load(f)
        except Exception as e:
            print(f"Could not read {det_file}: {e}")
    return frames, dets, frames_dir


def load_site():
    return json.loads(SITE_FILE.read_text(encoding="utf-8"))


# ======================================================== shared state ====

class EventStore:
    """SQLite event log — events generated only from real detection activity."""

    def __init__(self, path: Path):
        self.lock = threading.Lock()
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.execute("""CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, kind TEXT, level INTEGER,
            level_name TEXT, zone TEXT, message TEXT, lat REAL, lon REAL, alt REAL,
            camera_confirmed INTEGER, source TEXT,
            drone_type TEXT, confidence REAL, contributing_sensors TEXT)""")
        self.db.commit()
        for col, ctype in [("drone_type", "TEXT"), ("confidence", "REAL"), ("contributing_sensors", "TEXT")]:
            try:
                self.db.execute(f"ALTER TABLE events ADD COLUMN {col} {ctype}")
                self.db.commit()
            except sqlite3.OperationalError:
                pass

    def clear_session(self):
        """Remove all events from previous sessions — fresh start each run."""
        with self.lock:
            self.db.execute("DELETE FROM events")
            try:
                self.db.execute("DELETE FROM sqlite_sequence WHERE name='events'")
            except sqlite3.OperationalError:
                pass
            self.db.commit()

    def add(self, ev: dict, pos: dict | None = None, source: str = "geofence") -> dict:
        pos = pos or {}
        row = {"ts": time.time(), "kind": ev.get("kind", "INFO"), "level": int(ev.get("level", 0)),
               "level_name": ev.get("level_name", LEVELS[int(ev.get("level", 0))]),
               "zone": ev.get("zone", ""), "message": ev.get("message", ""),
               "lat": pos.get("lat"), "lon": pos.get("lon"),
               "alt": pos.get("alt_rel", pos.get("alt")),
               "camera_confirmed": int(bool(ev.get("camera_confirmed"))), "source": source,
               "drone_type": ev.get("drone_type", ""),
               "confidence": ev.get("confidence"),
               "contributing_sensors": ev.get("contributing_sensors", "")}
        with self.lock:
            cur = self.db.execute(
                "INSERT INTO events (ts,kind,level,level_name,zone,message,lat,lon,alt,camera_confirmed,source,"
                "drone_type,confidence,contributing_sensors)"
                " VALUES (:ts,:kind,:level,:level_name,:zone,:message,:lat,:lon,:alt,:camera_confirmed,:source,"
                ":drone_type,:confidence,:contributing_sensors)",
                row)
            self.db.commit()
            row["id"] = cur.lastrowid
        return row

    def recent(self, limit: int = 200) -> list[dict]:
        with self.lock:
            cur = self.db.execute("SELECT id,ts,kind,level,level_name,zone,message,lat,lon,alt,"
                                  "camera_confirmed,source,drone_type,confidence,contributing_sensors"
                                  " FROM events ORDER BY id DESC LIMIT ?",
                                  (limit,))
            cols = [c[0] for c in cur.description]
            return [dict(zip(cols, r)) for r in cur.fetchall()]

    def stats(self) -> dict:
        with self.lock:
            rows = self.db.execute("SELECT level_name, COUNT(*) FROM events WHERE kind IN "
                                   "('ZONE_ENTER','LEVEL_CHANGE','DETECTION') GROUP BY level_name").fetchall()
        return {k: v for k, v in rows}


class Hub:
    """Fan-out of JSON messages to every connected dashboard."""

    def __init__(self):
        self.clients: set[asyncio.Queue] = set()

    def broadcast(self, msg: dict):
        if not self.clients:
            return
        text = json.dumps(msg, separators=(",", ":"))
        for q in list(self.clients):
            if q.full():                      # slow client: drop its oldest message
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(text)


class Telemetry:
    """Latest MAVLink vehicle state (updated by the UDP listener)."""

    def __init__(self):
        self.vehicle_sysid = None
        self.autopilot = None
        self.armed = False
        self.pos = None                       # dict lat/lon/alt_msl/alt_rel/vn/ve/vd/heading
        self.groundspeed = 0.0
        self.climb = 0.0
        self.battery = None
        self.gps_fix = None
        self.sats = None
        self.last_seen = 0.0
        self.packets = 0
        self.rate = 0.0
        self.source = ""
        self._rate_t0, self._rate_n = time.time(), 0

    def link_ok(self) -> bool:
        return self.pos is not None and time.time() - self.last_seen < LINK_TIMEOUT_S

    def handle(self, msg, addr):
        if msg.name == "HEARTBEAT":
            if msg.type == MAV_TYPE_GCS:      # another ground station (e.g. QGC itself)
                return
            if self.vehicle_sysid is None:
                self.vehicle_sysid = msg.sysid
                print(f"[mavlink] vehicle sysid {msg.sysid} from {addr[0]}:{addr[1]}")
            if msg.sysid == self.vehicle_sysid:
                self.armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                self.autopilot = {12: "PX4", 3: "ArduPilot"}.get(msg.autopilot, f"autopilot {msg.autopilot}")
        if self.vehicle_sysid is not None and msg.sysid != self.vehicle_sysid:
            return
        self.packets += 1
        self._rate_n += 1
        now = time.time()
        if now - self._rate_t0 >= 2.0:
            self.rate = self._rate_n / (now - self._rate_t0)
            self._rate_t0, self._rate_n = now, 0
        self.source = f"udp {addr[0]}:{addr[1]}"
        if msg.name == "GLOBAL_POSITION_INT":
            self.last_seen = now
            self.pos = {"lat": msg.lat / 1e7, "lon": msg.lon / 1e7, "alt_msl": msg.alt / 1000,
                        "alt_rel": msg.relative_alt / 1000, "vn": msg.vx / 100, "ve": msg.vy / 100,
                        "vd": msg.vz / 100,
                        "heading": (msg.hdg / 100) if msg.hdg != 65535 else None}
            self.climb = -msg.vz / 100
            self.groundspeed = (msg.vx ** 2 + msg.vy ** 2) ** 0.5 / 100
        elif msg.name == "VFR_HUD":
            self.groundspeed, self.climb = msg.groundspeed, msg.climb
        elif msg.name == "SYS_STATUS":
            self.battery = msg.battery_remaining if msg.battery_remaining >= 0 else None
        elif msg.name == "GPS_RAW_INT":
            self.gps_fix, self.sats = msg.fix_type, msg.satellites_visible


class MavlinkUDP(asyncio.DatagramProtocol):
    def __init__(self, telemetry: Telemetry):
        self.t = telemetry
        self.parsers: dict = {}

    def datagram_received(self, data, addr):
        p = self.parsers.setdefault(addr, MavlinkParser())
        for msg in p.feed(data):
            self.t.handle(msg, addr)


class State:
    def __init__(self):
        self.site = load_site()
        self.zones = load_zones(ZONES_FILE)
        self.monitor = GeofenceMonitor(self.zones)           # MAVLink-based target
        self.sensor_monitor = GeofenceMonitor(self.zones)    # sensor-detected targets (radar/LiDAR)
        self.events = EventStore(DB_FILE)
        self.hub = Hub()
        self.tel = Telemetry()
        self.camera = {"drone": False, "confidence": 0.0, "t": 0.0, "frame": None, "drone_type": ""}
        self.geofence_status = None
        self.sensor_geofence_status = None                   # geofence result from sensor detections
        self.link_was_ok = False
        self.track: list = []                 # recent positions for late-joining dashboards
        self.sensor_track: list = []          # track from sensor detections
        self.t0 = time.monotonic()
        self.sim = None
        self.sensor_enabled = {"radar": True, "lidar": True, "audio": True, "camera": True}
        self.fusion = FusionEngine()
        self.data_mode = "DATASET_REPLAY"  # "LIVE" when real hardware connected

        # MMAUD feed (preferred) or legacy individual sensor files
        site = self.site["sensor_site"]
        sens = self.site.get("sensors", {})
        self.mmaud = MMAUDFeedReplay(
            MMAUD_FEED_FILE, site,
            sens.get("radar", {}).get("heading_deg", 0.0)
        )
        # Legacy fallback sensors (used when MMAUD feed is not available)
        self.sensors = {
            name: SensorReplay(name, HERE / sens.get(name, {}).get("file", f"data/{name}/{name}.json"),
                               site, sens.get(name, {}).get("heading_deg", 0.0))
            for name in ("radar", "lidar", "audio")
        }

    def mission(self) -> dict:
        try:
            p = parse_plan(json.loads(MISSION_FILE.read_text(encoding="utf-8")))
        except Exception as e:
            return {"error": str(e), "waypoints": [], "home": None}
        p["file"] = MISSION_FILE.name
        return p


S: State | None = None


# ============================================================ loops ====

async def camera_loop():
    """Cycle through ALL drone type folders — Avata, M300, Mavic_2, Mavic_3, Phantom_4."""
    delay = 1.0 / max(PLAYBACK_FPS, 1)

    # Build the full playlist: [(frame_path, det_dict, drone_type_name, frame_name, global_idx), ...]
    playlist = []
    total_frames = 0
    for dtype in DRONE_TYPES:
        frames, dets, frames_dir = load_frames_for_type(dtype)
        if not frames:
            print(f"[camera] no frames in {dtype['frames']}")
            continue
        print(f"[camera] {len(frames)} frames for {dtype['name']}")
        for fname in frames:
            det = dets.get(fname, {"drone": False, "confidence": 0.0, "box": None})
            playlist.append({
                "path": frames_dir / fname,
                "det": det,
                "drone_type": dtype["name"],
                "frame_name": fname,
            })
        total_frames += len(frames)

    if not playlist:
        print("[camera] no frames found in any drone type folder")
        return

    print(f"[camera] total {total_frames} frames across {sum(1 for d in DRONE_TYPES if (HERE / d['frames']).exists())} drone types at {PLAYBACK_FPS} fps ({total_frames/PLAYBACK_FPS:.1f} s per cycle)")

    while True:
        for i, item in enumerate(playlist):
            if not S.sensor_enabled.get("camera", True):
                await asyncio.sleep(delay)
                continue
            det = item["det"]
            S.camera = {
                "drone": bool(det.get("drone")),
                "confidence": det.get("confidence", 0.0),
                "t": time.time(),
                "frame": item["frame_name"],
                "drone_type": item["drone_type"],
            }
            if S.hub.clients:
                path = item["path"]
                img_b64 = base64.b64encode(path.read_bytes()).decode("ascii")
                mime = "image/png" if path.suffix.lower() == ".png" else "image/jpeg"
                S.hub.broadcast({
                    "type": "frame", "index": i, "total": len(playlist),
                    "frame_name": item["frame_name"],
                    "drone_type": item["drone_type"],
                    "image": f"data:{mime};base64,{img_b64}",
                    "drone": det.get("drone", False),
                    "confidence": det.get("confidence", 0.0),
                    "box": det.get("box", None),
                })
            await asyncio.sleep(delay)


def _camera_confirms() -> bool:
    cam = S.camera
    if not cam["drone"] or time.time() - cam["t"] > 1.0 or not S.tel.pos:
        return False
    s = S.site["sensor_site"]
    rng = geo.haversine_m(s["lat"], s["lon"], S.tel.pos["lat"], S.tel.pos["lon"])
    return rng <= S.site.get("sensors", {}).get("camera", {}).get("range_m", 700)


def _best_sensor_position(mmaud_data, sensor_dets):
    """Pick the best sensor-detected position for geofence checking.

    Priority: fusion > radar > mid360 > avia (based on accuracy).
    Returns (lat, lon, alt) or None if no sensor has a detection with position.
    """
    if mmaud_data:
        # Try fusion first, then radar, then mid360, then avia
        for key in ("fusion", "radar", "mid360", "avia"):
            s = mmaud_data.get(key)
            if s and s.get("detected") and s.get("position"):
                pos = s["position"]
                return pos.get("lat"), pos.get("lon"), pos.get("alt", 0)
    # Fallback: use legacy sensor detections (radar first, then lidar)
    for name in ("radar", "lidar"):
        dets = sensor_dets.get(name, {}).get("detections", [])
        if dets:
            d = dets[0]
            if "lat" in d and "lon" in d:
                return d["lat"], d["lon"], d.get("alt", 0)
    return None


def _fuse_geofence(mav_status, sensor_status):
    """Fuse MAVLink geofence and sensor geofence into one combined status.

    Takes the HIGHER threat level from either source and merges reasons/zones.
    """
    if mav_status is None and sensor_status is None:
        return None
    if mav_status is None:
        return sensor_status
    if sensor_status is None:
        return mav_status

    # Pick higher threat level
    mav_lvl = mav_status.get("level", 0)
    sen_lvl = sensor_status.get("level", 0)

    if sen_lvl > mav_lvl:
        # Sensor detections show higher threat — use sensor status as base
        fused = dict(sensor_status)
        fused["reasons"] = sensor_status.get("reasons", []) + [
            f"(MAVLink: {LEVELS[mav_lvl]})" if mav_lvl > 0 else "(MAVLink: no threat)"]
        fused["source"] = "sensor+mavlink"
        # Merge zone info from both
        mav_zones = {z["id"]: z for z in mav_status.get("zones", [])}
        sen_zones = {z["id"]: z for z in sensor_status.get("zones", [])}
        for zid, zdata in mav_zones.items():
            if zid not in sen_zones:
                fused.setdefault("zones", []).append(zdata)
        return fused
    else:
        # MAVLink shows equal or higher threat — use MAVLink status as base
        fused = dict(mav_status)
        if sen_lvl > 0:
            fused["reasons"] = mav_status.get("reasons", []) + [
                f"Sensor detection: {LEVELS[sen_lvl]}"]
        fused["source"] = "mavlink+sensor" if sen_lvl > 0 else "mavlink"
        return fused


async def telemetry_loop():
    """5 Hz: run the geofence on MAVLink + sensor positions and push to dashboards."""
    period = 1.0 / TELEMETRY_HZ
    last_sensor_push = 0.0
    while True:
        await asyncio.sleep(period)
        tel = S.tel
        ok = tel.link_ok()
        if ok != S.link_was_ok:
            ev = {"kind": "LINK_UP" if ok else "LINK_LOST", "level": 0,
                  "message": (f"MAVLink telemetry connected ({tel.autopilot or 'vehicle'} "
                              f"sysid {tel.vehicle_sysid}, {tel.source})") if ok
                  else "MAVLink telemetry lost"}
            S.hub.broadcast({"type": "event", "event": S.events.add(ev, tel.pos, "mavlink")})
            S.link_was_ok = ok
            if not ok:
                S.geofence_status = None

        msg = {"type": "telemetry", "link": "OK" if ok else ("LOST" if tel.pos else "WAITING"),
               "source": tel.source, "rate_hz": round(tel.rate, 1), "packets": tel.packets,
               "autopilot": tel.autopilot, "sysid": tel.vehicle_sysid, "armed": tel.armed,
               "battery": tel.battery, "gps_fix": tel.gps_fix, "sats": tel.sats,
               "data_mode": S.data_mode,
               "camera": {"drone": S.camera["drone"], "confidence": S.camera["confidence"],
                          "drone_type": S.camera.get("drone_type", "")}}
        if tel.pos:
            p = tel.pos
            site = S.site["sensor_site"]
            e, n, u = geo.geodetic_to_enu(p["lat"], p["lon"], p["alt_rel"], site["lat"], site["lon"],
                                          site.get("height_agl", 0.0))
            rng, az, el = geo.enu_to_polar(e, n, u)
            msg["drone"] = {**p, "groundspeed": round(tel.groundspeed, 2), "climb": round(tel.climb, 2)}
            msg["relative"] = {"range_m": round(rng, 1), "bearing_deg": round(az, 1),
                               "elevation_deg": round(el, 1)}

        # ---- MAVLink geofence (when link is up) ----
        mav_geofence = None
        if ok:
            p = tel.pos
            cam = _camera_confirms()
            sample = TrackSample(time.time(), p["lat"], p["lon"], p["alt_rel"], tel.groundspeed,
                                 tel.climb, p["heading"] or 0.0)
            mav_geofence, events = S.monitor.update(sample, cam)
            for ev in events:
                S.hub.broadcast({"type": "event", "event": S.events.add(ev, p)})
            S.track.append([round(p["lat"], 7), round(p["lon"], 7), round(p["alt_rel"], 1),
                            mav_geofence["level"]])
            del S.track[:-600]

        # ---- Sensor data (MMAUD or legacy) ----
        now = time.monotonic()
        elapsed = now - S.t0
        mmaud_data = None
        sensor_dets = {}
        se = S.sensor_enabled

        if now - last_sensor_push >= 0.2:   # run at 5 Hz (match MMAUD rate)
            last_sensor_push = now

            if S.mmaud.available:
                mmaud_data = S.mmaud.at(elapsed)
                if se.get("radar", True) or se.get("lidar", True):
                    msg["mmaud"] = mmaud_data
                msg["mmaud_status"] = S.mmaud.status()
                sensor_dets = {
                    "radar": {
                        "sensor": "radar", "available": se.get("radar", True),
                        "message": f"MMAUD radar · window {mmaud_data['window_index']}/{mmaud_data['total_windows']}" if se.get("radar", True) else "Radar STOPPED",
                        "detections": S.mmaud.sensor_detections("radar", elapsed) if se.get("radar", True) else [],
                    },
                    "lidar": {
                        "sensor": "lidar", "available": se.get("lidar", True),
                        "message": (f"MMAUD avia+mid360 · window {mmaud_data['window_index']}/{mmaud_data['total_windows']}" if se.get("lidar", True) else "LiDAR STOPPED"),
                        "detections": ((S.mmaud.sensor_detections("avia", elapsed) +
                                       S.mmaud.sensor_detections("mid360", elapsed)) if se.get("lidar", True) else []),
                    },
                    "audio": {
                        "sensor": "audio", "available": se.get("audio", True) and False,
                        "message": ("Audio not in MMAUD feed" if se.get("audio", True) else "Audio STOPPED"),
                        "detections": [],
                    },
                }
                msg["sensors"] = sensor_dets
            else:
                sensor_dets = {}
                for name, r in S.sensors.items():
                    if se.get(name, True):
                        sensor_dets[name] = {**r.status(), "detections": r.at(elapsed)}
                    else:
                        sensor_dets[name] = {"sensor": name, "available": False,
                                             "message": f"{name.title()} STOPPED", "detections": []}
                msg["sensors"] = sensor_dets

        # ---- Multi-sensor fusion ----
        fusion_result = S.fusion.fuse(mmaud_data, S.camera, se)
        if fusion_result:
            msg["fusion"] = fusion_result
            # Generate events from fusion state changes
            ev_type = fusion_result.get("event")
            if ev_type and fusion_result.get("detected"):
                f_pos = fusion_result.get("position") or {}
                sensors_str = ",".join(fusion_result.get("contributing_sensors", []))
                dtype = fusion_result.get("drone_type", "")
                conf = fusion_result.get("confidence", 0)
                if ev_type == "NEW_DETECTION":
                    det_ev = S.events.add({
                        "kind": "DETECTION", "level": 1, "level_name": "ADVISORY",
                        "message": f"Drone detected by {sensors_str} — {dtype} ({conf*100:.0f}% confidence)",
                        "drone_type": dtype, "confidence": conf,
                        "contributing_sensors": sensors_str,
                    }, {"lat": f_pos.get("lat"), "lon": f_pos.get("lon"), "alt_rel": f_pos.get("alt")}, "fusion")
                    S.hub.broadcast({"type": "event", "event": det_ev})
                elif ev_type == "TYPE_CHANGE":
                    det_ev = S.events.add({
                        "kind": "CLASSIFICATION", "level": 1, "level_name": "ADVISORY",
                        "message": f"Drone reclassified: now {dtype} ({conf*100:.0f}%) by {sensors_str}",
                        "drone_type": dtype, "confidence": conf,
                        "contributing_sensors": sensors_str,
                    }, {"lat": f_pos.get("lat"), "lon": f_pos.get("lon"), "alt_rel": f_pos.get("alt")}, "fusion")
                    S.hub.broadcast({"type": "event", "event": det_ev})
            elif fusion_result.get("event") == "TRACK_LOST":
                lost_ev = S.events.add({
                    "kind": "TRACK_LOST", "level": 0,
                    "message": f"Track lost on {fusion_result.get('drone_type', 'drone')}",
                    "drone_type": fusion_result.get("drone_type", ""),
                }, source="fusion")
                S.hub.broadcast({"type": "event", "event": lost_ev})

        # ---- Sensor geofence: check sensor-detected positions against zones ----
        sensor_geofence = None
        any_pos_sensor = se.get("radar", True) or se.get("lidar", True)
        # Use fusion position if available, else fall back to best single sensor
        if fusion_result and fusion_result.get("detected") and fusion_result.get("position"):
            fp = fusion_result["position"]
            best_pos = (fp["lat"], fp["lon"], fp.get("alt", 0))
        else:
            best_pos = _best_sensor_position(mmaud_data, sensor_dets) if any_pos_sensor else None
        if best_pos:
            s_lat, s_lon, s_alt = best_pos
            sensor_sample = TrackSample(time.time(), s_lat, s_lon, s_alt, 0.0, 0.0, 0.0)
            cam_det = S.camera["drone"] and time.time() - S.camera["t"] < 1.0
            sensor_geofence, s_events = S.sensor_monitor.update(sensor_sample, cam_det)
            S.sensor_geofence_status = sensor_geofence
            for ev in s_events:
                ev["source"] = "sensor"
                ev["message"] = f"[Sensor] {ev['message']}"
                # Enrich geofence events with fusion data
                if fusion_result and fusion_result.get("detected"):
                    ev["drone_type"] = fusion_result.get("drone_type", "")
                    ev["confidence"] = fusion_result.get("confidence")
                    ev["contributing_sensors"] = ",".join(fusion_result.get("contributing_sensors", []))
                S.hub.broadcast({"type": "event", "event": S.events.add(
                    ev, {"lat": s_lat, "lon": s_lon, "alt_rel": s_alt}, "sensor")})
            S.sensor_track.append([round(s_lat, 7), round(s_lon, 7), round(s_alt, 1),
                                   sensor_geofence["level"]])
            del S.sensor_track[:-300]

        # ---- Fuse MAVLink + sensor geofence into one status ----
        fused = _fuse_geofence(mav_geofence, sensor_geofence)
        S.geofence_status = fused
        msg["geofence"] = fused

        # Include sensor geofence separately so dashboard can show it
        if sensor_geofence:
            msg["sensor_geofence"] = sensor_geofence

        S.hub.broadcast(msg)


# ============================================================== app ====

@asynccontextmanager
async def lifespan(app):
    global S
    S = State()
    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(
        lambda: MavlinkUDP(S.tel), local_addr=("0.0.0.0", MAVLINK_PORT))
    print(f"[mavlink] listening on udp 0.0.0.0:{MAVLINK_PORT}")
    tasks = [asyncio.create_task(camera_loop()), asyncio.create_task(telemetry_loop())]
    if RUN_SIM:
        from sim_drone import SimDrone
        S.sim = SimDrone(MISSION_FILE, [f"127.0.0.1:{MAVLINK_PORT}"] + SIM_EXTRA_TARGETS)
        threading.Thread(target=S.sim.run, daemon=True).start()
    S.events.clear_session()
    S.events.add({"kind": "SYSTEM", "level": 0,
                  "message": f"AeroSense server started — {S.data_mode} mode"}, source="system")
    try:
        yield
    finally:
        for t in tasks:
            t.cancel()
        if S.sim:
            S.sim.stop()
        transport.close()


SIM_EXTRA_TARGETS: list = []
app = FastAPI(title="AeroSense Server", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])
app.mount("/static", StaticFiles(directory=str(HERE / "static")), name="static")


@app.get("/")
def home():
    return {
        "server": "AeroSense server is running",
        "playback_fps": PLAYBACK_FPS,
        "mavlink_udp_port": MAVLINK_PORT,
        "mavlink_link": "OK" if S and S.tel.link_ok() else "WAITING",
        "mmaud_feed": S.mmaud.available if S else False,
        "try": ["/dashboard", "/health", "/info", "/api/telemetry", "/api/zones", "/api/mission",
                "/api/export/plan", "/api/events", "/ws"],
    }


@app.get("/dashboard")
def dashboard():
    return FileResponse(HERE / "test_dashboard.html")


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/info")
def info():
    return {
        "playback_fps": PLAYBACK_FPS,
        "mmaud_feed": S.mmaud.status() if S else {},
        "sensors": {k: v.status() for k, v in S.sensors.items()} if S else {},
    }


@app.get("/frame/{index}")
def get_frame(index: int):
    # Legacy endpoint: serves Avata frames for backward compat
    frames_dir = HERE / "data" / "test_frames" / "Avata"
    if not frames_dir.exists():
        return JSONResponse({"error": "no frames"}, status_code=404)
    exts = (".jpg", ".jpeg", ".png")
    frames = sorted(f for f in os.listdir(frames_dir) if f.lower().endswith(exts))
    if index < 0 or index >= len(frames):
        return JSONResponse({"error": "frame index out of range"}, status_code=404)
    return FileResponse(frames_dir / frames[index])


@app.get("/detection/{index}")
def get_detection(index: int):
    det_file = HERE / "data" / "avata_detections.json"
    frames_dir = HERE / "data" / "test_frames" / "Avata"
    exts = (".jpg", ".jpeg", ".png")
    frames = sorted(f for f in os.listdir(frames_dir) if f.lower().endswith(exts)) if frames_dir.exists() else []
    if index < 0 or index >= len(frames):
        return JSONResponse({"error": "frame index out of range"}, status_code=404)
    name = frames[index]
    dets = {}
    if det_file.exists():
        try:
            dets = json.loads(det_file.read_text())
        except Exception:
            pass
    return {"index": index, "frame": name,
            **dets.get(name, {"drone": False, "confidence": 0.0, "box": None})}


# ---------- mission / geofence API ----------

@app.get("/api/site")
def api_site():
    return S.site


@app.get("/api/telemetry")
def api_telemetry():
    t = S.tel
    return {"link": "OK" if t.link_ok() else "WAITING", "source": t.source, "position": t.pos,
            "groundspeed": t.groundspeed, "climb": t.climb, "armed": t.armed,
            "autopilot": t.autopilot, "geofence": S.geofence_status}


@app.get("/api/zones")
def api_zones():
    return {"zones": [z.to_api() for z in S.zones]}


@app.put("/api/zones")
async def api_put_zones(request: Request):
    body = await request.json()
    try:
        zones = zones_from_api(body.get("zones", []))
    except (ValueError, TypeError, KeyError) as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    save_zones(ZONES_FILE, zones, S.site.get("site_name", ""))
    S.zones = zones
    S.monitor.set_zones(zones)
    S.sensor_monitor.set_zones(zones)
    S.hub.broadcast({"type": "zones", "zones": [z.to_api() for z in zones]})
    S.hub.broadcast({"type": "event", "event": S.events.add(
        {"kind": "CONFIG", "level": 0, "message": f"Geofence updated: {len(zones)} zones"},
        source="operator")})
    return {"ok": True, "zones": [z.to_api() for z in zones]}


@app.get("/api/mission")
def api_mission():
    return S.mission()


@app.put("/api/mission")
async def api_put_mission(request: Request):
    """Accepts a QGroundControl .plan JSON, or {"waypoints":[{lat,lon,alt}], "cruise_speed": 12}."""
    body = await request.json()
    try:
        if body.get("fileType") == "Plan":
            parsed = parse_plan(body)
            plan = body
        else:
            wps = [{"lat": float(w["lat"]), "lon": float(w["lon"]), "alt": float(w["alt"])}
                   for w in body.get("waypoints") or []]
            if len(wps) < 2:
                raise ValueError("mission needs at least 2 waypoints")
            home = body.get("home") or [wps[0]["lat"], wps[0]["lon"], S.site.get("ground_alt_msl", 0)]
            plan = build_plan(wps, home, S.zones, float(body.get("cruise_speed", 12)))
            parsed = parse_plan(plan)
        if len(parsed["waypoints"]) < 2:
            raise ValueError("mission needs at least 2 waypoints")
    except (ValueError, TypeError, KeyError, IndexError) as e:
        return JSONResponse({"error": str(e)}, status_code=400)
    MISSION_FILE.write_text(json.dumps(plan, indent=2), encoding="utf-8")
    S.hub.broadcast({"type": "mission", "mission": S.mission()})
    S.hub.broadcast({"type": "event", "event": S.events.add(
        {"kind": "CONFIG", "level": 0,
         "message": f"Mission updated: {len(parsed['waypoints'])} waypoints"}, source="operator")})
    return {"ok": True, "mission": S.mission()}


@app.get("/api/export/plan")
def api_export_plan():
    """Current mission + current zones as a QGroundControl .plan (open it in QGC: Plan → Open)."""
    m = S.mission()
    plan = build_plan([w for w in m["waypoints"] if w.get("command") != 22], m["home"], S.zones,
                      m.get("cruise_speed", 12.0))
    return Response(json.dumps(plan, indent=2), media_type="application/json",
                    headers={"Content-Disposition": 'attachment; filename="aerosense_fast_campus.plan"'})


@app.get("/api/events")
def api_events(limit: int = 200):
    return {"events": S.events.recent(min(int(limit), 1000)), "stats": S.events.stats()}


@app.post("/api/operator-action")
async def api_operator_action(request: Request):
    body = await request.json()
    action = str(body.get("action", ""))[:120]
    ev = S.events.add({"kind": "OPERATOR_ACTION", "level": 0, "message": f"Operator: {action}"},
                      S.tel.pos, source="operator")
    S.hub.broadcast({"type": "event", "event": ev})
    return {"ok": True}


# ---------- sensor control API ----------

VALID_SENSORS = ("radar", "lidar", "audio", "camera")


@app.get("/api/sensors/status")
def api_sensors_status():
    return {"sensors": dict(S.sensor_enabled)}


@app.post("/api/sensors/toggle")
async def api_sensors_toggle(request: Request):
    body = await request.json()
    sensor = str(body.get("sensor", ""))
    if sensor not in VALID_SENSORS:
        return JSONResponse({"error": f"unknown sensor {sensor!r}"}, status_code=400)
    enabled = bool(body.get("enabled", not S.sensor_enabled.get(sensor, True)))
    S.sensor_enabled[sensor] = enabled
    ev = S.events.add(
        {"kind": "SENSOR_CONTROL", "level": 0,
         "message": f"Sensor {sensor} {'started' if enabled else 'stopped'} by operator"},
        source="operator")
    S.hub.broadcast({"type": "event", "event": ev})
    S.hub.broadcast({"type": "sensor_state", "states": dict(S.sensor_enabled)})
    return {"ok": True, "sensor": sensor, "enabled": enabled}


def _handle_sensor_toggle(sensor: str, enabled: bool | None):
    if sensor not in VALID_SENSORS:
        return
    if enabled is None:
        enabled = not S.sensor_enabled.get(sensor, True)
    S.sensor_enabled[sensor] = enabled
    ev = S.events.add(
        {"kind": "SENSOR_CONTROL", "level": 0,
         "message": f"Sensor {sensor} {'started' if enabled else 'stopped'} by operator"},
        source="operator")
    S.hub.broadcast({"type": "event", "event": ev})
    S.hub.broadcast({"type": "sensor_state", "states": dict(S.sensor_enabled)})


# ---------- WebSocket: everything live ----------

@app.websocket("/ws")
async def stream(websocket: WebSocket):
    await websocket.accept()
    q: asyncio.Queue = asyncio.Queue(maxsize=60)
    await websocket.send_text(json.dumps({
        "type": "hello", "site": S.site, "zones": [z.to_api() for z in S.zones],
        "mission": S.mission(), "track": S.track[-300:],
        "sensor_track": S.sensor_track[-200:],
        "events": S.events.recent(100),
        "mmaud_available": S.mmaud.available,
        "sensor_states": dict(S.sensor_enabled),
        "data_mode": S.data_mode,
    }))
    S.hub.clients.add(q)
    print(f"Dashboard connected ({len(S.hub.clients)} total).")

    async def writer():
        while True:
            await websocket.send_text(await q.get())

    async def reader():
        while True:
            raw = await websocket.receive_text()
            try:
                cmd = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
            if cmd.get("type") == "sensor_toggle":
                sensor = str(cmd.get("sensor", ""))
                enabled = cmd.get("enabled")
                if isinstance(enabled, bool):
                    _handle_sensor_toggle(sensor, enabled)
                else:
                    _handle_sensor_toggle(sensor, None)

    tasks = [asyncio.create_task(writer()), asyncio.create_task(reader())]
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for t in done:                         # normal end = client closed the socket
            if not t.cancelled() and t.exception() and \
                    not isinstance(t.exception(), (WebSocketDisconnect, RuntimeError, OSError)):
                print("WebSocket error:", repr(t.exception()))
    finally:
        for t in tasks:
            t.cancel()
        S.hub.clients.discard(q)
        print("Dashboard disconnected.")


if __name__ == "__main__":
    import uvicorn

    ap = argparse.ArgumentParser(description="AeroSense server")
    ap.add_argument("--sim", action="store_true", help="also run the built-in virtual drone")
    ap.add_argument("--sim-also-to", action="append", default=[],
                    help="extra host:port for the virtual drone, e.g. 127.0.0.1:14550 for QGC")
    ap.add_argument("--fps", type=int, default=PLAYBACK_FPS)
    ap.add_argument("--mavlink-port", type=int, default=MAVLINK_PORT)
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    RUN_SIM = args.sim or RUN_SIM
    SIM_EXTRA_TARGETS[:] = args.sim_also_to
    PLAYBACK_FPS = args.fps
    MAVLINK_PORT = args.mavlink_port
    # host="0.0.0.0" lets OTHER laptops on the same WiFi open the dashboard.
    uvicorn.run(app, host="0.0.0.0", port=args.port)