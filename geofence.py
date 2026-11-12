"""
AeroSense — geofence.py
3D geo-fencing + threat classification for the "protect FAST campus" mission.

A zone is a polygon on the map PLUS an altitude band (metres above ground):

    restricted  FAST Campus airspace         0 – 120 m AGL
    critical    Academic block no-fly core   0 –  60 m AGL
    monitoring  Campus buffer (approach)     0 – 200 m AGL

So the same drone can be harmless at 300 m (over-flight above the ceiling)
and a BREACH at 40 m over the same spot — the "ground level vs sky level"
requirement.

Threat levels (decision support only — the system never claims to stop a drone):

    0 CLEAR     nothing in or near protected airspace
    1 ADVISORY  inside the monitoring buffer, or over-flying above the ceiling
    2 WARNING   closing in on restricted airspace (distance / time-to-breach),
                or descending towards its ceiling
    3 BREACH    inside a restricted zone's polygon AND altitude band
    4 CRITICAL  inside a critical (no-fly core) zone's polygon AND band

Zones are stored as GeoJSON (opens in geojson.io / QGIS) and can be exported
as QGroundControl .plan geofence polygons (see qgc_plan.py).
"""
from __future__ import annotations

import json
import uuid
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

import geo

LEVELS = ["CLEAR", "ADVISORY", "WARNING", "BREACH", "CRITICAL"]
ZONE_KINDS = ("monitoring", "restricted", "critical")
DEFAULT_COLORS = {"monitoring": "#ffa502", "restricted": "#ff4757", "critical": "#c039ff"}

RECOMMENDED_ACTION = {
    0: "Continue surveillance.",
    1: "Track target and log observation.",
    2: "Alert security desk, slew camera to target, prepare response.",
    3: "Restricted airspace violated — notify security, record evidence, search for the operator.",
    4: "Critical asset over-flight — recommend area lockdown / clear personnel (decision support).",
}


# ------------------------------------------------------------------- zones --

@dataclass
class Zone:
    name: str
    kind: str
    polygon: list            # [[lat, lon], ...] (not closed)
    min_alt: float = 0.0     # metres AGL
    max_alt: float = 120.0   # metres AGL
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    color: str = ""

    def __post_init__(self):
        if self.kind not in ZONE_KINDS:
            raise ValueError(f"zone kind must be one of {ZONE_KINDS}, got {self.kind!r}")
        if len(self.polygon) < 3:
            raise ValueError(f"zone {self.name!r} needs at least 3 vertices")
        pts = []
        for p in self.polygon:
            lat, lon = float(p[0]), float(p[1])
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError(f"bad coordinate {p} in zone {self.name!r}")
            pts.append([lat, lon])
        # drop a closing vertex if the caller passed a closed ring
        if len(pts) > 3 and pts[0] == pts[-1]:
            pts.pop()
        self.polygon = pts
        self.min_alt, self.max_alt = float(self.min_alt), float(self.max_alt)
        if self.max_alt <= self.min_alt:
            raise ValueError(f"zone {self.name!r}: max_alt must be greater than min_alt")
        if not self.color:
            self.color = DEFAULT_COLORS[self.kind]

    # GeoJSON uses [lon, lat] order — convert carefully.
    def to_feature(self) -> dict:
        ring = [[p[1], p[0]] for p in self.polygon]
        ring.append(ring[0])
        return {
            "type": "Feature",
            "id": self.id,
            "properties": {"name": self.name, "kind": self.kind, "min_alt_m": self.min_alt,
                           "max_alt_m": self.max_alt, "color": self.color},
            "geometry": {"type": "Polygon", "coordinates": [ring]},
        }

    @classmethod
    def from_feature(cls, f: dict) -> "Zone":
        props = f.get("properties", {})
        ring = f["geometry"]["coordinates"][0]
        return cls(name=props.get("name", "Zone"), kind=props.get("kind", "restricted"),
                   polygon=[[c[1], c[0]] for c in ring],
                   min_alt=props.get("min_alt_m", 0.0), max_alt=props.get("max_alt_m", 120.0),
                   id=str(f.get("id") or uuid.uuid4().hex[:8]), color=props.get("color", ""))

    def to_api(self) -> dict:
        return asdict(self)


def zones_from_api(items: list) -> list[Zone]:
    """Validate a list of zone dicts coming from the dashboard."""
    zones = []
    for z in items:
        zones.append(Zone(name=str(z.get("name", "Zone"))[:60], kind=z.get("kind", "restricted"),
                          polygon=z.get("polygon", []), min_alt=z.get("min_alt", 0),
                          max_alt=z.get("max_alt", 120), id=str(z.get("id") or uuid.uuid4().hex[:8]),
                          color=str(z.get("color", ""))[:9]))
    return zones


def load_zones(path: Path) -> list[Zone]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return [Zone.from_feature(f) for f in data.get("features", [])]


def save_zones(path: Path, zones: list[Zone], site_name: str = "") -> None:
    fc = {"type": "FeatureCollection", "name": site_name or "AeroSense zones",
          "features": [z.to_feature() for z in zones]}
    tmp = Path(path).with_suffix(".tmp")
    tmp.write_text(json.dumps(fc, indent=2), encoding="utf-8")
    tmp.replace(path)


# --------------------------------------------------------------- monitoring --

@dataclass
class TrackSample:
    t: float            # seconds (monotonic or epoch)
    lat: float
    lon: float
    alt_rel: float      # metres above ground / home
    groundspeed: float = 0.0
    climb: float = 0.0  # m/s, positive = going up
    heading: float = 0.0


class GeofenceMonitor:
    """
    Feed it one TrackSample at a time; it returns the current status and any
    events (zone entry/exit, threat-level changes).

    Zone membership only flips after `confirm_samples` consecutive samples to
    stop GPS jitter on a boundary from spamming enter/exit events.
    """

    def __init__(self, zones: list[Zone], warn_distance_m: float = 60.0,
                 warn_time_s: float = 20.0, confirm_samples: int = 2,
                 vertical_margin_m: float = 30.0):
        self.warn_distance_m = warn_distance_m
        self.warn_time_s = warn_time_s
        self.vertical_margin_m = vertical_margin_m
        self.confirm_samples = max(1, confirm_samples)
        self.set_zones(zones)

    def set_zones(self, zones: list[Zone]) -> None:
        self.zones = list(zones)
        self._inside = {z.id: False for z in self.zones}
        self._pending = {z.id: 0 for z in self.zones}
        self._prev_dist: dict[str, float] = {}
        self._closing: dict[str, float] = {}
        self._prev_t: Optional[float] = None
        self.level = 0

    def update(self, s: TrackSample, camera_detected: bool = False) -> tuple[dict, list[dict]]:
        events: list[dict] = []
        dt = None if self._prev_t is None else max(1e-3, s.t - self._prev_t)
        self._prev_t = s.t

        zone_rows = []
        for z in self.zones:
            inside_2d = geo.point_in_polygon(s.lat, s.lon, z.polygon)
            sd = geo.signed_distance_m(s.lat, s.lon, z.polygon)       # <0 inside
            in_band = z.min_alt <= s.alt_rel <= z.max_alt
            raw_inside = inside_2d and in_band

            # closing speed towards the zone boundary (EMA-smoothed, m/s, >0 = approaching)
            prev = self._prev_dist.get(z.id)
            if prev is not None and dt is not None:
                inst = (prev - sd) / dt
                self._closing[z.id] = 0.6 * self._closing.get(z.id, inst) + 0.4 * inst
            self._prev_dist[z.id] = sd
            closing = self._closing.get(z.id, 0.0)

            # hysteresis
            if raw_inside != self._inside[z.id]:
                self._pending[z.id] += 1
                if self._pending[z.id] >= self.confirm_samples:
                    self._inside[z.id] = raw_inside
                    self._pending[z.id] = 0
                    events.append(self._zone_event(z, raw_inside, s, camera_detected))
            else:
                self._pending[z.id] = 0

            if inside_2d and s.alt_rel > z.max_alt:
                vert = "ABOVE"
            elif inside_2d and s.alt_rel < z.min_alt:
                vert = "BELOW"
            else:
                vert = "IN BAND" if in_band else "OUT"
            zone_rows.append({
                "id": z.id, "name": z.name, "kind": z.kind,
                "min_alt": z.min_alt, "max_alt": z.max_alt,
                "inside_2d": inside_2d, "inside": self._inside[z.id],
                "vertical": vert,
                "distance_m": round(sd, 1),
                "closing_mps": round(closing, 2),
                "ceiling_margin_m": round(s.alt_rel - z.max_alt, 1),
            })

        level, reasons, ttb = self._classify(s, zone_rows)
        if level != self.level:
            events.append({
                "kind": "LEVEL_CHANGE", "level": level, "level_name": LEVELS[level],
                "zone": "", "message": f"Threat level {LEVELS[self.level]} → {LEVELS[level]}"
                                       + (f" ({reasons[0]})" if reasons else ""),
                "camera_confirmed": camera_detected,
            })
            self.level = level

        status = {
            "level": level,
            "level_name": LEVELS[level],
            "reasons": reasons,
            "recommended_action": RECOMMENDED_ACTION[level],
            "time_to_breach_s": ttb,
            "zones": zone_rows,
            "camera_confirmed": bool(camera_detected and level >= 1),
        }
        return status, events

    # ------------------------------------------------------------ helpers --
    def _zone_event(self, z: Zone, entered: bool, s: TrackSample, cam: bool) -> dict:
        verb = "ENTERED" if entered else "EXITED"
        lvl = {"critical": 4, "restricted": 3, "monitoring": 1}[z.kind] if entered else 0
        return {
            "kind": "ZONE_ENTER" if entered else "ZONE_EXIT",
            "zone": z.name, "zone_kind": z.kind, "level": lvl, "level_name": LEVELS[lvl],
            "message": f"Drone {verb} {z.name} [{z.min_alt:.0f}–{z.max_alt:.0f} m AGL] "
                       f"at {s.alt_rel:.0f} m" + (" · camera-confirmed" if cam else ""),
            "camera_confirmed": cam,
        }

    def _classify(self, s: TrackSample, rows: list[dict]) -> tuple[int, list[str], Optional[float]]:
        found: list[tuple[int, str]] = []
        ttb = None
        for r in rows:
            protected = r["kind"] in ("restricted", "critical")
            if r["inside"]:
                lvl = {"critical": 4, "restricted": 3, "monitoring": 1}[r["kind"]]
                found.append((lvl, f"inside {r['name']} at {s.alt_rel:.0f} m AGL"))
                continue
            if not protected:
                continue
            if r["inside_2d"] and r["vertical"] == "IN BAND":
                # crossing the boundary; membership not yet confirmed by hysteresis
                found.append((2, f"crossing into {r['name']}"))
                continue
            # horizontal approach — only if the drone is at (or near) the zone's altitude band
            near_band = s.alt_rel <= r["max_alt"] + self.vertical_margin_m
            if r["distance_m"] > 0 and r["closing_mps"] > 0.3 and near_band:
                t = r["distance_m"] / r["closing_mps"]
                if r["distance_m"] <= self.warn_distance_m or t <= self.warn_time_s:
                    ttb = round(t, 1) if ttb is None else min(ttb, round(t, 1))
                    found.append((2, f"{r['distance_m']:.0f} m from {r['name']}, closing at "
                                     f"{r['closing_mps']:.1f} m/s"))
            if r["vertical"] == "ABOVE":
                if s.climb < -0.5 and r["ceiling_margin_m"] < self.vertical_margin_m:
                    found.append((2, f"descending towards {r['name']} ceiling "
                                     f"({r['ceiling_margin_m']:.0f} m above it)"))
                else:
                    found.append((1, f"over-flying {r['name']} above its "
                                     f"{r['max_alt']:.0f} m ceiling"))
        found.sort(key=lambda x: -x[0])
        level = found[0][0] if found else 0
        return level, [txt for _, txt in found], ttb
