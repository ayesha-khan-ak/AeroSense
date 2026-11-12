"""
AeroSense — sensor_adapters.py
Plug-in point for the radar, LiDAR and audio models (teammates' work).

Each model writes ONE JSON file (the "contract"), the same idea as
data/detections.json for the camera.  The server replays it on the same clock
as everything else, converts positions onto the FAST campus map and pushes it
to the dashboard.

  data/radar/radar_tracks.json
  data/lidar/lidar_tracks.json
  data/audio/audio_detections.json

Contract
--------
{
  "sensor": "radar",                    # radar | lidar | audio
  "coordinate_frame": "sensor_xyz",     # sensor_xyz | polar | enu | wgs84 | none (audio)
  "detections": [
     {"t": 0.00, "x": 41.2, "y": -3.5, "z": 18.0, "confidence": 0.93, "track_id": 1},
     ...
  ]
}

  t                 seconds from the start of the sequence (MMAUD timestamps minus the first one)
  sensor_xyz        x forward, y left, z up (metres) in the sensor's own frame
                    (MMAUD LiDAR/radar convention). The sensor's compass heading
                    comes from site_config.json -> sensors.<name>.heading_deg
  polar             "range" (m), "azimuth" (deg from north, clockwise), "elevation" (deg)
  enu               "east", "north", "up" (m) from the sensor mast
  wgs84             "lat", "lon", "alt" (alt = metres above ground)
  audio             no position: {"t":..,"drone":true,"confidence":0.88,"label":"Mavic"}

Use tools/convert_sensor_output.py to turn a CSV from the models into this file.
"""
from __future__ import annotations

import json
from pathlib import Path

import geo

FRAMES = ("sensor_xyz", "polar", "enu", "wgs84", "none")


class SensorReplay:
    def __init__(self, name: str, path: Path, site: dict, heading_deg: float = 0.0,
                 window_s: float = 0.5):
        self.name = name
        self.path = Path(path)
        self.site = site                      # {"lat","lon","height_agl"}
        self.heading = heading_deg
        self.window = window_s
        self.detections: list[dict] = []
        self.frame = "none"
        self.duration = 0.0
        self.error = ""
        self.load()

    @property
    def available(self) -> bool:
        return bool(self.detections)

    def load(self) -> None:
        self.detections, self.error = [], ""
        if not self.path.exists():
            self.error = f"no data file ({'/'.join(self.path.parts[-3:])})"
            return
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            self.frame = data.get("coordinate_frame", "sensor_xyz" if self.name != "audio" else "none")
            if self.frame not in FRAMES:
                raise ValueError(f"unknown coordinate_frame {self.frame!r}")
            dets = sorted(data.get("detections", []), key=lambda d: float(d["t"]))
            self.detections = [self._to_map(d) for d in dets]
            self.duration = (self.detections[-1]["t"] + 1.0) if self.detections else 0.0
        except Exception as e:                # bad file must never crash the server
            self.error = f"could not load: {e}"
            self.detections = []

    def _to_map(self, d: dict) -> dict:
        out = {"t": float(d["t"]), "confidence": float(d.get("confidence", 1.0)),
               "track_id": d.get("track_id"), "label": d.get("label", "drone"),
               "drone": bool(d.get("drone", True))}
        if self.frame == "none":
            return out
        s = self.site
        h0 = float(s.get("height_agl", 0.0))
        if self.frame == "sensor_xyz":
            e, n, u = geo.sensor_xyz_to_enu(float(d["x"]), float(d["y"]), float(d["z"]), self.heading)
        elif self.frame == "polar":
            e, n, u = geo.polar_to_enu(float(d["range"]), float(d["azimuth"]), float(d["elevation"]))
        elif self.frame == "enu":
            e, n, u = float(d["east"]), float(d["north"]), float(d["up"])
        else:  # wgs84
            e, n, u = geo.geodetic_to_enu(float(d["lat"]), float(d["lon"]), float(d["alt"]),
                                          s["lat"], s["lon"], h0)
        lat, lon, alt = geo.enu_to_geodetic(e, n, u, s["lat"], s["lon"], h0)
        rng, az, el = geo.enu_to_polar(e, n, u)
        out.update({"lat": lat, "lon": lon, "alt": alt, "range": rng, "azimuth": az, "elevation": el})
        return out

    def at(self, elapsed_s: float) -> list[dict]:
        """Detections inside the replay window ending at `elapsed_s` (loops forever)."""
        if not self.detections:
            return []
        t = elapsed_s % self.duration
        return [d for d in self.detections if t - self.window < d["t"] <= t]

    def status(self) -> dict:
        return {"sensor": self.name, "available": self.available, "frame": self.frame,
                "count": len(self.detections), "duration_s": round(self.duration, 1),
                "message": self.error or f"{len(self.detections)} detections loaded"}
