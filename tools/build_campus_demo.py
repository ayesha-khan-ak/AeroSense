"""
Generate the default FAST-NUCES Islamabad scenario:
  data/site_config.json                 sensor site + map centre
  data/geofence/fast_campus_zones.json  3 zones with altitude bands (GeoJSON)
  data/missions/fast_intruder_demo.plan QGroundControl plan for the intruder flight

Run from the repo root:   python tools/build_campus_demo.py
(Only needed if you want to regenerate; the files are already committed.)

IMPORTANT: the campus polygon here is an APPROXIMATE square of ~15 acres
around the campus centre.  Open the dashboard → "Mission & Geo-fence Planner"
on a laptop with internet (satellite imagery), drag the vertices onto the real
campus boundary and press "Save zones".
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import geo                       # noqa: E402
from geofence import Zone, save_zones   # noqa: E402
from qgc_plan import build_plan  # noqa: E402

# FAST-NUCES Islamabad, A.K. Brohi Road, H-11/4 (approximate campus centre)
CENTER_LAT, CENTER_LON = 33.65645, 73.01641
GROUND_MSL = 510.0               # approx. ground elevation (m) — only used for MSL display


def ll(e, n):
    lat, lon, _ = geo.enu_to_geodetic(e, n, 0, CENTER_LAT, CENTER_LON)
    return [round(lat, 7), round(lon, 7)]


def rect(e0, n0, e1, n1):
    return [ll(e0, n1), ll(e1, n1), ll(e1, n0), ll(e0, n0)]   # clockwise


def main():
    (ROOT / "data" / "geofence").mkdir(parents=True, exist_ok=True)
    (ROOT / "data" / "missions").mkdir(parents=True, exist_ok=True)

    sensor = ll(90, -100)
    site = {
        "site_name": "FAST-NUCES Islamabad Campus (H-11/4)",
        "map_center": [CENTER_LAT, CENTER_LON],
        "map_zoom": 16,
        "ground_alt_msl": GROUND_MSL,
        "sensor_site": {"name": "AeroSense sensor mast", "lat": sensor[0], "lon": sensor[1],
                        "height_agl": 15.0},
        "sensors": {
            "camera": {"fov_deg": 60, "range_m": 700},
            "radar": {"heading_deg": 0.0, "range_m": 1000, "file": "data/radar/radar_tracks.json"},
            "lidar": {"heading_deg": 0.0, "range_m": 300, "file": "data/lidar/lidar_tracks.json"},
            "audio": {"file": "data/audio/audio_detections.json"},
        },
        "note": "Campus polygon is approximate. Adjust it in the dashboard planner over satellite imagery.",
    }
    (ROOT / "data" / "site_config.json").write_text(json.dumps(site, indent=2), encoding="utf-8")

    zones = [
        Zone(name="Campus Buffer (Monitoring)", kind="monitoring",
             polygon=rect(-300, -300, 300, 300), min_alt=0, max_alt=200, id="buffer"),
        Zone(name="FAST Campus Restricted Airspace", kind="restricted",
             polygon=rect(-125, -125, 125, 125), min_alt=0, max_alt=120, id="campus"),
        Zone(name="Academic Block No-Fly Core", kind="critical",
             polygon=rect(-45, -25, 45, 45), min_alt=0, max_alt=60, id="core"),
    ]
    save_zones(ROOT / "data" / "geofence" / "fast_campus_zones.json", zones, site["site_name"])

    # Intruder scenario (metres E/N of campus centre, altitude above ground):
    #  high approach above the 120 m ceiling (ADVISORY) -> descent into campus
    #  airspace (WARNING -> BREACH) -> over the academic block below 60 m
    #  (CRITICAL) -> exits north-east (BREACH -> ADVISORY -> CLEAR) -> loops back
    route = [(-650, -520, 30), (-420, -330, 160), (-250, -200, 160), (-60, -60, 160),
             (0, 0, 70), (10, 20, 40), (60, 80, 45), (200, 220, 90), (450, 420, 120),
             (520, -420, 150), (-650, -520, 60)]
    wps = [{"lat": ll(e, n)[0], "lon": ll(e, n)[1], "alt": a} for e, n, a in route]
    home = ll(-650, -520) + [GROUND_MSL]
    plan = build_plan(wps, home, zones, cruise_speed=12.0)
    (ROOT / "data" / "missions" / "fast_intruder_demo.plan").write_text(
        json.dumps(plan, indent=2), encoding="utf-8")
    print("Wrote site_config.json, fast_campus_zones.json, fast_intruder_demo.plan")


if __name__ == "__main__":
    main()
