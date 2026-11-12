import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import geo  # noqa: E402
from geofence import Zone, GeofenceMonitor, TrackSample, load_zones, save_zones, zones_from_api  # noqa: E402

LAT0, LON0 = 33.65645, 73.01641


def square(half_m, e0=0.0, n0=0.0):
    pts = []
    for e, n in [(-1, 1), (1, 1), (1, -1), (-1, -1)]:
        lat, lon, _ = geo.enu_to_geodetic(e0 + e * half_m, n0 + n * half_m, 0, LAT0, LON0)
        pts.append([lat, lon])
    return pts


def at(e, n):
    lat, lon, _ = geo.enu_to_geodetic(e, n, 0, LAT0, LON0)
    return lat, lon


# ------------------------------------------------------------------ geo --

def test_enu_roundtrip_is_centimetre_accurate():
    for e, n, u in [(0, 0, 0), (500, -300, 120), (-2000, 1500, 10)]:
        lat, lon, alt = geo.enu_to_geodetic(e, n, u, LAT0, LON0, 510)
        e2, n2, u2 = geo.geodetic_to_enu(lat, lon, alt, LAT0, LON0, 510)
        assert abs(e - e2) < 0.01 and abs(n - n2) < 0.01 and abs(u - u2) < 1e-9


def test_enu_agrees_with_haversine():
    lat, lon = at(300, 400)
    assert geo.haversine_m(LAT0, LON0, lat, lon) == pytest.approx(500, abs=1.0)
    assert geo.bearing_deg(LAT0, LON0, lat, lon) == pytest.approx(36.87, abs=0.2)


def test_point_in_polygon_and_distance():
    poly = square(100)
    assert geo.point_in_polygon(*at(0, 0), poly)
    assert geo.point_in_polygon(*at(95, -95), poly)
    assert not geo.point_in_polygon(*at(105, 0), poly)
    assert geo.signed_distance_m(*at(150, 0), poly) == pytest.approx(50, abs=0.5)
    assert geo.signed_distance_m(*at(80, 0), poly) == pytest.approx(-20, abs=0.5)


def test_concave_polygon():
    # "U" shape: the notch (0, 50) is outside
    pts = [at(-100, 100), at(-40, 100), at(-40, 0), at(40, 0), at(40, 100), at(100, 100),
           at(100, -100), at(-100, -100)]
    poly = [[p[0], p[1]] for p in pts]
    assert not geo.point_in_polygon(*at(0, 50), poly)
    assert geo.point_in_polygon(*at(70, 50), poly)
    assert geo.point_in_polygon(*at(0, -50), poly)


def test_sensor_frame_conversion():
    # sensor x-axis pointing east (heading 90): forward 10 m -> east 10 m; left 5 m -> north 5 m
    e, n, u = geo.sensor_xyz_to_enu(10, 5, 2, 90)
    assert (e, n, u) == pytest.approx((10, 5, 2), abs=1e-9)
    r, az, el = geo.enu_to_polar(*geo.polar_to_enu(300, 45, 10))
    assert (r, az, el) == pytest.approx((300, 45, 10), abs=1e-6)


# ------------------------------------------------------------- zones --

def test_zone_validation():
    with pytest.raises(ValueError):
        Zone("x", "restricted", square(10)[:2])
    with pytest.raises(ValueError):
        Zone("x", "weird", square(10))
    with pytest.raises(ValueError):
        Zone("x", "restricted", square(10), min_alt=100, max_alt=50)
    z = Zone("x", "restricted", square(10) + [square(10)[0]])   # closed ring is accepted
    assert len(z.polygon) == 4


def test_geojson_roundtrip(tmp_path):
    zones = [Zone("Campus", "restricted", square(125), 0, 120, id="c"),
             Zone("Core", "critical", square(40), 0, 60, id="k")]
    f = tmp_path / "z.json"
    save_zones(f, zones)
    data = json.loads(f.read_text())
    ring = data["features"][0]["geometry"]["coordinates"][0]
    assert ring[0] == ring[-1] and ring[0][0] > 70          # [lon, lat] order, closed ring
    back = load_zones(f)
    assert [(z.id, z.kind, z.max_alt) for z in back] == [("c", "restricted", 120), ("k", "critical", 60)]
    assert [c for p in back[0].polygon for c in p] == pytest.approx([c for p in zones[0].polygon for c in p])
    assert zones_from_api([z.to_api() for z in back])[1].name == "Core"


def test_repo_zone_file_loads():
    zones = load_zones(ROOT / "data/geofence/fast_campus_zones.json")
    kinds = {z.kind for z in zones}
    assert kinds == {"monitoring", "restricted", "critical"}
    campus = next(z for z in zones if z.kind == "restricted")
    assert 50_000 < geo.polygon_area_m2(campus.polygon) < 80_000    # ~15 acres


# ------------------------------------------------------- monitor logic --

def make_monitor():
    return GeofenceMonitor([
        Zone("Buffer", "monitoring", square(300), 0, 200, id="b"),
        Zone("Campus", "restricted", square(125), 0, 120, id="c"),
        Zone("Core", "critical", square(40), 0, 60, id="k"),
    ])


def feed(mon, pts, cam=False):
    out = []
    for i, (e, n, alt) in enumerate(pts):
        status, events = mon.update(TrackSample(i * 0.2, *at(e, n), alt), cam)
        out.append((status, events))
    return out


def test_altitude_band_overflight_is_not_a_breach():
    mon = make_monitor()
    res = feed(mon, [(0, 0, 300)] * 4)          # right above the core, but at 300 m
    status = res[-1][0]
    assert status["level_name"] == "ADVISORY"   # over-flight above ceilings
    assert not any(r["inside"] for r in status["zones"])
    assert any("above" in r for r in status["reasons"])


def test_low_drone_over_core_is_critical_with_hysteresis():
    mon = make_monitor()
    res = feed(mon, [(0, 0, 40)] * 3, cam=True)
    assert res[0][0]["level"] < 4               # first sample: not yet confirmed
    final = res[-1][0]
    assert final["level_name"] == "CRITICAL" and final["camera_confirmed"]
    kinds = [e["kind"] for _, evs in res for e in evs]
    assert kinds.count("ZONE_ENTER") == 3       # buffer, campus, core
    assert all(e["camera_confirmed"] for _, evs in res for e in evs if e["kind"] == "ZONE_ENTER")


def test_breach_between_core_and_campus_edge():
    mon = make_monitor()
    final = feed(mon, [(90, 90, 80)] * 3)[-1][0]
    assert final["level_name"] == "BREACH"
    assert final["recommended_action"].startswith("Restricted airspace violated")


def test_approach_warning_with_time_to_breach():
    mon = make_monitor()
    # flying west->east at 10 m/s, 60 m altitude, outside the campus (edge at e=-125)
    path = [(-260 + 2 * i, 0, 60) for i in range(30)]       # 2 m per 0.2 s = 10 m/s
    res = feed(mon, path)
    levels = [s["level_name"] for s, _ in res]
    assert "WARNING" in levels
    warn = next(s for s, _ in res if s["level_name"] == "WARNING")
    assert warn["time_to_breach_s"] is not None and warn["time_to_breach_s"] <= 20


def test_high_approach_does_not_warn():
    mon = make_monitor()
    path = [(-260 + 2 * i, 0, 180) for i in range(30)]      # well above 120 m + margin
    levels = {s["level_name"] for s, _ in feed(mon, path)}
    assert "WARNING" not in levels


def test_exit_events_and_return_to_clear():
    mon = make_monitor()
    res = feed(mon, [(0, 0, 40)] * 3 + [(1000, 1000, 40)] * 3)
    final = res[-1][0]
    assert final["level_name"] == "CLEAR"
    assert sum(1 for _, evs in res for e in evs if e["kind"] == "ZONE_EXIT") == 3
