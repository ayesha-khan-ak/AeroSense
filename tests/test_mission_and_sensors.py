import json
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import geo  # noqa: E402
from geofence import GeofenceMonitor, TrackSample, load_zones  # noqa: E402
from mavlink_lite import MavlinkParser  # noqa: E402
from qgc_plan import build_plan, parse_plan  # noqa: E402
from sensor_adapters import SensorReplay  # noqa: E402
from sim_drone import SimDrone  # noqa: E402

PLAN = ROOT / "data/missions/fast_intruder_demo.plan"
ZONES = ROOT / "data/geofence/fast_campus_zones.json"


def test_repo_plan_is_valid_qgc_format():
    plan = json.loads(PLAN.read_text())
    assert plan["fileType"] == "Plan" and plan["groundStation"] == "QGroundControl"
    assert plan["geoFence"]["version"] == 2 and plan["rallyPoints"]["points"] == []
    items = plan["mission"]["items"]
    assert items[0]["command"] == 22 and all(i["type"] == "SimpleItem" for i in items)
    assert all(len(i["params"]) == 7 for i in items)
    assert all(p["inclusion"] is False for p in plan["geoFence"]["polygons"])


def test_plan_roundtrip():
    wps = [{"lat": 33.65, "lon": 73.01, "alt": 30}, {"lat": 33.66, "lon": 73.02, "alt": 80}]
    parsed = parse_plan(build_plan(wps, [33.65, 73.01, 510], [], cruise_speed=9))
    assert parsed["cruise_speed"] == 9 and parsed["home"] == [33.65, 73.01, 510]
    assert [w["command"] for w in parsed["waypoints"]] == [22, 16, 16]
    assert parsed["waypoints"][-1]["alt"] == 80


def test_parse_rejects_non_plan():
    with pytest.raises(ValueError):
        parse_plan({"fileType": "Mission"})


def test_full_intruder_scenario_hits_every_threat_level():
    """Fly the demo mission (fast-forward) through the MAVLink codec into the geofence."""
    drone = SimDrone(PLAN, targets=["127.0.0.1:9"])
    mon = GeofenceMonitor(load_zones(ZONES))
    parser = MavlinkParser()
    seen, t = set(), 0.0
    for i in range(3500):                               # 350 s of flight at 10 Hz
        drone._step(0.1)
        t += 0.1
        if i % 2:
            continue
        pkt = drone.enc.encode("GLOBAL_POSITION_INT", lat=int(drone.lat * 1e7), lon=int(drone.lon * 1e7),
                               relative_alt=int(drone.alt * 1000), vz=int(-drone.vu * 100))
        (m,) = parser.feed(pkt)
        status, _ = mon.update(TrackSample(t, m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000,
                                           climb=-m.vz / 100))
        seen.add(status["level_name"])
    assert seen == {"CLEAR", "ADVISORY", "WARNING", "BREACH", "CRITICAL"}


def test_sim_drone_sends_real_udp_mavlink():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(3)
    port = rx.getsockname()[1]
    drone = SimDrone(PLAN, targets=[f"127.0.0.1:{port}"])
    th = threading.Thread(target=drone.run, daemon=True)
    th.start()
    names, p = set(), MavlinkParser()
    deadline = time.time() + 3
    try:
        while time.time() < deadline and not {"HEARTBEAT", "GLOBAL_POSITION_INT"} <= names:
            names |= {m.name for m in p.feed(rx.recv(2048))}
    finally:
        drone.stop()
        rx.close()
    assert {"HEARTBEAT", "GLOBAL_POSITION_INT"} <= names


SITE = {"lat": 33.6555, "lon": 73.0174, "height_agl": 15.0}


def write(tmp_path, data, name="r.json"):
    f = tmp_path / name
    f.write_text(json.dumps(data))
    return f


def test_sensor_replay_sensor_xyz(tmp_path):
    f = write(tmp_path, {"sensor": "lidar", "coordinate_frame": "sensor_xyz", "detections": [
        {"t": 0.0, "x": 100, "y": 0, "z": 50, "confidence": 0.9},
        {"t": 1.0, "x": 0, "y": 100, "z": 50, "confidence": 0.8}]})
    r = SensorReplay("lidar", f, SITE, heading_deg=0)          # x-axis points north
    assert r.available and r.status()["count"] == 2
    d0 = r.detections[0]
    assert d0["azimuth"] == pytest.approx(0, abs=1e-6) and d0["alt"] == pytest.approx(65)
    assert geo.haversine_m(SITE["lat"], SITE["lon"], d0["lat"], d0["lon"]) == pytest.approx(100, abs=0.5)
    assert r.detections[1]["azimuth"] == pytest.approx(270, abs=1e-6)   # y = left = west
    assert [d["t"] for d in r.at(1.0)] == [1.0]
    assert [d["t"] for d in r.at(2.0 + 0.1)] == [0.0]           # loops (duration = 2 s)


def test_sensor_replay_polar_and_audio(tmp_path):
    f = write(tmp_path, {"coordinate_frame": "polar",
                         "detections": [{"t": 0, "range": 200, "azimuth": 90, "elevation": 0}]})
    d = SensorReplay("radar", f, SITE).detections[0]
    assert d["azimuth"] == pytest.approx(90) and d["range"] == pytest.approx(200)
    fa = write(tmp_path, {"sensor": "audio", "coordinate_frame": "none",
                          "detections": [{"t": 0, "drone": True, "confidence": 0.7, "label": "Mavic"}]}, "a.json")
    a = SensorReplay("audio", fa, SITE)
    assert a.detections[0]["drone"] and "lat" not in a.detections[0]


def test_missing_or_broken_sensor_file_never_crashes(tmp_path):
    assert not SensorReplay("radar", tmp_path / "nope.json", SITE).available
    bad = tmp_path / "bad.json"
    bad.write_text("{not json")
    r = SensorReplay("radar", bad, SITE)
    assert not r.available and "could not load" in r.status()["message"]
