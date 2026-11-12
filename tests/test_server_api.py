"""
Server API tests. Need fastapi + httpx (CI installs both):
    pip install -r requirements.txt httpx pytest
"""
import shutil
import sys
from pathlib import Path

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi.testclient import TestClient  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


@pytest.fixture()
def client(tmp_path, monkeypatch):
    import server
    # work on copies so tests never modify the committed zones/mission
    for name in ("ZONES_FILE", "MISSION_FILE"):
        src = getattr(server, name)
        dst = tmp_path / src.name
        shutil.copy(src, dst)
        monkeypatch.setattr(server, name, dst)
    monkeypatch.setattr(server, "DB_FILE", tmp_path / "events.db")
    monkeypatch.setattr(server, "MAVLINK_PORT", 0)              # any free UDP port
    monkeypatch.setattr(server, "RUN_SIM", False)
    with TestClient(server.app) as c:
        yield c


def test_basic_endpoints(client):
    assert client.get("/health").json() == {"status": "ok"}
    assert client.get("/dashboard").status_code == 200
    assert client.get("/static/aeromap.js").status_code == 200
    assert client.get("/api/site").json()["site_name"].startswith("FAST")


def test_zones_validation_and_update(client):
    zones = client.get("/api/zones").json()["zones"]
    assert len(zones) == 3
    bad = client.put("/api/zones", json={"zones": [{"name": "x", "kind": "restricted", "polygon": [[1, 2]]}]})
    assert bad.status_code == 400
    zones[1]["max_alt"] = 150
    r = client.put("/api/zones", json={"zones": zones})
    assert r.status_code == 200 and r.json()["zones"][1]["max_alt"] == 150


def test_mission_and_plan_export(client):
    assert client.put("/api/mission", json={"waypoints": []}).status_code == 400
    wps = [{"lat": 33.65, "lon": 73.01, "alt": 40}, {"lat": 33.657, "lon": 73.017, "alt": 60}]
    assert client.put("/api/mission", json={"waypoints": wps, "cruise_speed": 10}).status_code == 200
    plan = client.get("/api/export/plan").json()
    assert plan["fileType"] == "Plan" and len(plan["geoFence"]["polygons"]) == 2
    assert client.put("/api/mission", json=plan).status_code == 200      # re-import a .plan


def test_events_and_operator_action(client):
    assert client.post("/api/operator-action", json={"action": "unit test"}).json()["ok"]
    kinds = [e["kind"] for e in client.get("/api/events").json()["events"]]
    assert "OPERATOR_ACTION" in kinds


def test_websocket_hello(client):
    with client.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello" and len(hello["zones"]) == 3 and hello["mission"]["waypoints"]
