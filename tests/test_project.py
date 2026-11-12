from pathlib import Path
import ast
import json

ROOT = Path(__file__).resolve().parents[1]

PY_FILES = ["server.py", "make_detections.py", "make_test_frames.py", "geo.py", "geofence.py",
            "mavlink_lite.py", "qgc_plan.py", "sim_drone.py", "sensor_adapters.py",
            "tools/build_campus_demo.py", "tools/convert_sensor_output.py"]


def test_required_project_files_exist():
    required = PY_FILES + ["test_dashboard.html", "static/aeromap.js", "requirements.txt",
                           "requirements_detection.txt", "data/site_config.json",
                           "data/geofence/fast_campus_zones.json",
                           "data/missions/fast_intruder_demo.plan"]
    for filename in required:
        assert (ROOT / filename).is_file(), f"Missing project file: {filename}"


def test_python_files_have_valid_syntax():
    for filename in PY_FILES:
        source = (ROOT / filename).read_text(encoding="utf-8")
        ast.parse(source, filename=filename)


def test_backend_dependencies_are_declared():
    requirements = (ROOT / "requirements.txt").read_text(encoding="utf-8").lower()
    assert "fastapi" in requirements
    assert "uvicorn" in requirements


def test_dashboard_controls_and_panels_are_present():
    html = (ROOT / "test_dashboard.html").read_text(encoding="utf-8")
    for element_id in ["videoWrap", "frame", "status", "lidarValue", "mapLive", "mapPlan",
                       "zoneList", "threatLevel", "altChart", "radarBlips", "fullEventTableBody",
                       "zoneEditor", "wpBody"]:
        assert f'id="{element_id}"' in html, f"Dashboard element missing: {element_id}"
    assert "/static/aeromap.js" in html


def test_site_config_is_fast_islamabad():
    site = json.loads((ROOT / "data/site_config.json").read_text(encoding="utf-8"))
    lat, lon = site["map_center"]
    assert 33.6 < lat < 33.7 and 72.95 < lon < 73.05          # H-11, Islamabad
    assert {"lat", "lon", "height_agl"} <= set(site["sensor_site"])
