from pathlib import Path
import ast

ROOT = Path(__file__).resolve().parents[1]


def test_required_project_files_exist():
    required = [
        "server.py",
        "make_detections.py",
        "make_test_frames.py",
        "test_dashboard.html",
        "requirements.txt",
        "requirements_detection.txt",
    ]
    for filename in required:
        assert (ROOT / filename).is_file(), f"Missing project file: {filename}"


def test_python_files_have_valid_syntax():
    python_files = [
        "server.py",
        "make_detections.py",
        "make_test_frames.py",
    ]
    for filename in python_files:
        source = (ROOT / filename).read_text(encoding="utf-8")
        ast.parse(source, filename=filename)


def test_backend_dependencies_are_declared():
    requirements = (ROOT / "requirements.txt").read_text(
        encoding="utf-8"
    ).lower()
    assert "fastapi" in requirements
    assert "uvicorn" in requirements


def test_dashboard_controls_and_camera_are_present():
    html = (ROOT / "test_dashboard.html").read_text(
        encoding="utf-8"
    )
    for element_id in [
        "startBtn",
        "stopBtn",
        "resumeBtn",
        "videoWrap",
        "frame",
    ]:
        assert f'id="{element_id}"' in html, (
            f"Dashboard element missing: {element_id}"
        )
