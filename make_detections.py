# ============================================================
#  AeroSense — make_detections.py
#  Runs your teammate's trained model over the camera frames and
#  writes detections.json (the "contract" the server reads).
#
#  This is what makes the server REAL instead of dummy:
#  instead of hand-typed detections, the MODEL decides which
#  frames have a drone and where the box goes.
#
#  Model config you were given:
#     imgsz = 640, class 0 = drone, conf = 0.25
#
#  HOW TO USE (on the laptop that has the .pt file):
#     1. pip install ultralytics
#     2. put the model file here as:  data/detection.pt
#        (unzip detection.pt.zip first — you need the .pt, not the .zip)
#     3. put your camera images in:   data/frames/
#     4. run:  python make_detections.py
#     5. it writes:  data/detections.json
#     Then just start server.py as normal — now it's real.
# ============================================================

import os
import json
import shutil
import zipfile
from pathlib import Path

# ---------- Settings (match the model config you were given) ----------
HERE          = Path(__file__).parent
DATA_DIR      = HERE / "data"
FRAMES_DIR    = DATA_DIR / "frames"
OUTPUT_FILE   = DATA_DIR / "detections.json"
IMG_SIZE      = 640          # imgsz the model was trained with
CONF_THRESHOLD = 0.25        # initial confidence; raise to 0.35-0.5 if you get false alarms
DRONE_CLASS_ID = 0           # class 0 = drone
# ----------------------------------------------------------------------


def resolve_model_path():
    """
    Find the model file no matter how it arrived.

    A PyTorch .pt file is INTERNALLY a zip, so it often shows up named
    'detection.pt.zip' or inside a wrapper zip. This handles every case
    so you DON'T have to unzip anything by hand:

      1. If a real .pt file is already in data/  -> use it.
      2. If a .zip is in data/:
           - if it contains a .pt inside  -> extract that .pt and use it.
           - otherwise the .zip IS the model (just misnamed) -> copy it to
             data/detection.pt and use that (torch reads it by content).
    """
    # 1) a real .pt already sitting in data/
    for name in ("detection.pt", "best.pt"):
        p = DATA_DIR / name
        if p.exists() and p.is_file():
            return p
    pts = sorted(DATA_DIR.glob("*.pt"))
    if pts:
        return pts[0]

    # 2) look for any .zip in data/ and sort it out automatically
    zips = sorted(DATA_DIR.glob("*.zip"))
    for z in zips:
        try:
            with zipfile.ZipFile(z) as zf:
                members = zf.namelist()
                # 2a) a .pt packed inside the zip -> extract just that file
                inner_pt = next((m for m in members if m.lower().endswith(".pt")), None)
                if inner_pt:
                    target = DATA_DIR / "detection.pt"
                    with zf.open(inner_pt) as src, open(target, "wb") as dst:
                        shutil.copyfileobj(src, dst)
                    print(f"Extracted model from {z.name} -> {target.name}")
                    return target
                # 2b) the zip itself is a torch checkpoint (has data.pkl inside)
                if any(m.endswith("data.pkl") for m in members):
                    target = DATA_DIR / "detection.pt"
                    shutil.copyfile(z, target)
                    print(f"{z.name} is the model itself -> copied to {target.name}")
                    return target
        except zipfile.BadZipFile:
            # not actually a zip -> maybe it's really a .pt with a wrong name
            target = DATA_DIR / "detection.pt"
            shutil.copyfile(z, target)
            print(f"{z.name} is not a zip; treating it as the model -> {target.name}")
            return target

    return None


def load_frame_names():
    """Exact image filenames in data/frames/, sorted — same order the server uses."""
    if not FRAMES_DIR.exists():
        raise SystemExit(f"Frames folder not found: {FRAMES_DIR}")
    exts = (".jpg", ".jpeg", ".png")
    names = [f for f in sorted(os.listdir(FRAMES_DIR)) if f.lower().endswith(exts)]
    if not names:
        raise SystemExit(f"No images (.jpg/.jpeg/.png) found in {FRAMES_DIR}")
    return names


def main():
    model_path = resolve_model_path()
    if model_path is None:
        raise SystemExit(
            "No model found in data/.\n"
            "Put the model file there as ANY of these and re-run:\n"
            "  detection.pt   OR   best.pt   OR   detection.pt.zip\n"
            "You do NOT need to unzip it — the script handles that."
        )

    # ultralytics covers YOLOv8/v9/v11 .pt files (the usual case for a '.pt' handed over).
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics is needed. Run:  pip install ultralytics")

    print(f"Loading model: {model_path}")
    model = YOLO(str(model_path))

    frame_names = load_frame_names()
    print(f"Found {len(frame_names)} frames. Running detection (imgsz={IMG_SIZE}, conf={CONF_THRESHOLD})...")

    detections = {}
    drone_count = 0

    for name in frame_names:
        img_path = FRAMES_DIR / name

        # run the model on this one image
        results = model.predict(
            source=str(img_path),
            imgsz=IMG_SIZE,
            conf=CONF_THRESHOLD,
            classes=[DRONE_CLASS_ID],   # only keep the 'drone' class
            verbose=False,
        )

        best = None          # the single most-confident drone box for this frame
        best_conf = 0.0

        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                cls = int(b.cls[0])
                if cls != DRONE_CLASS_ID:
                    continue
                conf = float(b.conf[0])
                if conf < best_conf:
                    continue
                # ultralytics gives xyxy (corner coords). The server/UI wants
                # [x, y, width, height] with x,y = top-left corner, in ORIGINAL pixels.
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                best = [round(x1), round(y1), round(x2 - x1), round(y2 - y1)]
                best_conf = conf

        if best is not None:
            detections[name] = {
                "drone": True,
                "confidence": round(best_conf, 3),
                "box": best,                 # [x, y, width, height]
            }
            drone_count += 1
        else:
            detections[name] = {
                "drone": False,
                "confidence": 0.0,
                "box": None,
            }

    with open(OUTPUT_FILE, "w") as f:
        json.dump(detections, f, indent=2)

    print(f"\nDone. {drone_count} of {len(frame_names)} frames have a drone.")
    print(f"Wrote: {OUTPUT_FILE}")
    print("Now run:  python server.py")


if __name__ == "__main__":
    main()
