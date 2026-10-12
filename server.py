# ============================================================
#  AeroSense Camera Server  (FastAPI)
#  Your part: the SERVER that holds the camera frames + detection
#  results, and streams them to the dashboard (on another laptop).
#
#  HOW IT WORKS (simple):
#   - You put your camera images in   data/frames/   (frame_0001.jpg ...)
#   - You put detection results in     data/detections.json
#   - You run this server on YOUR laptop.
#   - Your teammate's dashboard connects over WiFi and plays the frames
#     one by one (like a live video), with each frame's detection result.
#
#  WHY THESE CHOICES:
#   - "Pre-computed" detections (read from a file) = safe, nothing can
#     crash during the demo. Your vision teammate produces detections.json.
#   - WebSocket streaming = the server pushes frames on a timer, so it
#     looks like a live feed. A REST endpoint is ALSO provided as backup.
# ============================================================

import os
import json
import base64
import asyncio
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware

# ---------- Settings you can change ----------
FRAMES_DIR = Path(__file__).parent / "data" / "frames"     # where your images are
DETECTIONS_FILE = Path(__file__).parent / "data" / "detections.json"
PLAYBACK_FPS = 1          # frames per second. 1 = each frame shows for a FULL second (easy to see each detection). Raise later for faster playback.
# ---------------------------------------------

app = FastAPI(title="AeroSense Camera Server")

# CORS = let the dashboard on ANOTHER laptop connect to this server.
# "*" means allow everyone — fine for a local demo on your own WiFi.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def load_frames():
    """Get a sorted list of image files in the frames folder."""
    if not FRAMES_DIR.exists():
        return []
    exts = (".jpg", ".jpeg", ".png")
    files = [f for f in sorted(os.listdir(FRAMES_DIR)) if f.lower().endswith(exts)]
    return files


def load_detections():
    """
    Load detection results.
    Expected format in detections.json:
      {
        "frame_0001.jpg": {"drone": false, "confidence": 0.0, "box": null},
        "frame_0002.jpg": {"drone": true,  "confidence": 0.91,
                            "box": [x, y, width, height]},
        ...
      }
    If the file is missing, every frame is treated as "no drone".
    """
    if DETECTIONS_FILE.exists():
        try:
            with open(DETECTIONS_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            print("Could not read detections.json:", e)
            return {}
    return {}


# ---------- Basic info endpoints (open these in a browser to check) ----------

@app.get("/")
def home():
    frames = load_frames()
    return {
        "server": "AeroSense Camera Server is running",
        "frames_found": len(frames),
        "detections_file_present": DETECTIONS_FILE.exists(),
        "playback_fps": PLAYBACK_FPS,
        "try": ["/health", "/info", "/frame/0  (first frame image)",
                "/ws  (websocket stream for the dashboard)"],
    }


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/info")
def info():
    frames = load_frames()
    dets = load_detections()
    drone_frames = sum(1 for v in dets.values() if v.get("drone"))
    return {
        "total_frames": len(frames),
        "frames_with_detection_data": len(dets),
        "frames_with_drone": drone_frames,
        "playback_fps": PLAYBACK_FPS,
    }


# ---------- REST: get one frame image by its number (backup method) ----------

@app.get("/frame/{index}")
def get_frame(index: int):
    frames = load_frames()
    if index < 0 or index >= len(frames):
        return JSONResponse({"error": "frame index out of range"}, status_code=404)
    return FileResponse(FRAMES_DIR / frames[index])


# ---------- REST: get the detection result for one frame (backup method) ----------

@app.get("/detection/{index}")
def get_detection(index: int):
    frames = load_frames()
    dets = load_detections()
    if index < 0 or index >= len(frames):
        return JSONResponse({"error": "frame index out of range"}, status_code=404)
    name = frames[index]
    result = dets.get(name, {"drone": False, "confidence": 0.0, "box": None})
    return {"index": index, "frame": name, **result}


# ---------- WebSocket: the MAIN way the dashboard gets the "live" feed ----------
# The server pushes, one by one, each frame (as an image) + its detection
# result, on a timer. The dashboard just displays what it receives.

@app.websocket("/ws")
async def stream(websocket: WebSocket):
    await websocket.accept()
    frames = load_frames()
    dets = load_detections()
    delay = 1.0 / max(PLAYBACK_FPS, 1)

    if not frames:
        await websocket.send_json({"type": "error",
                                   "message": "No frames found in data/frames/"})
        await websocket.close()
        return

    try:
        for i, name in enumerate(frames):
            # read the image and encode it so it can travel over the websocket
            with open(FRAMES_DIR / name, "rb") as f:
                img_b64 = base64.b64encode(f.read()).decode("utf-8")

            det = dets.get(name, {"drone": False, "confidence": 0.0, "box": None})

            await websocket.send_json({
                "type": "frame",
                "index": i,
                "total": len(frames),
                "frame_name": name,
                "image": "data:image/jpeg;base64," + img_b64,  # ready to show in <img>
                "drone": det.get("drone", False),
                "confidence": det.get("confidence", 0.0),
                "box": det.get("box", None),
            })
            await asyncio.sleep(delay)

        # tell the dashboard the replay finished
        await websocket.send_json({"type": "end"})
    except WebSocketDisconnect:
        print("Dashboard disconnected.")
    except Exception as e:
        print("Streaming error:", e)


if __name__ == "__main__":
    import uvicorn
    # host="0.0.0.0" is IMPORTANT: it lets OTHER laptops on your WiFi connect.
    uvicorn.run(app, host="0.0.0.0", port=8000)
