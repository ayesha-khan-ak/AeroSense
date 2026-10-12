# AeroSense Camera Server (Your Part)

This is the **server** that holds the camera frames and detection results, and
streams them to the dashboard running on your teammate's laptop. It makes the
recorded MMAUD camera data look like a **live feed**.

---

## What this does (in plain words)

- You put camera images in `data/frames/`  (e.g. frame_0001.jpg, frame_0002.jpg ...)
- You put detection results in `data/detections.json`  (which frames have a drone)
- You run the server on YOUR laptop.
- Your teammate's dashboard connects over WiFi and plays the frames one by one,
  like a live video. Each frame comes with its detection result, so the dashboard
  can turn the camera panel RED + beep + show an alert when a drone is detected.

---

## HOW TO RUN IT (step by step)

### 1. Install the requirements (one time)
Open a terminal in this folder and run:
```
pip install -r requirements.txt
```

### 2. (Optional) Make test frames to try it before you have real data
```
python make_test_frames.py
```
This puts 5 small test images in `data/frames/` so you can see it work right away.
Later, delete these and put your real MMAUD camera images there instead.

### 3. Start the server
```
python server.py
```
You should see it start on port 8000. Leave this terminal running.

### 4. Check it works
Open a browser on the SAME laptop and go to:
```
http://127.0.0.1:8000/
```
You should see a message saying the server is running and how many frames it found.
Also try  http://127.0.0.1:8000/info  to see the detection summary.

---

## CONNECTING FROM YOUR TEAMMATE'S LAPTOP (important)

Both laptops must be on the **same WiFi**.

1. Find YOUR laptop's IP address:
   - Windows: open Command Prompt, type `ipconfig`, look for "IPv4 Address"
     (something like 192.168.1.37)
   - Mac/Linux: type `ifconfig` or `ip addr`, find the 192.168.x.x address
2. Your teammate's dashboard connects to:
   - WebSocket (main):  `ws://YOUR_IP:8000/ws`
   - or REST frame:     `http://YOUR_IP:8000/frame/0`
   Example: `ws://192.168.1.37:8000/ws`
3. If the teammate's laptop can't connect, it's almost always the **firewall** on
   your laptop. Allow Python through the firewall, or temporarily turn the firewall
   off for the demo (on a trusted network).

---

## WHAT YOUR TEAMMATES NEED TO GIVE YOU

- **Vision teammate:** the camera images (put in `data/frames/`) AND the detection
  results file `data/detections.json`. The format is:
  ```json
  {
    "frame_0001.jpg": {"drone": false, "confidence": 0.0,  "box": null},
    "frame_0003.jpg": {"drone": true,  "confidence": 0.88, "box": [120, 80, 60, 40]}
  }
  ```
  - `drone`: true/false — is a drone in this frame?
  - `confidence`: 0.0 to 1.0 — how sure the model is
  - `box`: [x, y, width, height] of the drone in the image, or null if no drone
- **Frontend teammate:** connects to `ws://YOUR_IP:8000/ws` and, for each message,
  shows the `image` on the left and uses the `drone` flag to turn the camera panel
  RED + beep + alert.

---

## THE ENDPOINTS (what the server offers)

| Address | What it gives |
|---|---|
| `/`              | Server status + how many frames found |
| `/health`        | Just says "ok" (to check it's alive) |
| `/info`          | Total frames, how many have drones |
| `/frame/{n}`     | The image for frame number n (backup method) |
| `/detection/{n}` | The detection result for frame number n (backup method) |
| `/ws`            | **WebSocket** — streams every frame + result in order (main method) |

---

## SETTINGS YOU CAN CHANGE (top of server.py)

- `PLAYBACK_FPS = 5`  → how many frames per second the "live feed" plays.
  Lower = slower/calmer (good for a demo). Raise it for faster playback.

---

## IF SOMETHING GOES WRONG

- **"No frames found"** → your images aren't in `data/frames/`, or aren't .jpg/.png.
- **Teammate can't connect** → firewall (see above), or wrong IP, or not same WiFi.
- **WebSocket error on start** → run `pip install "uvicorn[standard]"` (it needs the
  websocket library; it's already in requirements.txt).
