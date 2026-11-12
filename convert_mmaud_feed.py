#!/usr/bin/env python3
"""
convert_mmaud_feed.py  –  Convert frontend_demo_feed.json → contract-format JSON files
======================================================================================

This script reads the MMAUD frontend_demo_feed.json (produced by your teammate)
and writes two contract-format files that server.py can also load as legacy fallback:

    data/radar/radar_tracks.json
    data/lidar/lidar_tracks.json

Contract format (same as sensor_adapters.py expects):
    {
      "sensor": "radar",
      "coordinate_frame": "sensor_xyz",
      "detections": [
        {"t": 0.00, "x": 41.2, "y": -3.5, "z": 18.0, "confidence": 0.93},
        ...
      ]
    }

Usage:
    python3 tools/convert_mmaud_feed.py                       # uses default path
    python3 tools/convert_mmaud_feed.py path/to/feed.json     # custom path

The MMAUD feed format (per window, 5 Hz = 0.2 s apart):
    [
      {
        "radar":    {"detected": true/false, "position": {"x","y","z"} or null, "class_probs": {...} or null},
        "avia":     {"detected": ..., "position": ..., "class_probs": ...},
        "mid360":   {"detected": ..., "position": ..., "class_probs": ...},
        "placeholder_fusion": {"detected": ..., "position": ..., "class_probs": ...}
      },
      ...
    ]

Positions are in metres, sensor_xyz frame (x=forward, y=left, z=up from sensor mast).
"""

import json, sys, pathlib

HERE = pathlib.Path(__file__).resolve().parent.parent          # project root
DEFAULT_FEED = HERE / "data" / "frontend_demo_feed.json"
DT = 0.2  # 5 Hz → 200 ms between windows


def convert(feed_path: pathlib.Path):
    print(f"Reading MMAUD feed from: {feed_path}")
    with open(feed_path) as f:
        windows = json.load(f)

    if not isinstance(windows, list):
        print("ERROR: expected a JSON array of windows"); sys.exit(1)

    print(f"  {len(windows)} windows ({len(windows) * DT:.1f} s at 5 Hz)")

    # ------------------------------------------------------------------ radar
    radar_dets = []
    for i, w in enumerate(windows):
        r = w.get("radar")
        if not r or not r.get("detected"):
            continue
        pos = r.get("position") or {}
        x, y, z = pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)
        # Pick best class prob as confidence
        probs = r.get("class_probs") or {}
        conf = max(probs.values()) if probs else 0.5
        radar_dets.append({
            "t": round(i * DT, 2),
            "x": x, "y": y, "z": z,
            "confidence": round(conf, 3),
        })

    radar_out = {
        "sensor": "radar",
        "coordinate_frame": "sensor_xyz",
        "detections": radar_dets,
    }

    radar_dir = HERE / "data" / "radar"
    radar_dir.mkdir(parents=True, exist_ok=True)
    radar_file = radar_dir / "radar_tracks.json"
    with open(radar_file, "w") as f:
        json.dump(radar_out, f, indent=2)
    print(f"  Wrote {len(radar_dets)} radar detections → {radar_file}")

    # ------------------------------------------------------------------ lidar
    # Merge avia + mid360 into one lidar track file
    lidar_dets = []
    for i, w in enumerate(windows):
        # prefer mid360 for position (360° coverage), fall back to avia
        mid = w.get("mid360") or {}
        avi = w.get("avia") or {}
        src = None
        if mid.get("detected") and mid.get("position"):
            src = mid
        elif avi.get("detected") and avi.get("position"):
            src = avi
        if not src:
            continue
        pos = src["position"]
        x, y, z = pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)
        probs = src.get("class_probs") or {}
        conf = max(probs.values()) if probs else 0.5
        lidar_dets.append({
            "t": round(i * DT, 2),
            "x": x, "y": y, "z": z,
            "confidence": round(conf, 3),
        })

    lidar_out = {
        "sensor": "lidar",
        "coordinate_frame": "sensor_xyz",
        "detections": lidar_dets,
    }

    lidar_dir = HERE / "data" / "lidar"
    lidar_dir.mkdir(parents=True, exist_ok=True)
    lidar_file = lidar_dir / "lidar_tracks.json"
    with open(lidar_file, "w") as f:
        json.dump(lidar_out, f, indent=2)
    print(f"  Wrote {len(lidar_dets)} lidar detections → {lidar_file}")

    # -------------------------------------------------------------- summary
    print()
    print("Done!  Files written:")
    print(f"  {radar_file}")
    print(f"  {lidar_file}")
    print()
    print("These are also loaded automatically by server.py's MMAUDFeedReplay class.")
    print("The legacy SensorReplay in sensor_adapters.py can use them as fallback.")


if __name__ == "__main__":
    feed = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_FEED
    if not feed.exists():
        print(f"ERROR: {feed} not found.")
        print()
        print("Download frontend_demo_feed.json from your teammate's Google Drive folder")
        print("and place it at:  data/frontend_demo_feed.json")
        print()
        print("Then run:  python3 tools/convert_mmaud_feed.py")
        sys.exit(1)
    convert(feed)
