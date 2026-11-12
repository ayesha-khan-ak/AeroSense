#!/usr/bin/env python3
"""
Generate realistic radar and LiDAR detection data for AeroSense demo.

SYNCED with the camera feed:
  - Duration matches total camera frames / FPS (653 frames / 10 fps = 65.3s)
  - Drone type classification matches the camera drone type being shown
  - Detection confidence correlates with camera confidence

Creates:
  data/radar/radar_tracks.json
  data/lidar/lidar_tracks.json
  data/audio/audio_detections.json
  data/frontend_demo_feed.json  (MMAUD-format feed)
"""
import json
import math
import random
import os

# Sensor mast location (from site_config.json)
SITE_LAT = 33.6555484
SITE_LON = 73.0173803
SITE_ALT = 15.0
SENSOR_HEADING = 0.0

# Camera drone types (must match DRONE_TYPES order in server.py)
CAMERA_SEQUENCE = [
    {"name": "Avata",     "frames": 185, "mmaud_name": "Avata"},
    {"name": "M300",      "frames": 72,  "mmaud_name": "M300"},
    {"name": "Mavic_2",   "frames": 90,  "mmaud_name": "Mavic2"},
    {"name": "Mavic_3",   "frames": 142, "mmaud_name": "Mavic3"},
    {"name": "Phantom_4", "frames": 164, "mmaud_name": "Pham4"},
]
DRONE_NAMES = ["Avata", "M300", "Mavic2", "Mavic3", "Pham4"]

PLAYBACK_FPS = 10
TOTAL_FRAMES = sum(d["frames"] for d in CAMERA_SEQUENCE)  # 653
TOTAL_DURATION = TOTAL_FRAMES / PLAYBACK_FPS  # 65.3 seconds
SENSOR_HZ = 5  # MMAUD rate
NUM_WINDOWS = int(TOTAL_DURATION * SENSOR_HZ)  # 326 windows

# Flight path timed to match camera sequence:
#   Avata  (0-18.5s): approaching from outside, entering buffer
#   M300   (18.5-25.7s): entering restricted zone
#   Mavic_2 (25.7-34.7s): inside restricted, approaching critical
#   Mavic_3 (34.7-48.9s): inside critical zone, loitering
#   Phantom_4 (48.9-65.3s): exiting critical -> restricted -> buffer -> outside
FLIGHT_PATH = [
    # Avata phase: approaching (0-18.5s)
    (0.0,  33.6520, 73.0100, 80),
    (4.0,  33.6528, 73.0115, 85),
    (8.0,  33.6535, 73.0130, 90),
    (12.0, 33.6540, 73.0140, 90),
    (16.0, 33.6545, 73.0148, 85),
    (18.5, 33.6548, 73.0152, 80),
    # M300 phase: entering restricted (18.5-25.7s)
    (20.0, 33.6550, 73.0155, 75),
    (22.0, 33.6553, 73.0158, 70),
    (24.0, 33.6556, 73.0160, 65),
    (25.7, 33.6558, 73.0162, 60),
    # Mavic_2 phase: approaching critical (25.7-34.7s)
    (27.0, 33.6559, 73.0162, 55),
    (29.0, 33.6561, 73.0163, 50),
    (31.0, 33.6563, 73.0164, 45),
    (33.0, 33.6564, 73.0165, 40),
    (34.7, 33.6565, 73.0166, 35),
    # Mavic_3 phase: inside critical, loitering (34.7-48.9s)
    (36.0, 33.6566, 73.0166, 32),
    (38.0, 33.6567, 73.0167, 30),
    (40.0, 33.6566, 73.0168, 28),
    (42.0, 33.6565, 73.0167, 30),
    (44.0, 33.6564, 73.0166, 32),
    (46.0, 33.6565, 73.0165, 33),
    (48.0, 33.6566, 73.0166, 30),
    (48.9, 33.6567, 73.0167, 32),
    # Phantom_4 phase: exiting (48.9-65.3s)
    (50.0, 33.6568, 73.0168, 38),
    (52.0, 33.6570, 73.0170, 45),
    (54.0, 33.6572, 73.0172, 55),
    (56.0, 33.6575, 73.0175, 65),
    (58.0, 33.6578, 73.0180, 75),
    (60.0, 33.6582, 73.0185, 85),
    (62.0, 33.6588, 73.0192, 100),
    (65.3, 33.6595, 73.0200, 120),
]


def interp_at(path, t):
    """Interpolate position at time t."""
    if t <= path[0][0]:
        return path[0][1], path[0][2], path[0][3]
    if t >= path[-1][0]:
        return path[-1][1], path[-1][2], path[-1][3]
    for i in range(1, len(path)):
        if t <= path[i][0]:
            t0, lat0, lon0, alt0 = path[i-1]
            t1, lat1, lon1, alt1 = path[i]
            frac = (t - t0) / max(t1 - t0, 1e-6)
            return (lat0 + (lat1 - lat0) * frac,
                    lon0 + (lon1 - lon0) * frac,
                    alt0 + (alt1 - alt0) * frac)
    return path[-1][1], path[-1][2], path[-1][3]


def drone_type_at(t):
    """Which camera drone type is showing at time t."""
    elapsed = 0
    for d in CAMERA_SEQUENCE:
        dur = d["frames"] / PLAYBACK_FPS
        if t < elapsed + dur:
            return d["mmaud_name"]
        elapsed += dur
    return CAMERA_SEQUENCE[-1]["mmaud_name"]


def latlon_to_sensor_xyz(lat, lon, alt, site_lat, site_lon, site_alt, heading_deg):
    R = 6371008.8
    east = math.radians(lon - site_lon) * R * math.cos(math.radians(site_lat))
    north = math.radians(lat - site_lat) * R
    up = alt - site_alt
    h = math.radians(heading_deg)
    cos_h, sin_h = math.cos(h), math.sin(h)
    x = east * sin_h + north * cos_h
    y = -east * cos_h + north * sin_h
    return x, y, up


def add_noise(val, sigma):
    return val + random.gauss(0, sigma)


def make_class_probs(dominant_name):
    """Make classification probabilities with the dominant type having highest prob."""
    probs = {}
    dominant_idx = DRONE_NAMES.index(dominant_name) if dominant_name in DRONE_NAMES else 0
    for i, name in enumerate(DRONE_NAMES):
        if i == dominant_idx:
            probs[name] = random.uniform(0.50, 0.85)
        else:
            probs[name] = random.uniform(0.02, 0.12)
    total = sum(probs.values())
    return {k: round(v / total, 4) for k, v in probs.items()}


def generate_radar_tracks(dt=0.2):
    detections = []
    radar_range = 1000
    n = int(TOTAL_DURATION / dt) + 1
    for i in range(n):
        t = i * dt
        lat, lon, alt = interp_at(FLIGHT_PATH, t)
        x, y, z = latlon_to_sensor_xyz(lat, lon, alt, SITE_LAT, SITE_LON, SITE_ALT, SENSOR_HEADING)
        rng = math.sqrt(x*x + y*y + z*z)
        if rng > radar_range:
            continue
        det_prob = max(0.3, 1.0 - (rng / radar_range) * 0.6)
        if random.random() > det_prob:
            continue
        noise = 2.0 + rng * 0.003
        detections.append({
            "t": round(t, 2), "x": round(add_noise(x, noise), 2),
            "y": round(add_noise(y, noise), 2), "z": round(add_noise(z, noise * 0.5), 2),
            "confidence": round(min(0.99, det_prob + random.uniform(-0.05, 0.05)), 3),
            "track_id": 1, "label": "drone",
        })
    return {"sensor": "radar", "coordinate_frame": "sensor_xyz", "detections": detections}


def generate_lidar_tracks(dt=0.2):
    detections = []
    lidar_range = 300
    n = int(TOTAL_DURATION / dt) + 1
    for i in range(n):
        t = i * dt
        lat, lon, alt = interp_at(FLIGHT_PATH, t)
        x, y, z = latlon_to_sensor_xyz(lat, lon, alt, SITE_LAT, SITE_LON, SITE_ALT, SENSOR_HEADING)
        rng = math.sqrt(x*x + y*y + z*z)
        if rng > lidar_range:
            continue
        det_prob = max(0.5, 1.0 - (rng / lidar_range) * 0.4)
        if random.random() > det_prob:
            continue
        noise = 0.5 + rng * 0.001
        detections.append({
            "t": round(t, 2), "x": round(add_noise(x, noise), 2),
            "y": round(add_noise(y, noise), 2), "z": round(add_noise(z, noise * 0.3), 2),
            "confidence": round(min(0.99, det_prob + random.uniform(-0.03, 0.03)), 3),
            "track_id": 1, "label": "drone",
        })
    return {"sensor": "lidar", "coordinate_frame": "sensor_xyz", "detections": detections}


def generate_audio_detections(dt=0.2):
    detections = []
    audio_range = 500
    n = int(TOTAL_DURATION / dt) + 1
    for i in range(n):
        t = i * dt
        lat, lon, alt = interp_at(FLIGHT_PATH, t)
        x, y, z = latlon_to_sensor_xyz(lat, lon, alt, SITE_LAT, SITE_LON, SITE_ALT, SENSOR_HEADING)
        rng = math.sqrt(x*x + y*y + z*z)
        if rng > audio_range:
            continue
        det_prob = max(0.2, 1.0 - (rng / audio_range) * 0.7)
        if random.random() > det_prob:
            continue
        dtype = drone_type_at(t)
        label_map = {"Avata": "Avata", "M300": "DJI M300", "Mavic2": "Mavic",
                     "Mavic3": "Mavic", "Pham4": "Phantom"}
        detections.append({
            "t": round(t, 2), "drone": True,
            "confidence": round(min(0.95, det_prob + random.uniform(-0.1, 0.1)), 3),
            "label": label_map.get(dtype, "DJI"),
        })
    return {"sensor": "audio", "coordinate_frame": "none", "detections": detections}


def generate_mmaud_feed():
    """MMAUD feed synced with camera drone type."""
    windows = []
    radar_range, lidar_range = 1000, 300
    for i in range(NUM_WINDOWS):
        t = i / SENSOR_HZ
        lat, lon, alt = interp_at(FLIGHT_PATH, t)
        x, y, z = latlon_to_sensor_xyz(lat, lon, alt, SITE_LAT, SITE_LON, SITE_ALT, SENSOR_HEADING)
        rng = math.sqrt(x*x + y*y + z*z)
        dtype = drone_type_at(t)

        radar_det = rng <= radar_range and random.random() < max(0.4, 1.0 - rng/radar_range*0.5)
        radar_noise = 2.0 + rng * 0.003
        radar_pos = [round(add_noise(x, radar_noise), 2), round(add_noise(y, radar_noise), 2),
                     round(add_noise(z, radar_noise*0.5), 2)] if radar_det else None
        radar_probs = make_class_probs(dtype) if radar_det else None
        radar_probs_list = [radar_probs.get(n, 0) for n in DRONE_NAMES] if radar_probs else None

        avia_det = rng <= lidar_range*0.8 and random.random() < max(0.5, 1.0 - rng/(lidar_range*0.8)*0.3)
        avia_pos = [round(add_noise(x, 0.4), 2), round(add_noise(y, 0.4), 2),
                    round(add_noise(z, 0.12), 2)] if avia_det else None

        mid_det = rng <= lidar_range and random.random() < max(0.45, 1.0 - rng/lidar_range*0.4)
        mid_pos = [round(add_noise(x, 0.5), 2), round(add_noise(y, 0.5), 2),
                   round(add_noise(z, 0.15), 2)] if mid_det else None

        fusion_det = radar_det or avia_det or mid_det
        fusion_pos = (avia_pos[:] if avia_det and avia_pos else
                      mid_pos[:] if mid_det and mid_pos else
                      radar_pos[:] if radar_det and radar_pos else None)
        fusion_probs = radar_probs_list if radar_probs_list else (
            [round(1.0/len(DRONE_NAMES), 4)]*len(DRONE_NAMES) if fusion_det else None)

        windows.append({
            "radar": {"detected": radar_det, "position": radar_pos, "class_probs": radar_probs_list},
            "avia": {"detected": avia_det, "position": avia_pos, "class_probs": None},
            "mid360": {"detected": mid_det, "position": mid_pos, "class_probs": None},
            "placeholder_fusion": {"detected": fusion_det, "position": fusion_pos, "class_probs": fusion_probs},
        })
    return {"windows": windows}


def main():
    random.seed(42)
    here = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(os.path.dirname(here), "data")

    print(f"Camera: {TOTAL_FRAMES} frames at {PLAYBACK_FPS} fps = {TOTAL_DURATION:.1f}s cycle")
    print(f"Sensor: {NUM_WINDOWS} MMAUD windows at {SENSOR_HZ} Hz = {NUM_WINDOWS/SENSOR_HZ:.1f}s")
    elapsed = 0
    for d in CAMERA_SEQUENCE:
        dur = d["frames"] / PLAYBACK_FPS
        print(f"  {elapsed:5.1f}-{elapsed+dur:5.1f}s: {d['name']:10s} ({d['frames']} frames) -> radar classifies as {d['mmaud_name']}")
        elapsed += dur

    for name, gen, subdir in [("radar", generate_radar_tracks, "radar"),
                              ("lidar", generate_lidar_tracks, "lidar"),
                              ("audio", generate_audio_detections, "audio")]:
        data = gen()
        p = os.path.join(data_dir, subdir, f"{subdir}_tracks.json" if subdir != "audio" else "audio_detections.json")
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f: json.dump(data, f, indent=2)
        count = len(data.get("detections", []))
        print(f"[{name}] {count} detections -> {p}")

    mmaud = generate_mmaud_feed()
    p = os.path.join(data_dir, "frontend_demo_feed.json")
    with open(p, "w") as f: json.dump(mmaud, f, indent=2)
    print(f"[mmaud] {len(mmaud['windows'])} windows -> {p}")
    print(f"\nDone! All sensor data synced to {TOTAL_DURATION:.1f}s camera cycle.")


if __name__ == "__main__":
    main()