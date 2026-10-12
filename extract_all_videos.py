import cv2
from pathlib import Path

BASE = Path(__file__).resolve().parent
VIDEOS_DIR = BASE / "data" / "test_videos"
OUTPUT_DIR = BASE / "data" / "test_frames"

videos = [
    "Mavic_2_Test.mp4",
    "Mavic_3_Test.mp4",
    "Phantom_4_Test.mp4",
    "M300_Test.mp4",
]

for video_name in videos:
    video_path = VIDEOS_DIR / video_name
    drone_name = video_path.stem.replace("_Test", "")
    frame_dir = OUTPUT_DIR / drone_name
    frame_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    count = 0

    while True:
        success, frame = cap.read()

        if not success:
            break

        frame_path = frame_dir / f"frame_{count:04d}.jpg"

        if not cv2.imwrite(str(frame_path), frame):
            raise RuntimeError(f"Cannot save frame: {frame_path}")

        count += 1

    cap.release()

    print(f"{drone_name}: {count} frames extracted")

print("\nAll videos processed successfully!")