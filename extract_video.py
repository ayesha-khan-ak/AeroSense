import cv2
from pathlib import Path

video_path = Path("data/test_videos/Avata_Test.mp4")
output_dir = Path("data/test_frames/Avata")
output_dir.mkdir(parents=True, exist_ok=True)

cap = cv2.VideoCapture(str(video_path))

if not cap.isOpened():
    raise RuntimeError("Video could not be opened")

count = 0

while True:
    success, frame = cap.read()

    if not success:
        break

    output_path = output_dir / f"frame_{count:04d}.jpg"

    if not cv2.imwrite(str(output_path), frame):
        raise RuntimeError(f"Failed to save {output_path}")

    count += 1

cap.release()

print("Frames extracted:", count)
print("Saved to:", output_dir.resolve())