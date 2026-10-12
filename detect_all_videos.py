from pathlib import Path
import make_detections as detector

BASE = Path(__file__).resolve().parent
DATA = BASE / "data"

videos = ["Mavic_2", "Mavic_3", "Phantom_4", "M300"]

for name in videos:
    print(f"\n===== Processing {name} =====")

    detector.FRAMES_DIR = DATA / "test_frames" / name
    detector.OUTPUT_FILE = DATA / f"{name}_detections.json"

    detector.main()

print("\nAll four videos processed successfully!")