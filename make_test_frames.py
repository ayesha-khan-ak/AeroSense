# Makes 5 simple test images so you can run the server BEFORE you have
# the real MMAUD frames. Run once:  python make_test_frames.py
# Later, delete these and drop your real MMAUD images into data/frames/.

from pathlib import Path

try:
    from PIL import Image, ImageDraw
except ImportError:
    raise SystemExit("Pillow is needed. Run:  pip install pillow")

OUT = Path(__file__).parent / "data" / "frames"
OUT.mkdir(parents=True, exist_ok=True)

for i in range(1, 6):
    img = Image.new("RGB", (320, 240), (30, 40, 60))
    d = ImageDraw.Draw(img)
    d.text((10, 10), f"TEST FRAME {i}", fill=(255, 255, 255))
    # draw a little "drone" on frames 3 and 4 to match the sample detections
    if i in (3, 4):
        d.ellipse((120, 80, 180, 120), outline=(255, 80, 80), width=3)
        d.text((120, 125), "drone", fill=(255, 120, 120))
    img.save(OUT / f"frame_{i:04d}.jpg")

print("Created 5 test frames in data/frames/")
