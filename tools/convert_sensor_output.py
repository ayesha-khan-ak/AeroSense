"""
Convert a radar / LiDAR / audio model's CSV output into the AeroSense sensor contract.

Examples
--------
LiDAR or radar positions in the sensor frame (MMAUD style), columns t,x,y,z,confidence:
    python tools/convert_sensor_output.py lidar_preds.csv --sensor lidar --frame sensor_xyz

Radar in polar form, columns timestamp,range,azimuth,elevation,score:
    python tools/convert_sensor_output.py radar.csv --sensor radar --frame polar \
        --col t=timestamp --col confidence=score

Audio classifier, columns t,drone,confidence,label:
    python tools/convert_sensor_output.py audio.csv --sensor audio --frame none

Absolute timestamps (e.g. ROS 1706256803.22) are shifted so the first row is t = 0.
Output goes to data/<sensor>/... where server.py picks it up automatically.
"""
import argparse
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = {"radar": "data/radar/radar_tracks.json", "lidar": "data/lidar/lidar_tracks.json",
               "audio": "data/audio/audio_detections.json"}
NEEDED = {"sensor_xyz": ["x", "y", "z"], "polar": ["range", "azimuth", "elevation"],
          "enu": ["east", "north", "up"], "wgs84": ["lat", "lon", "alt"], "none": []}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--sensor", required=True, choices=["radar", "lidar", "audio"])
    ap.add_argument("--frame", default="sensor_xyz", choices=list(NEEDED))
    ap.add_argument("--col", action="append", default=[],
                    help="rename mapping contract_name=csv_column, e.g. t=timestamp")
    ap.add_argument("--out", default=None)
    ap.add_argument("--min-conf", type=float, default=0.0)
    args = ap.parse_args()

    rename = dict(c.split("=", 1) for c in args.col)
    col = lambda name: rename.get(name, name)          # noqa: E731
    rows = list(csv.DictReader(open(args.csv, newline="", encoding="utf-8")))
    if not rows:
        raise SystemExit("CSV is empty")
    t0 = float(rows[0][col("t")])
    dets = []
    for r in rows:
        conf = float(r.get(col("confidence"), 1.0) or 1.0)
        if conf < args.min_conf:
            continue
        d = {"t": round(float(r[col("t")]) - t0, 3), "confidence": round(conf, 3)}
        for k in NEEDED[args.frame]:
            d[k] = float(r[col(k)])
        for k in ("track_id", "label"):
            if col(k) in r and r[col(k)] != "":
                d[k] = r[col(k)]
        if args.sensor == "audio" and col("drone") in r:
            d["drone"] = str(r[col("drone")]).strip().lower() in ("1", "true", "yes", "drone")
        dets.append(d)

    out = ROOT / (args.out or DEFAULT_OUT[args.sensor])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"sensor": args.sensor, "coordinate_frame": args.frame,
                               "detections": dets}, indent=1), encoding="utf-8")
    print(f"Wrote {len(dets)} detections -> {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
