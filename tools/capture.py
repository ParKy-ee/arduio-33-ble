"""Capture one Nano 33 BLE IMU from USB serial into a per-point CSV file."""

import argparse
import csv
import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path


FIELDS = ["sensor_id", "host_time_s", "boot_ms", "seq", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]
POINTS = {
    "chest", "lumbar", "pelvis", "neck", "head",
    "left_upper_arm", "right_upper_arm", "left_forearm", "right_forearm",
    "left_thigh", "right_thigh", "left_shin", "right_shin",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="COM port, such as COM5")
    parser.add_argument("--output", type=Path, default=Path("imu_output"))
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=0, help="0 records until Ctrl+C")
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc

    args.output.mkdir(parents=True, exist_ok=True)
    started = datetime.now(timezone.utc)
    session = started.strftime("%Y%m%dT%H%M%SZ")
    handle = None
    writer = None
    point = None
    count = 0
    first_host = None

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as source:
            print(f"Reading {args.port}. Press Ctrl+C to stop.")
            deadline = time.monotonic() + args.seconds if args.seconds else None
            while deadline is None or time.monotonic() < deadline:
                raw = source.readline()
                received = time.time()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line or line.startswith("#") or line.startswith("sensor_id,"):
                    continue
                values = line.split(",")
                if len(values) != 9:
                    continue
                sensor_id = values[0]
                if sensor_id not in POINTS or not re.fullmatch(r"[a-z_]+", sensor_id):
                    continue
                try:
                    int(values[1]); int(values[2])
                    [float(value) for value in values[3:]]
                except ValueError:
                    continue
                if point is None:
                    point = sensor_id
                    path = args.output / f"{session}_{point}.csv"
                    handle = path.open("w", newline="", encoding="utf-8")
                    writer = csv.writer(handle)
                    writer.writerow(FIELDS)
                    first_host = received
                    print(f"Writing {path}")
                if sensor_id != point:
                    print(f"Ignoring unexpected sensor_id {sensor_id}")
                    continue
                writer.writerow([sensor_id, f"{received:.6f}", *values[1:]])
                count += 1
                if count % 100 == 0:
                    handle.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if handle:
            handle.close()
            metadata = {
                "sensor_id": point,
                "port": args.port,
                "started_utc": started.isoformat(),
                "first_host_time_s": first_host,
                "samples": count,
                "accelerometer_units": "m/s^2",
                "gyroscope_units": "rad/s",
                "sensor_frame": "sensor local axes; mount orientation must be calibrated",
                "position_note": "An IMU does not directly measure world position.",
            }
            path.with_name(path.stem + "_metadata.json").write_text(
                json.dumps(metadata, indent=2), encoding="utf-8"
            )
            print(f"Saved {count} samples to {path}")


if __name__ == "__main__":
    main()
