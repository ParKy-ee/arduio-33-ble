"""Capture one Nano 33 BLE IMU from USB serial into a per-point CSV file."""

import argparse
import csv
import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from view_imu import estimate_angles, read_capture


FIELDS = [
    "sensor_id", "host_time_s", "boot_ms", "seq",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
]
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
    first_boot_ms = None
    last_boot_ms = None
    first_sequence = None
    last_sequence = None

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
                if len(values) not in (9, 12):
                    continue
                sensor_id = values[0]
                if sensor_id not in POINTS or not re.fullmatch(r"[a-z_]+", sensor_id):
                    continue
                try:
                    int(values[1]); int(values[2])
                    [float(value) for value in values[3:9]]
                    if len(values) == 12:
                        [float(value) for value in values[9:] if value]
                except ValueError:
                    continue
                if point is None:
                    point = sensor_id
                    path = args.output / f"{session}_{point}.csv"
                    handle = path.open("w", newline="", encoding="utf-8")
                    writer = csv.writer(handle)
                    writer.writerow(FIELDS)
                    print(f"Writing {path}")
                if sensor_id != point:
                    print(f"Ignoring unexpected sensor_id {sensor_id}")
                    continue
                mag_values = values[9:12] if len(values) == 12 else ["", "", ""]
                writer.writerow([sensor_id, f"{received:.6f}", *values[1:9], *mag_values])
                count += 1
                boot_ms = int(values[1])
                if first_boot_ms is None:
                    first_boot_ms = boot_ms
                    first_sequence = int(values[2])
                last_boot_ms = boot_ms
                last_sequence = int(values[2])
                if count % 100 == 0:
                    handle.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if handle:
            handle.close()
            print(f"Saved {count} samples to {path}")
            rate = (
                round((last_sequence - first_sequence) * 1000
                      / (last_boot_ms - first_boot_ms), 2)
                if count > 1 and last_boot_ms > first_boot_ms else None
            )
            if rate is not None:
                gaps = max(0, last_sequence - first_sequence + 1 - count)
                print(f"Firmware sample rate: {rate} Hz; sequence gaps: {gaps}")

            angles = estimate_angles(read_capture(path))
            numeric_rows = [[round(value, 6) for value in row] for row in angles]
            orientation_path = path.with_name(path.stem + "_orientation.json")
            orientation_path.write_text(json.dumps(numeric_rows, indent=2) + "\n",
                                        encoding="utf-8")
            print(f"Saved {len(numeric_rows)} numeric [roll, pitch, yaw] rows (radians) to {orientation_path}")


if __name__ == "__main__":
    main()
