"""Generate a deterministic Nano 33 BLE Sense-like IMU CSV without hardware."""

import argparse
import csv
import json
import math
import time
from pathlib import Path

from capture import FIELDS, POINTS
from view_imu import estimate_angles, read_capture, rotation_matrix


GRAVITY = 9.80665
MAG_FIELD_WORLD_UT = (20.0, 0.0, 42.0)


def motion(time_s):
    """Return sensor-frame Euler angles and rates for a smooth multi-axis turn."""
    roll_amp = math.radians(38.0)
    pitch_amp = math.radians(26.0)
    yaw_sway_amp = math.radians(24.0)
    roll_hz = 0.20
    pitch_hz = 0.14
    yaw_sway_hz = 0.11
    yaw_rate = math.radians(30.0)

    roll = roll_amp * math.sin(2.0 * math.pi * roll_hz * time_s)
    pitch = pitch_amp * math.sin(2.0 * math.pi * pitch_hz * time_s)
    yaw = yaw_rate * time_s + yaw_sway_amp * math.sin(2.0 * math.pi * yaw_sway_hz * time_s)

    roll_dot = roll_amp * 2.0 * math.pi * roll_hz * math.cos(2.0 * math.pi * roll_hz * time_s)
    pitch_dot = pitch_amp * 2.0 * math.pi * pitch_hz * math.cos(2.0 * math.pi * pitch_hz * time_s)
    yaw_dot = yaw_rate + yaw_sway_amp * 2.0 * math.pi * yaw_sway_hz \
        * math.cos(2.0 * math.pi * yaw_sway_hz * time_s)

    # Euler-angle derivatives converted to angular velocity in the sensor frame.
    gyro = (
        roll_dot - yaw_dot * math.sin(pitch),
        pitch_dot * math.cos(roll) + yaw_dot * math.sin(roll) * math.cos(pitch),
        -pitch_dot * math.sin(roll) + yaw_dot * math.cos(roll) * math.cos(pitch),
    )
    return roll, pitch, yaw, gyro


def sensor_vector(rotation, world_vector):
    """Transform a world-frame vector into the rotating sensor frame."""
    return [
        sum(rotation[row][column] * world_vector[row] for row in range(3))
        for column in range(3)
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("imu_output/simulated_chest.csv"))
    parser.add_argument("--sensor-id", choices=sorted(POINTS), default="chest")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--mag-hz", type=float, default=10.0)
    args = parser.parse_args()
    if args.seconds <= 0 or args.hz <= 0 or args.mag_hz <= 0:
        parser.error("--seconds, --hz, and --mag-hz must all be positive")

    sample_count = int(round(args.seconds * args.hz)) + 1
    mag_interval = max(1, int(round(args.hz / args.mag_hz)))
    host_start = time.time()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for sequence in range(sample_count):
            time_s = sequence / args.hz
            roll, pitch, yaw, gyro = motion(time_s)
            rotation = rotation_matrix(roll, pitch, yaw)
            acceleration = sensor_vector(rotation, (0.0, 0.0, GRAVITY))
            magnetic = sensor_vector(rotation, MAG_FIELD_WORLD_UT)
            mag_values = magnetic if sequence % mag_interval == 0 else (None, None, None)

            row = {
                "sensor_id": args.sensor_id,
                "host_time_s": f"{host_start + time_s:.6f}",
                "boot_ms": round(time_s * 1000),
                "seq": sequence,
            }
            for axis, value in zip("xyz", acceleration):
                row[f"accel_{axis}"] = f"{value:.6f}"
            for axis, value in zip("xyz", gyro):
                row[f"gyro_{axis}"] = f"{value:.6f}"
            for axis, value in zip("xyz", mag_values):
                row[f"mag_{axis}"] = "" if value is None else f"{value:.6f}"
            writer.writerow(row)

    angles = estimate_angles(read_capture(args.output))
    orientation_path = args.output.with_name(args.output.stem + "_orientation.json")
    numeric_rows = [[round(value, 6) for value in row] for row in angles]
    orientation_path.write_text(json.dumps(numeric_rows, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {sample_count} simulated IMU rows to {args.output}")
    print(f"Wrote numeric [roll, pitch, yaw] radians to {orientation_path}")


if __name__ == "__main__":
    main()
