"""Align per-point IMU recordings into the wide CSV shape of imu_samples.csv."""

import argparse
import csv
import glob
from bisect import bisect_left
from pathlib import Path

from capture import POINTS


AXES = ("x", "y", "z")


def load_recording(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"Empty recording: {path}")
    points = {row["sensor_id"] for row in rows}
    if len(points) != 1 or next(iter(points)) not in POINTS:
        raise ValueError(f"Expected one known sensor_id in {path}: {points}")
    rows.sort(key=lambda row: float(row["host_time_s"]))
    return next(iter(points)), rows


def nearest(rows, times, target, tolerance):
    index = bisect_left(times, target)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(rows)]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(times[i] - target))
    return rows[best] if abs(times[best] - target) <= tolerance else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="One captured CSV per body point; wildcards accepted")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--hz", type=float, default=100)
    parser.add_argument("--tolerance-ms", type=float, default=30)
    args = parser.parse_args()
    if args.hz <= 0 or args.tolerance_ms < 0:
        parser.error("--hz must be positive and tolerance must be nonnegative")

    recordings = {}
    paths = [Path(match) for pattern in args.inputs for match in glob.glob(pattern)]
    if not paths:
        parser.error("No input CSV files matched")
    for path in paths:
        point, rows = load_recording(path)
        if point in recordings:
            parser.error(f"Duplicate point {point}; choose one recording per point")
        recordings[point] = rows

    times = {point: [float(row["host_time_s"]) for row in rows]
             for point, rows in recordings.items()}
    start = max(values[0] for values in times.values())
    end = min(values[-1] for values in times.values())
    if end < start:
        parser.error("Recordings do not overlap in host time")

    points = sorted(recordings)
    fields = ["time_s"] + [f"{signal}_{point}_{axis}"
                             for point in points
                             for signal in ("accel", "gyro", "mag")
                             for axis in AXES]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for frame in range(int((end - start) * args.hz) + 1):
            target = start + frame / args.hz
            output = {"time_s": f"{frame / args.hz:.3f}"}
            for point in points:
                row = nearest(recordings[point], times[point], target,
                              args.tolerance_ms / 1000)
                if row:
                    for signal in ("accel", "gyro", "mag"):
                        for axis in AXES:
                            source = f"{signal}_{axis}"
                            if source in row:
                                output[f"{signal}_{point}_{axis}"] = row[source]
            writer.writerow(output)
            count += 1
    print(f"Wrote {count} synchronized rows for {', '.join(points)} to {args.output}")
    print("Blank cells mean no sample within the requested time tolerance.")


if __name__ == "__main__":
    main()
