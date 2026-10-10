"""Join actual IMU samples by receive time without resampling or cloning rows."""

from __future__ import annotations

import argparse
import csv
import glob
from bisect import bisect_left
from pathlib import Path

from capture import MOUNT_SITES


SIGNALS = tuple(f"{signal}_{axis}" for signal in ("accel", "gyro", "corrected_gyro")
                for axis in "xyz")
METADATA = ("sensor_uid", "body_part", "side", "subject_id", "session_id", "trial_id",
            "action_label", "phase_label", "phase_elapsed_ms", "calibration_id",
            "calibration_method_version", "calibration_time_boot_ms", "calibration_boot_id", "calibrated",
            "relative_angle_valid", "relative_orientation_valid", "unit_accel", "unit_gyro",
            "boot_ms", "seq", "stream_epoch", "gap_before", "valid", "firmware_status",
            "roll_deg", "pitch_deg", "relative_roll_deg", "relative_pitch_deg", "relative_tilt_deg")


def load_recording(path: Path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"Empty recording: {path}")
    sites = {row.get("mount_site") or row.get("recorded_site") or row.get("sensor_id") for row in rows}
    sites.discard(None)
    if len(sites) != 1 or next(iter(sites)) not in MOUNT_SITES:
        raise ValueError(f"Expected exactly one configured mount_site in {path}: {sites}")
    for row in rows:
        if not row.get("host_time_s"):
            raise ValueError(f"Missing host_time_s in {path}")
        row["host_time_s"] = float(row["host_time_s"])
    rows.sort(key=lambda row: row["host_time_s"])
    return next(iter(sites)), rows


def nearest_unused(rows, timestamps, used, target, tolerance):
    index = bisect_left(timestamps, target)
    candidates = [i for i in range(max(0, index - 1), min(len(rows), index + 2)) if i not in used]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(timestamps[i] - target))
    if abs(timestamps[best] - target) > tolerance:
        return None
    used.add(best)
    return rows[best]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="One recording per sensor; wildcards accepted")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--reference-site", default="", help="Site whose actual timestamps define output rows")
    parser.add_argument("--tolerance-ms", type=float, default=35)
    args = parser.parse_args()
    if args.tolerance_ms < 0:
        parser.error("--tolerance-ms must be nonnegative")
    paths = [Path(match) for pattern in args.inputs for match in glob.glob(pattern)]
    if not paths:
        parser.error("No input CSV files matched")
    recordings = {}
    for path in paths:
        site, rows = load_recording(path)
        if site in recordings:
            parser.error(f"Multiple files for mount_site={site}; select one per synchronized trial")
        recordings[site] = rows
    reference = args.reference_site or max(recordings, key=lambda site: len(recordings[site]))
    if reference not in recordings:
        parser.error(f"Reference site {reference!r} is not among the inputs")

    fields = ["host_time_s", "reference_site"]
    for site in sorted(recordings):
        fields.extend(f"{name}_{site}" for name in (*METADATA, *SIGNALS))
    timestamps = {site: [row["host_time_s"] for row in rows] for site, rows in recordings.items()}
    used = {site: set() for site in recordings}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for reference_index, ref_row in enumerate(recordings[reference]):
            target = ref_row["host_time_s"]
            output = {"host_time_s": f"{target:.6f}", "reference_site": reference}
            for site, rows in recordings.items():
                match = ref_row if site == reference else nearest_unused(
                    rows, timestamps[site], used[site], target, args.tolerance_ms / 1000.0)
                if site == reference:
                    used[site].add(reference_index)
                if match is None:
                    continue
                for field in (*METADATA, *SIGNALS):
                    output[f"{field}_{site}"] = match.get(field, "")
            writer.writerow(output)
            count += 1
    print(f"Wrote {count} rows at actual {reference} samples; unmatched sensor fields remain blank.")
    print("No interpolation or repeated samples were generated.")


if __name__ == "__main__":
    main()
