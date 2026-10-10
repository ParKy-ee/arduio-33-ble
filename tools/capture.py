"""Capture one firmware IMU stream from USB serial with explicit metadata."""

from __future__ import annotations

import argparse
import csv
import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from view_imu import estimate_angles, read_capture


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "config" / "pose_labels.json").read_text(encoding="utf-8"))
MOUNT_SITES = tuple(CONFIG["mount_sites"])
BODY_PARTS = tuple(CONFIG["body_parts"])
FIELDS = [
    "sensor_uid", "sensor_id", "mount_site", "body_part", "side", "subject_id",
    "session_id", "trial_id", "action_label", "phase_label", "phase_elapsed_ms",
    "calibration_id", "calibration_method_version", "calibration_time_boot_ms", "calibration_boot_id",
    "calibrated", "relative_angle_valid", "relative_orientation_valid", "unit_accel", "unit_gyro",
    "host_time_iso", "host_time_s", "boot_ms", "seq", "stream_epoch", "gap_before",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
    "corrected_gyro_x", "corrected_gyro_y", "corrected_gyro_z", "roll_deg", "pitch_deg",
    "relative_roll_deg", "relative_pitch_deg", "relative_tilt_deg", "valid", "firmware_status",
]


def parse_line(line: str):
    values = line.split(",")
    if len(values) in (21, 23, 24) and re.fullmatch(r"[0-9a-fA-F]{12}", values[0]):
        try:
            boot, seq = int(values[1]), int(values[2])
            ax, ay, az, gx, gy, gz = map(float, values[3:9])
            calibrated = values[10].strip().lower() == "true"
            roll, pitch = map(float, values[12:14])
            cgx, cgy, cgz = map(float, values[14:17])
            rel_roll, rel_pitch = map(float, values[17:19])
            relative_valid = values[19].strip().lower() == "true"
            orientation_valid = values[20].strip().lower() == "true"
            calibration_method_version = int(values[21]) if len(values) == 23 and values[21] else (1 if calibrated else "")
            calibration_time_boot_ms = int(values[22]) if len(values) == 23 and values[22] else ""
            if len(values) == 24:
                calibration_method_version = int(values[21]) if values[21] else (1 if calibrated else "")
                calibration_time_boot_ms = int(values[22]) if values[22] else ""
                calibration_boot_id = int(values[23]) if values[23] else ""
            else:
                calibration_boot_id = ""
        except ValueError:
            return None
        return {"sensor_uid": values[0].lower(), "mount_site": None, "boot_ms": boot, "seq": seq,
                "accel": (ax * 9.80665, ay * 9.80665, az * 9.80665), "gyro": (gx, gy, gz),
                "corrected_gyro": (cgx, cgy, cgz), "calibration_id": values[9],
                "calibrated": calibrated, "status": values[11], "roll": roll, "pitch": pitch,
                "relative_roll": rel_roll, "relative_pitch": rel_pitch,
                "calibration_method_version": calibration_method_version,
                "calibration_time_boot_ms": calibration_time_boot_ms,
                "calibration_boot_id": calibration_boot_id,
                "relative_valid": relative_valid, "orientation_valid": orientation_valid, "format": "current"}
    if len(values) not in (9, 12):
        return None
    try:
        boot, seq = int(values[1]), int(values[2])
        accel = tuple(map(float, values[3:6]))
        gyro = tuple(map(float, values[6:9]))
        if len(values) == 12:
            [float(value) for value in values[9:] if value]
    except ValueError:
        return None
    # Legacy firmwares sent m/s^2, while current firmware emits a 12-hex MAC UID.
    return {"sensor_uid": values[0], "mount_site": values[0], "boot_ms": boot, "seq": seq,
            "accel": accel, "gyro": gyro, "corrected_gyro": ("", "", ""),
            "calibration_id": "", "calibrated": False, "status": "unknown",
            "calibration_method_version": "", "calibration_time_boot_ms": "",
            "calibration_boot_id": "",
            "roll": "", "pitch": "", "relative_roll": "", "relative_pitch": "",
            "relative_valid": False, "orientation_valid": False, "format": "legacy"}


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="COM port, such as COM5")
    parser.add_argument("--output", type=Path, default=Path("imu_output"))
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=0, help="0 records until Ctrl+C")
    parser.add_argument("--mount-site", choices=MOUNT_SITES, default="wrist")
    parser.add_argument("--body-part", choices=BODY_PARTS, default="wrist")
    parser.add_argument("--side", choices=("left", "right", "unknown"), default="unknown")
    parser.add_argument("--subject-id", default="P01")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--trial-id", default="")
    parser.add_argument("--action-label", default="", help="Action ID from config/pose_labels.json")
    parser.add_argument("--phase-label", choices=("", "neutral", "approaching", "holding", "returning"), default="")
    return parser


def main():
    args = make_parser().parse_args()
    if args.seconds < 0:
        raise SystemExit("--seconds must be non-negative")
    if args.action_label and args.action_label not in {
        action["id"] for body in CONFIG["body_parts"].values() for action in body.get("actions", [])
    }:
        raise SystemExit("Unknown --action-label; see config/pose_labels.json")
    if args.action_label and args.action_label not in {
        action["id"] for action in CONFIG["body_parts"][args.body_part].get("actions", [])
    }:
        raise SystemExit(f"Action {args.action_label!r} does not belong to body part {args.body_part!r}")
    side = args.side
    if side == "unknown" and args.mount_site.startswith("left_"):
        side = "left"
    elif side == "unknown" and args.mount_site.startswith("right_"):
        side = "right"
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install serial dependencies with: python -m pip install pyserial") from exc

    args.output.mkdir(parents=True, exist_ok=True)
    started = datetime.now(timezone.utc)
    session_id = args.session_id or started.strftime("%Y%m%dT%H%M%SZ")
    trial_id = args.trial_id or session_id + "_001"
    session_stamp = started.strftime("%Y%m%dT%H%M%SZ")
    file_part = 1
    stream_epoch = 0
    count = 0
    duplicates = 0
    first_seq = last_seq = first_boot = last_boot = None
    previous_key = None
    previous_calibration = None
    global_previous_seq = global_previous_boot = None
    handle = writer = None
    paths: list[Path] = []

    def open_segment(segment: int):
        nonlocal handle, writer
        path = args.output / f"{session_stamp}_{args.mount_site}_{args.action_label or 'unlabeled'}_part{segment:02d}.csv"
        handle = path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        paths.append(path)
        print(f"Writing {path}")

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as source:
            print(f"Reading {args.port}; stop with Ctrl+C")
            open_segment(file_part)
            deadline = time.monotonic() + args.seconds if args.seconds else None
            while deadline is None or time.monotonic() < deadline:
                raw = source.readline()
                received = datetime.now(timezone.utc)
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line or line.startswith("#") or line.startswith("sensor_uid,") or line.startswith("sensor_id,"):
                    continue
                sample = parse_line(line)
                if sample is None:
                    continue
                if sample["mount_site"] and sample["mount_site"] != args.mount_site:
                    print(f"Ignoring {sample['mount_site']}: configured mount site is {args.mount_site}")
                    continue
                key = (sample["boot_ms"], sample["seq"])
                if key == previous_key:
                    duplicates += 1
                    continue
                reboot = global_previous_boot is not None and sample["boot_ms"] < global_previous_boot
                if not reboot and global_previous_seq is not None and sample["seq"] <= global_previous_seq:
                    duplicates += 1
                    continue
                previous_key = key
                global_previous_seq = sample["seq"]
                global_previous_boot = sample["boot_ms"]
                calibration_id = sample["calibration_id"]
                if count and calibration_id != previous_calibration:
                    handle.close()
                    file_part += 1
                    stream_epoch += 1
                    trial_id = f"{args.trial_id or session_id + '_001'}_cal{file_part}"
                    open_segment(file_part)
                    first_seq = last_seq = first_boot = last_boot = None
                gap = last_seq is not None and sample["seq"] != last_seq + 1
                if reboot:
                    stream_epoch += 1
                row = {
                    "sensor_uid": sample["sensor_uid"], "sensor_id": sample["sensor_uid"],
                    "mount_site": args.mount_site, "body_part": args.body_part, "side": side,
                    "subject_id": args.subject_id, "session_id": session_id, "trial_id": trial_id,
                    "action_label": args.action_label, "phase_label": args.phase_label,
                    "phase_elapsed_ms": "", "calibration_id": calibration_id,
                    "calibration_method_version": sample["calibration_method_version"],
                    "calibration_time_boot_ms": sample["calibration_time_boot_ms"],
                    "calibration_boot_id": sample["calibration_boot_id"],
                    "calibrated": sample["calibrated"], "relative_angle_valid": sample["relative_valid"],
                    "relative_orientation_valid": sample["orientation_valid"],
                    "unit_accel": "m/s^2", "unit_gyro": "rad/s",
                    "host_time_iso": received.isoformat(), "host_time_s": received.timestamp(),
                    "boot_ms": sample["boot_ms"], "seq": sample["seq"], "stream_epoch": stream_epoch,
                    "gap_before": bool(gap), "valid": True, "firmware_status": sample["status"],
                    "roll_deg": sample["roll"], "pitch_deg": sample["pitch"],
                    "relative_roll_deg": sample["relative_roll"], "relative_pitch_deg": sample["relative_pitch"],
                    "relative_tilt_deg": sample["relative_pitch"],
                    "corrected_gyro_x": sample["corrected_gyro"][0],
                    "corrected_gyro_y": sample["corrected_gyro"][1],
                    "corrected_gyro_z": sample["corrected_gyro"][2],
                }
                for axis, value in zip("xyz", sample["accel"]):
                    row[f"accel_{axis}"] = value
                for axis, value in zip("xyz", sample["gyro"]):
                    row[f"gyro_{axis}"] = value
                writer.writerow(row)
                count += 1
                if count % 100 == 0:
                    handle.flush()
                if first_seq is None:
                    first_seq, first_boot = sample["seq"], sample["boot_ms"]
                last_seq, last_boot = sample["seq"], sample["boot_ms"]
                previous_calibration = calibration_id
    except KeyboardInterrupt:
        pass
    finally:
        if handle and not handle.closed:
            handle.close()
        if paths:
            print(f"Saved {count} samples in {len(paths)} calibration segment(s); duplicate samples ignored: {duplicates}")
            if first_seq is not None and last_boot and last_boot > first_boot:
                print(f"Estimated rate: {(last_seq-first_seq)*1000/(last_boot-first_boot):.2f} Hz")
            if len(paths) == 1 and count:
                try:
                    angles = estimate_angles(read_capture(paths[0]))
                    out = paths[0].with_name(paths[0].stem + "_orientation.json")
                    out.write_text(json.dumps(angles, indent=2) + "\n", encoding="utf-8")
                    print(f"Saved orientation rows to {out}")
                except Exception as error:
                    print(f"Orientation sidecar skipped: {error}")


if __name__ == "__main__":
    main()
