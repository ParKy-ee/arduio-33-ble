"""Shared causal window feature extraction for IMU training and inference."""

from __future__ import annotations

import math
from collections import deque
from typing import Iterable, Mapping

import numpy as np

RAW_CHANNELS = tuple(f"{kind}_{axis}" for kind in ("accel", "gyro") for axis in "xyz")
FEATURE_VERSION = "imu-window-v2-site-gated"


def _finite(row: Mapping, key: str) -> float:
    value = row.get(key)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return number if math.isfinite(number) else float("nan")


def window_features(rows: Iterable[Mapping], site: str, expected_hz: float = 20.0,
                    use_calibrated_angles: bool = False, body_part: str = "other",
                    side: str = "unknown") -> dict[str, float]:
    """Compute one feature row from already validated, time-ordered samples.

    Raw accelerometer values are m/s^2 and gyro values are rad/s. Continuous
    channels are gated by mount site so different installation axes never share
    one feature column.
    Calibrated relative angles are used only when every row has a valid profile.
    """
    values = list(rows)
    if len(values) < 3:
        raise ValueError("A window needs at least three samples")
    times = np.asarray([_finite(row, "device_time_s") for row in values], dtype=float)
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError("Window has invalid or non-increasing device timestamps")
    duration = times[-1] - times[0]
    if duration <= 0 or duration > max(1.25, len(values) / expected_hz * 1.8):
        raise ValueError("Window duration is inconsistent with its sample count")

    features: dict[str, float] = {}
    arrays: dict[str, np.ndarray] = {}
    for channel in RAW_CHANNELS:
        data = np.asarray([_finite(row, channel) for row in values], dtype=float)
        if not np.all(np.isfinite(data)):
            raise ValueError(f"Window has missing {channel} values")
        arrays[channel] = data
        features[f"{channel}_mean"] = float(np.mean(data))
        features[f"{channel}_std"] = float(np.std(data))
        features[f"{channel}_rms"] = float(np.sqrt(np.mean(np.square(data))))
        features[f"{channel}_range"] = float(np.max(data) - np.min(data))
        features[f"{channel}_slope_per_s"] = float(np.polyfit(times - times[0], data, 1)[0])

    for kind in ("accel", "gyro"):
        magnitude = np.sqrt(sum(np.square(arrays[f"{kind}_{axis}"]) for axis in "xyz"))
        features[f"{kind}_magnitude_mean"] = float(np.mean(magnitude))
        features[f"{kind}_magnitude_std"] = float(np.std(magnitude))
        features[f"{kind}_magnitude_rms"] = float(np.sqrt(np.mean(np.square(magnitude))))

    features[f"site__{site}"] = 1.0
    features[f"body_part__{body_part or 'other'}"] = 1.0
    features[f"side__{side or 'unknown'}"] = 1.0

    calibrated = use_calibrated_angles and all(
        str(row.get("calibrated", "")).lower() in ("true", "1")
        and str(row.get("relative_orientation_valid", row.get("calibrated", ""))).lower() in ("true", "1")
        for row in values
    )
    if calibrated:
        angles = {}
        for angle in ("relative_roll_deg", "relative_pitch_deg"):
            data = np.asarray([_finite(row, angle) for row in values], dtype=float)
            if not np.all(np.isfinite(data)):
                raise ValueError(f"Calibrated window is missing {angle}")
            angles[angle] = data
        for angle, data in angles.items():
            features[f"{angle}_mean"] = float(np.mean(data))
            features[f"{angle}_std"] = float(np.std(data))
            features[f"{angle}_range"] = float(np.max(data) - np.min(data))
            features[f"{angle}_slope_per_s"] = float(np.polyfit(times - times[0], data, 1)[0])
        features["relative_orientation_valid"] = 1.0
    else:
        features["relative_orientation_valid"] = 0.0
    context = {name: value for name, value in features.items()
               if name.startswith(("site__", "body_part__", "side__"))
               or name == "relative_orientation_valid"}
    scoped = {f"{name}__at_{site}": value for name, value in features.items()
              if name not in context}
    return {**scoped, **context}


def feature_names(site_names: Iterable[str], use_calibrated_angles: bool,
                  body_parts: Iterable[str] = (), sides: Iterable[str] = ()) -> list[str]:
    base = []
    for channel in RAW_CHANNELS:
        base.extend(f"{channel}_{stat}" for stat in ("mean", "std", "rms", "range", "slope_per_s"))
    for kind in ("accel", "gyro"):
        base.extend(f"{kind}_magnitude_{stat}" for stat in ("mean", "std", "rms"))
    if use_calibrated_angles:
        for angle in ("relative_roll_deg", "relative_pitch_deg"):
            base.extend(f"{angle}_{stat}" for stat in ("mean", "std", "range", "slope_per_s"))
    names = [f"{feature}__at_{site}" for site in sorted(set(site_names)) for feature in base]
    names.extend(f"site__{name}" for name in sorted(set(site_names)))
    names.extend(f"body_part__{name}" for name in sorted(set(body_parts)))
    names.extend(f"side__{name}" for name in sorted(set(sides or ("unknown",))))
    names.append("relative_orientation_valid")
    return names


def samples_to_windows(rows: list[dict], window_seconds: float, expected_hz: float,
                       gap_reset_seconds: float = 0.15,
                       use_calibrated_angles: bool = False) -> list[dict]:
    """Extract non-overlapping windows; never cross trial, boot, gap or calibration."""
    if window_seconds <= 0 or expected_hz <= 0:
        raise ValueError("window_seconds and expected_hz must be positive")
    if not rows:
        return []
    # Input order is capture order; sorting boot-time clocks would hide a reboot.
    result: list[dict] = []
    current: list[dict] = []
    for row in rows:
        if not row.get("valid", True):
            current = []
            continue
        if current:
            prev = current[-1]
            boundary = any(row.get(key) != prev.get(key) for key in
                           ("subject_id", "session_id", "trial_id", "stream_epoch",
                            "calibration_id", "calibration_method_version", "sensor_uid",
                            "mount_site", "body_part", "side"))
            gap = float(row["device_time_s"]) - float(prev["device_time_s"])
            seq_gap = (row.get("seq") is not None and prev.get("seq") is not None
                       and int(row["seq"]) != int(prev["seq"]) + 1)
            if boundary or seq_gap or row.get("gap_before") in (True, "true", 1, "1") or gap <= 0 or gap > gap_reset_seconds:
                current = []
        current.append(row)
        if float(row["device_time_s"]) - float(current[0]["device_time_s"]) + 1e-9 < window_seconds:
            continue
        window_rows = current[:]
        while (len(window_rows) > 3 and
               float(window_rows[-1]["device_time_s"]) - float(window_rows[0]["device_time_s"])
               > window_seconds + 1e-9):
            window_rows.pop(0)
        duration = float(window_rows[-1]["device_time_s"]) - float(window_rows[0]["device_time_s"])
        expected_count = max(3, int(round(duration * expected_hz)))
        if len(window_rows) < expected_count * 0.65:
            current = []
            continue
        site = str(window_rows[-1].get("mount_site") or "other")
        try:
            features = window_features(window_rows, site, expected_hz, use_calibrated_angles,
                                       str(window_rows[-1].get("body_part") or "other"),
                                       str(window_rows[-1].get("side") or "unknown"))
        except ValueError:
            current = []
            continue
        action = window_rows[-1].get("action_label")
        phase = window_rows[-1].get("phase_label")
        # Keep only windows whose annotation is consistent from start to finish.
        if action and all(row.get("action_label") == action for row in window_rows):
            if phase and all(row.get("phase_label") == phase for row in window_rows):
                result.append({
                    "features": features,
                    "action_label": action,
                    "phase_label": phase,
                    "body_part": window_rows[-1].get("body_part", ""),
                    "mount_site": site,
                    "subject_id": window_rows[-1].get("subject_id", ""),
                    "session_id": window_rows[-1].get("session_id", ""),
                    "trial_id": window_rows[-1].get("trial_id", ""),
                    "stream_epoch": window_rows[-1].get("stream_epoch", 0),
                    "sensor_uid": window_rows[-1].get("sensor_uid", ""),
                    "calibration_id": window_rows[-1].get("calibration_id", ""),
                    "side": window_rows[-1].get("side") or "unknown",
                    "calibration_method_version": window_rows[-1].get("calibration_method_version", 0),
                    "calibrated": window_rows[-1].get("calibrated", False),
                })
            elif not phase:
                # Legacy recordings can train an action-only prototype.
                result.append({
                    "features": features,
                    "action_label": action,
                    "phase_label": "",
                    "body_part": window_rows[-1].get("body_part", ""),
                    "mount_site": site,
                    "subject_id": window_rows[-1].get("subject_id", ""),
                    "session_id": window_rows[-1].get("session_id", ""),
                    "trial_id": window_rows[-1].get("trial_id", ""),
                    "stream_epoch": window_rows[-1].get("stream_epoch", 0),
                    "sensor_uid": window_rows[-1].get("sensor_uid", ""),
                    "calibration_id": window_rows[-1].get("calibration_id", ""),
                    "side": window_rows[-1].get("side") or "unknown",
                    "calibration_method_version": window_rows[-1].get("calibration_method_version", 0),
                    "calibrated": window_rows[-1].get("calibrated", False),
                })
        # Stride is about half a window; samples are still causal.
        while len(current) > 1 and float(current[-1]["device_time_s"]) - float(current[1]["device_time_s"]) >= window_seconds * 0.5:
            current.pop(0)
    return result


class StreamBuffer:
    """Per-stream ring buffer that resets on duplicates, gaps and stream changes."""

    def __init__(self, window_seconds: float, expected_hz: float, gap_reset_seconds: float = 0.15):
        self.window_seconds = window_seconds
        self.expected_hz = expected_hz
        self.gap_reset_seconds = gap_reset_seconds
        self.rows: deque[dict] = deque(maxlen=max(8, int(math.ceil(window_seconds * expected_hz * 2))))
        self.last_seq = None

    def add(self, row: dict) -> bool:
        if not row.get("valid", True):
            self.reset()
            return False
        seq = int(row["seq"])
        if self.last_seq is not None and seq == self.last_seq:
            # Polling clients often ask faster than the sensor samples. Ignore
            # the repeated observation without disturbing smoothing state.
            return False
        if self.last_seq is not None and seq < self.last_seq:
            self.reset()
        if self.rows:
            prev = self.rows[-1]
            gap = float(row["device_time_s"]) - float(prev["device_time_s"])
            if gap <= 0 or gap > self.gap_reset_seconds or any(
                row.get(key) != prev.get(key) for key in
                ("stream_epoch", "calibration_id", "calibration_method_version", "sensor_uid",
                 "mount_site", "body_part", "side")
            ):
                self.reset()
            elif int(row["seq"]) != int(prev["seq"]) + 1:
                self.reset()
        self.rows.append(row)
        self.last_seq = seq
        return True

    def reset(self):
        self.rows.clear()
        self.last_seq = None

    def features(self, site: str, use_calibrated_angles: bool = False):
        if len(self.rows) < 3:
            return None
        if float(self.rows[-1]["device_time_s"]) - float(self.rows[0]["device_time_s"]) < self.window_seconds:
            return None
        rows = list(self.rows)
        while (len(rows) > 3 and
               float(rows[-1]["device_time_s"]) - float(rows[0]["device_time_s"])
               > self.window_seconds + 1e-9):
            rows.pop(0)
        return window_features(rows, site, self.expected_hz, use_calibrated_angles,
                               str(rows[-1].get("body_part") or "other"),
                               str(rows[-1].get("side") or "unknown"))
