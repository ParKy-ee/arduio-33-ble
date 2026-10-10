"""Local web/API server for CSV replay and ESP32 live model inspection."""

from __future__ import annotations

import argparse
import ipaddress
import json
import sys
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from threading import Lock

import joblib
import numpy as np
from flask import Flask, jsonify, request, send_from_directory

from imu_features import StreamBuffer, feature_names

ROOT = Path(__file__).resolve().parents[1]
app = Flask(__name__)
bundle = None
states = {}
buffers = {}
live_cursors = {}
lock = Lock()


def reset_stream(key):
    state = states.setdefault(key, {})
    state.clear()
    if key in buffers:
        buffers[key].reset()


def reset_matching(site: str, body_part: str, side: str):
    suffix = f"|{site}|{body_part}|{side}"
    with lock:
        for key in list(buffers):
            parts = key.split("|")
            same_stream_context = (len(parts) >= 4 and parts[-3] == site and parts[-2] == body_part
                                   and (side == "unknown" or parts[-1] == side))
            if key.endswith(suffix) or same_stream_context:
                buffers[key].reset()
                states.pop(key, None)


def unknown(bundle_data, body_part="", site="", calibrated=False, reason="warming_up"):
    return {
        "body_part": body_part or "unknown", "action": "unknown", "stage": "unknown",
        "score": None, "score_kind": bundle_data.get("score_kind", "unavailable"),
        "held_ms": 0, "calibrated": bool(calibrated),
        "mount_site": site or "unknown", "model_version": bundle_data.get("model_version", "unavailable"),
        "reason": reason,
    }


def _class_probabilities(model, x):
    if model is None:
        return {}
    values = model.predict_proba(x)[0]
    return {str(label): float(probability) for label, probability in zip(model.classes_, values)}


def predict_from_features(features: dict, site: str, body_part: str,
                          calibrated: bool, stream_key: str | None = None,
                          device_time_s: float | None = None, side: str = "unknown"):
    item = bundle
    if item is None:
        return unknown({}, body_part, site, calibrated, "model_not_loaded")
    if site not in item["mount_sites"] or body_part not in item["body_parts"]:
        return unknown(item, body_part, site, calibrated, "unsupported_site_or_body_part")
    if (body_part, site) not in {tuple(pair) for pair in item.get("supported_pairs", [])}:
        return unknown(item, body_part, site, calibrated, "site_body_part_pair_not_seen_in_training")
    contexts = {tuple(context) for context in item.get("supported_contexts", [])}
    if contexts and (body_part, site, side or "unknown") not in contexts:
        return unknown(item, body_part, site, calibrated, "site_body_part_side_not_seen_in_training")
    if item.get("requires_calibration") and not calibrated:
        return unknown(item, body_part, site, calibrated, "calibration_required")

    x = np.asarray([[float(features.get(name, 0.0)) for name in item["feature_columns"]]], dtype=float)
    action_probs = _class_probabilities(item.get("action_model"), x)
    allowed = {entry["id"] for entry in item["config"]["body_parts"].get(body_part, {}).get("actions", [])}
    action_probs = {label: score for label, score in action_probs.items() if label in allowed}
    if not action_probs:
        return unknown(item, body_part, site, calibrated, "no_action_classes_for_body_part")
    action, action_score = max(action_probs.items(), key=lambda pair: pair[1])
    action_minimum = float(item["config"]["prediction"]["minimum_action_score"])
    if action_score < action_minimum:
        if stream_key is not None:
            states.pop(stream_key, None)
        return unknown(item, body_part, site, calibrated, "low_action_score")

    phase_model = item.get("phase_model")
    phase_probs = _class_probabilities(phase_model, x)
    if not phase_probs:
        stage, reason, phase_score = "unknown", item.get("phase_unavailable_reason") or "phase_model_unavailable", None
    else:
        stage, phase_score = max(phase_probs.items(), key=lambda pair: pair[1])
        if phase_score < float(item["config"]["prediction"]["minimum_phase_score"]):
            stage, reason = "unknown", "low_phase_score"
        else:
            reason = ""

    held_ms = 0
    if stream_key is not None:
        state = states.setdefault(stream_key, {})
        history = state.setdefault("action_history", deque(maxlen=int(item["config"]["prediction"]["smoothing_windows"])))
        history.append(action)
        action_counts = {candidate: history.count(candidate) for candidate in set(history)}
        majority = max(action_counts, key=action_counts.get)
        if action_counts[majority] >= 2 or "stable_action" not in state:
            state["stable_action"] = majority
        action = state["stable_action"]
        if stage in {"neutral", "approaching", "holding", "returning"}:
            phase_history = state.setdefault("phase_history", deque(maxlen=int(item["config"]["prediction"]["smoothing_windows"])))
            phase_history.append(stage)
            phase_counts = {candidate: phase_history.count(candidate) for candidate in set(phase_history)}
            phase_majority = max(phase_counts, key=phase_counts.get)
            if phase_counts[phase_majority] >= 2 or "stable_phase" not in state:
                state["stable_phase"] = phase_majority
            stage = state["stable_phase"]
        else:
            if "phase_history" in state:
                state["phase_history"].clear()
            state.pop("stable_phase", None)
        now = device_time_s if device_time_s is not None else 0.0
        if stage == "holding":
            if state.get("hold_action") != action:
                state["hold_action"] = action
                state["hold_started"] = now
            held_ms = max(0, int((now - float(state.get("hold_started", now))) * 1000))
            if held_ms < float(item["config"]["prediction"]["holding_stable_seconds"]) * 1000:
                stage, reason = "unknown", "confirming_holding"
        else:
            state.pop("hold_action", None)
            state.pop("hold_started", None)

    return {
        "body_part": body_part,
        "action": action,
        "stage": stage if stage in {"neutral", "approaching", "holding", "unknown", "returning"} else "unknown",
        "score": round(action_score, 4),
        "score_kind": item["score_kind"],
        "phase_score": None if phase_score is None else round(phase_score, 4),
        "held_ms": held_ms,
        "calibrated": bool(calibrated),
        "mount_site": site,
        "model_version": item["model_version"],
        "reason": reason,
    }


def normalize_live(payload: dict, site: str, body_part: str, side: str):
    boot_ms = int(payload.get("sample_ms", payload.get("boot_ms", 0)))
    row = {
        "sensor_uid": payload.get("sensor_uid") or payload.get("sensor_id") or "esp32",
        "mount_site": payload.get("mount_site") or site,
        "body_part": body_part,
        "side": (side if side != "unknown" else str(payload.get("side") or "unknown")),
        "seq": int(payload.get("seq", 0)),
        "device_time_s": boot_ms / 1000.0,
        "boot_ms": boot_ms,
        "stream_epoch": int(payload.get("boot_id", 0)),
        "calibration_id": payload.get("calibration_id") or "",
        "calibrated": bool(payload.get("calibrated", False)),
        "relative_angle_valid": bool(payload.get("relative_angle_valid", payload.get("calibrated", False))),
        "relative_orientation_valid": bool(payload.get("relative_orientation_valid", payload.get("calibrated", False))),
        "calibration_method_version": int(payload.get("calibration_method_version", 0)),
        "calibration_time_boot_ms": int(payload.get("calibration_time_boot_ms", 0)),
        "calibration_boot_id": int(payload.get("calibration_boot_id", 0)),
        "relative_roll_deg": payload.get("relative_roll_deg", 0.0),
        "relative_pitch_deg": payload.get("relative_pitch_deg", payload.get("wrist_flex_deg", 0.0)),
        "valid": bool(payload.get("valid", True)),
        "accel_x": float(payload["ax"]) * 9.80665,
        "accel_y": float(payload["ay"]) * 9.80665,
        "accel_z": float(payload["az"]) * 9.80665,
        "gyro_x": float(payload["gx"]),
        "gyro_y": float(payload["gy"]),
        "gyro_z": float(payload["gz"]),
    }
    return row


@app.get("/")
def index():
    return send_from_directory(ROOT, "model_test.html")


@app.get("/api/health")
def health():
    if bundle is None:
        return jsonify({"ready": False, "reason": "Train a model first with tools/train_model.py"}), 503
    return jsonify({"ready": True, "model_version": bundle["model_version"],
                    "feature_version": bundle["feature_version"],
                    "score_kind": bundle["score_kind"],
                    "window_seconds": bundle["window_seconds"],
                    "sampling_hz": bundle["sampling_hz"],
                    "gap_reset_seconds": bundle["config"]["prediction"]["gap_reset_seconds"],
                    "action_labels": bundle["action_labels"], "phase_labels": bundle["phase_labels"],
                    "phase_available": bundle.get("phase_model") is not None,
                    "phase_unavailable_reason": bundle.get("phase_unavailable_reason"),
                    "requires_calibration": bundle["requires_calibration"],
                    "calibration_requirements": bundle.get("calibration_requirements", {}),
                    "mount_sites": bundle["mount_sites"], "body_parts": bundle["body_parts"],
                    "sides": bundle.get("sides", []),
                    "supported_pairs": bundle.get("supported_pairs", []),
                    "supported_contexts": bundle.get("supported_contexts", []),
                    "body_part_config": bundle["config"]["body_parts"],
                    "legacy_file_actions": {filename: action["id"]
                        for body in bundle["config"]["body_parts"].values()
                        for action in body.get("actions", [])
                        for filename in action.get("legacy_files", [])}})


@app.post("/api/predict-window")
def predict_window():
    if bundle is None:
        return health()
    payload = request.get_json(force=True)
    rows = payload.get("rows", [])
    site = str(payload.get("mount_site", "other"))
    body_part = str(payload.get("body_part", "other"))
    if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
        return jsonify(unknown(bundle, body_part, site, False, "invalid_window_rows"))
    if len(rows) < 3:
        return jsonify(unknown(bundle, body_part, site, False, "window_too_short"))
    supported_calibration_versions = {int(value) for value in
                                      bundle.get("calibration_requirements", {}).get("method_versions", [])}
    try:
        required_seconds = float(bundle["window_seconds"])
        if float(rows[-1]["device_time_s"]) - float(rows[0]["device_time_s"]) + 1e-9 < required_seconds:
            return jsonify(unknown(bundle, body_part, site, False, "window_not_full"))
        rows = list(rows)
        while (len(rows) > 3 and
               float(rows[-1]["device_time_s"]) - float(rows[0]["device_time_s"])
               > required_seconds + 1e-9):
            rows.pop(0)
        for index, row in enumerate(rows):
            if row.get("mount_site", site) != site or row.get("body_part", body_part) != body_part:
                return jsonify(unknown(bundle, body_part, site, False, "window_crosses_site_or_body_part"))
            if bundle.get("requires_calibration") and (
                    not bool(row.get("calibrated"))
                    or not bool(row.get("relative_orientation_valid"))
                    or not row.get("calibration_id")
                    or int(row.get("calibration_method_version", 0)) not in supported_calibration_versions):
                return jsonify(unknown(bundle, body_part, site, False, "calibration_required_or_outdated"))
            if index == 0:
                continue
            previous = rows[index - 1]
            gap = float(row["device_time_s"]) - float(previous["device_time_s"])
            seq_gap = int(row["seq"]) != int(previous["seq"]) + 1
            boundary = any(row.get(key) != previous.get(key) for key in
                           ("calibration_id", "calibration_method_version", "calibrated",
                            "relative_orientation_valid", "stream_epoch",
                            "sensor_uid", "mount_site", "body_part", "side"))
            if gap <= 0 or gap > float(bundle["config"]["prediction"]["gap_reset_seconds"]) or seq_gap or boundary:
                return jsonify(unknown(bundle, body_part, site, False, "window_crosses_gap_or_stream_boundary"))
    except (KeyError, TypeError, ValueError):
        return jsonify(unknown(bundle, body_part, site, False, "invalid_window_metadata"))
    use_angles = bool(bundle.get("requires_calibration"))
    from imu_features import window_features
    try:
        features = window_features(rows, site, bundle["sampling_hz"], use_angles, body_part,
                                   str(rows[-1].get("side") or "unknown"))
    except (ValueError, KeyError, TypeError) as error:
        return jsonify(unknown(bundle, body_part, site, False, f"invalid_window: {error}"))
    calibrated = all(bool(row.get("calibrated", False)) for row in rows)
    stream_key = str(payload.get("stream_key") or "")
    try:
        requested_time = payload.get("device_time_s")
        device_time_s = float(rows[-1]["device_time_s"] if requested_time is None else requested_time)
    except (KeyError, TypeError, ValueError):
        return jsonify(unknown(bundle, body_part, site, calibrated, "invalid_window_timestamp"))
    if stream_key:
        previous = states.get(stream_key, {})
        stream_context = payload.get("stream_context") or {
            key: rows[-1].get(key, "") for key in
            ("sensor_uid", "mount_site", "body_part", "side", "subject_id", "session_id",
             "trial_id", "stream_epoch", "calibration_id", "calibration_method_version")
        }
        if (previous and (stream_context != previous.get("stream_context")
                          or int(payload.get("seq", -1)) != int(previous.get("last_input_seq", -2)) + 1
                          or device_time_s <= float(previous.get("last_input_time", 0.0))
                          or device_time_s - float(previous.get("last_input_time", 0.0)) >
                          float(bundle["config"]["prediction"]["gap_reset_seconds"]))):
            states.pop(stream_key, None)
        result = predict_from_features(features, site, body_part, calibrated, stream_key,
                                       device_time_s, str(rows[-1].get("side") or "unknown"))
        result.update({"calibration_id": rows[-1].get("calibration_id", ""),
                       "calibration_method_version": rows[-1].get("calibration_method_version", 0),
                       "calibration_time_boot_ms": rows[-1].get("calibration_time_boot_ms", 0),
                       "calibration_boot_id": rows[-1].get("calibration_boot_id", 0)})
        state = states.setdefault(stream_key, {})
        state["last_input_seq"] = int(payload.get("seq", rows[-1].get("seq", -1)))
        state["last_input_time"] = device_time_s
        state["stream_context"] = stream_context
        return jsonify(result)
    result = predict_from_features(features, site, body_part, calibrated,
                                   side=str(rows[-1].get("side") or "unknown"))
    result.update({"calibration_id": rows[-1].get("calibration_id", ""),
                   "calibration_method_version": rows[-1].get("calibration_method_version", 0),
                   "calibration_time_boot_ms": rows[-1].get("calibration_time_boot_ms", 0),
                   "calibration_boot_id": rows[-1].get("calibration_boot_id", 0)})
    return jsonify(result)


@app.get("/api/live")
def live():
    if bundle is None:
        return health()
    raw_ip = request.args.get("ip", "192.168.4.1").strip()
    site = request.args.get("mount_site", "")
    body_part = request.args.get("body_part", "")
    side = request.args.get("side", "unknown")
    try:
        address = ipaddress.ip_address(raw_ip)
        if not address.is_private:
            raise ValueError("Use a private ESP32 IP address")
        if not site or not body_part:
            raise ValueError("Select a mount site and body part")
        cursor_key = f"{address}|{site}|{body_part}|{side}"
        cursor = live_cursors.get(cursor_key, {})
        cursor_boot_id = int(cursor.get("boot_id", 0))
        cursor_seq = int(cursor.get("seq", 0))
        query = f"after_seq={cursor_seq}&boot_id={cursor_boot_id}"
        with urllib.request.urlopen(f"http://{address}/samples?{query}", timeout=2.0) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if not isinstance(payload, dict) or not isinstance(payload.get("samples"), list):
            raise ValueError("ESP /samples response is invalid; upload firmware with buffered sample support")
    except (KeyError, ValueError, OSError, urllib.error.URLError, json.JSONDecodeError) as error:
        old_cursor = live_cursors.pop(f"{raw_ip}|{site}|{body_part}|{side}", {})
        old_key = old_cursor.get("stream_key")
        if old_key:
            with lock:
                buffers.pop(old_key, None)
                states.pop(old_key, None)
        return jsonify(unknown(bundle, body_part, site, False, f"live_read_error: {error}")), 502

    current_boot_id = int(payload.get("boot_id", 0))
    latest_seq = int(payload.get("latest_seq", cursor_seq))
    incoming = payload["samples"]
    if incoming:
        newest_payload = incoming[-1]
        newest_row = normalize_live(newest_payload, site, body_part, side)
        if newest_row["mount_site"] != site:
            return jsonify(unknown(bundle, body_part, newest_row["mount_site"], newest_row["calibrated"],
                                   "selected_mount_site_does_not_match_esp")), 409
        newest_side = newest_payload.get("side")
        if side != "unknown" and newest_side not in (None, "unknown", side):
            return jsonify(unknown(bundle, body_part, site, newest_row["calibrated"],
                                   "selected_side_does_not_match_esp")), 409
        incoming = [sample for sample in incoming
                    if sample.get("mount_site") == site
                    and (side == "unknown" or sample.get("side") in (None, "unknown", side))]
    old_stream_key = cursor.get("stream_key")
    if ((cursor_boot_id and cursor_boot_id != current_boot_id) or bool(payload.get("dropped"))):
        if old_stream_key:
            with lock:
                buffers.pop(old_stream_key, None)
                states.pop(old_stream_key, None)

    if not incoming:
        if old_stream_key:
            with lock:
                cached = states.get(old_stream_key, {}).get("last_prediction")
                if cached is not None:
                    result = dict(cached)
                    result["duplicate_sample"] = True
                    return jsonify(result)
        return jsonify(unknown(bundle, body_part, site, False, "warming_up_or_waiting_for_sample"))

    latest_result = None
    latest_stream_key = old_stream_key
    with lock:
        supported_calibration_versions = {int(value) for value in
                                           bundle.get("calibration_requirements", {}).get("method_versions", [])}
        for sample_payload in incoming:
            row = normalize_live(sample_payload, site, body_part, side)
            if row["mount_site"] != site:
                result = unknown(bundle, body_part, row["mount_site"], row["calibrated"],
                                 "selected_mount_site_does_not_match_esp")
                return jsonify(result), 409
            device_side = sample_payload.get("side")
            if side != "unknown" and device_side not in (None, "unknown", side):
                result = unknown(bundle, body_part, site, row["calibrated"],
                                 "selected_side_does_not_match_esp")
                return jsonify(result), 409

            key = f"{row['sensor_uid']}|{row['mount_site']}|{body_part}|{row['side']}"
            latest_stream_key = key
            buffer = buffers.get(key)
            if buffer is None:
                buffer = StreamBuffer(bundle["window_seconds"], bundle["sampling_hz"],
                                      bundle["config"]["prediction"]["gap_reset_seconds"])
                buffers[key] = buffer
            was_calibrated = bool(row["calibrated"])
            if bundle.get("requires_calibration") and (
                    not was_calibrated or not row.get("relative_orientation_valid")
                    or not row.get("calibration_id")):
                states.pop(key, None)
                buffer.reset()
                latest_result = unknown(bundle, body_part, site, was_calibrated, "calibration_required")
                continue
            if (bundle.get("requires_calibration")
                    and int(row.get("calibration_method_version", 0)) not in supported_calibration_versions):
                states.pop(key, None)
                buffer.reset()
                latest_result = unknown(bundle, body_part, site, was_calibrated,
                                        "calibration_method_version_unavailable")
                continue
            if buffer.rows:
                previous = buffer.rows[-1]
                if (row["seq"] <= int(previous["seq"])
                        or row["seq"] != int(previous["seq"]) + 1
                        or row["device_time_s"] <= float(previous["device_time_s"])
                        or row["device_time_s"] - float(previous["device_time_s"]) > buffer.gap_reset_seconds
                        or row["calibration_id"] != previous.get("calibration_id")
                        or row["stream_epoch"] != previous.get("stream_epoch")
                        or row["side"] != previous.get("side")):
                    states.pop(key, None)
                    buffer.reset()
            if not buffer.add(row):
                continue
            state = states.setdefault(key, {})
            state["last_fresh_monotonic"] = time.monotonic()
            try:
                features = buffer.features(site, bundle.get("requires_calibration", False))
            except ValueError as error:
                latest_result = unknown(bundle, body_part, site, was_calibrated,
                                        f"invalid_window: {error}")
                continue
            if features is None:
                result = unknown(bundle, body_part, site, was_calibrated, "warming_up_or_stream_reset")
            else:
                result = predict_from_features(features, site, body_part, was_calibrated, key,
                                               row["device_time_s"], row["side"])
            result.update({"seq": row["seq"], "device_time_s": row["device_time_s"],
                           "calibration_id": row["calibration_id"],
                           "calibration_method_version": row["calibration_method_version"],
                           "calibration_time_boot_ms": row["calibration_time_boot_ms"],
                           "calibration_boot_id": row["calibration_boot_id"],
                           "firmware_status": sample_payload.get("status", "unknown"),
                           "firmware_status_is_not_ml_output": True})
            state["last_prediction"] = dict(result)
            latest_result = result

        live_cursors[cursor_key] = {
            "boot_id": current_boot_id,
            "seq": latest_seq,
            "stream_key": latest_stream_key,
        }
    return jsonify(latest_result or unknown(bundle, body_part, site, False, "no_usable_samples"))


def create_app(model_path: Path):
    global bundle
    if model_path.exists():
        bundle = joblib.load(model_path)
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models" / "pose_model" / "model_bundle.joblib")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if not args.model.exists():
        parser.error(f"Model bundle not found: {args.model}. Run tools/train_model.py first.")
    create_app(args.model)
    app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
