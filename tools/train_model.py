"""Train and evaluate site-aware IMU action and phase classifiers."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (classification_report, confusion_matrix, f1_score,
                             precision_recall_fscore_support)
from sklearn.model_selection import StratifiedGroupKFold
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

from imu_features import FEATURE_VERSION, feature_names, samples_to_windows

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "pose_labels.json"


def load_config(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def label_index(config: dict):
    return {action["id"]: body for body, value in config["body_parts"].items()
            for action in value.get("actions", [])}


def legacy_map(config: dict):
    return {filename.lower(): action["id"]
            for body in config["body_parts"].values()
            for action in body.get("actions", [])
            for filename in action.get("legacy_files", [])}


def read_recording(path: Path, config: dict) -> list[dict]:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        source = list(csv.DictReader(stream))
    if not source:
        return []
    action_bodies = label_index(config)
    action = legacy_map(config).get(path.name.lower())
    rows = []
    epoch = 0
    previous_seq = None
    previous_boot = None
    for index, raw in enumerate(source):
        sensor_uid = raw.get("sensor_uid") or raw.get("firmware_sensor_id") or raw.get("sensor_id") or "unknown"
        site = raw.get("mount_site") or raw.get("recorded_site") or raw.get("sensor_id") or "other"
        site = str(site).strip().lower()
        body_part = raw.get("body_part") or (action_bodies.get(action) if action else None) or "other"
        action_label = raw.get("action_label") or raw.get("pose_label") or action or ""
        body_part = str(body_part).strip().lower()
        action_label = str(action_label).strip()
        if site not in config["mount_sites"]:
            raise ValueError(f"{path} row {index + 2}: unknown mount_site {site!r}")
        if action_label in action_bodies and action_bodies[action_label] != body_part:
            raise ValueError(f"{path} row {index + 2}: action {action_label!r} belongs to "
                             f"{action_bodies[action_label]!r}, not body_part {body_part!r}")
        phase_label = raw.get("phase_label") or ""
        reviewed_value = str(raw.get("phase_annotation_reviewed") or "").strip().lower()
        has_review_metadata = "phase_annotation_reviewed" in raw and reviewed_value != ""
        phase_annotation_reviewed = (reviewed_value in ("true", "1", "yes")
                                     if has_review_metadata else None)
        # New recorder cue times are provisional until the annotator reviews
        # their boundaries. Older CSVs have no review column and retain their
        # previous explicit-label behavior.
        if has_review_metadata and not phase_annotation_reviewed:
            phase_label = ""
        try:
            calibration_method_version = int(raw.get("calibration_method_version") or 0)
            calibration_time_boot_ms = int(raw.get("calibration_time_boot_ms") or 0)
        except ValueError:
            calibration_method_version = calibration_time_boot_ms = 0
        try:
            boot_ms = int(float(raw.get("boot_ms") or raw.get("sample_ms") or 0))
            seq = int(float(raw.get("seq") or index))
            host_time = float(raw.get("host_time_s") or raw.get("host_timestamp_s") or index / config["sampling_hz"])
        except (TypeError, ValueError):
            continue
        if previous_seq is not None and (seq <= previous_seq or boot_ms < previous_boot):
            epoch += 1
        previous_seq, previous_boot = seq, boot_ms
        # Keep arrival order; a reboot begins a new stream epoch and cannot be
        # accidentally sorted back together by the reset boot-time clock.
        device_time = boot_ms / 1000 if boot_ms else host_time
        row = {
            "sensor_uid": sensor_uid,
            "mount_site": site,
            "body_part": body_part,
            "side": raw.get("side", ""),
            "subject_id": raw.get("subject_id", ""),
            "session_id": raw.get("session_id") or raw.get("recording_id") or path.stem,
            "trial_id": raw.get("trial_id") or raw.get("recording_id") or path.stem,
            "stream_epoch": f"{path.stem}:{epoch}",
            "calibration_id": raw.get("calibration_id") or "",
            "calibration_method_version": calibration_method_version,
            "calibration_time_boot_ms": calibration_time_boot_ms,
            "calibration_boot_id": raw.get("calibration_boot_id") or "",
            "calibrated": str(raw.get("calibrated", "false")).lower() in ("true", "1", "yes"),
            "relative_angle_valid": str(raw.get("relative_angle_valid", "false")).lower() in ("true", "1", "yes"),
            "relative_orientation_valid": str(raw.get("relative_orientation_valid", raw.get("calibrated", "false"))).lower() in ("true", "1", "yes"),
            "action_label": action_label,
            "phase_label": str(phase_label).strip(),
            "phase_annotation_source": raw.get("phase_annotation_source") or "legacy_explicit",
            "phase_annotation_reviewed": phase_annotation_reviewed,
            "boot_ms": boot_ms,
            "seq": seq,
            "device_time_s": device_time,
            "host_time_s": host_time,
            "valid": str(raw.get("valid", "true")).lower() in ("true", "1", "yes"),
        }
        for axis in "xyz":
            # Legacy/web captures use m/s^2 and rad/s. Firmware JSON ax..az are
            # in g; those are converted by serve_model before feature extraction.
            row[f"accel_{axis}"] = raw.get(f"accel_{axis}", "")
            row[f"gyro_{axis}"] = raw.get(f"gyro_{axis}", "")
        for key in ("relative_roll_deg", "relative_pitch_deg"):
            row[key] = raw.get(key, "")
        if row["relative_angle_valid"] and not row["relative_pitch_deg"]:
            row["relative_pitch_deg"] = raw.get("relative_tilt_deg", "")
        if row["relative_angle_valid"] and not row["relative_roll_deg"]:
            row["relative_roll_deg"] = "0"
        rows.append(row)
    return rows


def load_windows(inputs: list[Path], config: dict, seconds: float):
    result = []
    ignored_files = []
    for path in inputs:
        rows = read_recording(path, config)
        allowed = label_index(config)
        known = [row for row in rows if row["action_label"] in allowed]
        if not known:
            ignored_files.append(f"{path}: no action label or configured legacy filename mapping")
            continue
        use_angles = any(row["calibrated"] and row["relative_orientation_valid"]
                         and row["calibration_method_version"] >= 1
                         and row["calibration_id"] for row in known)
        # Preserve every sample as a boundary marker. Filtering uncalibrated or
        # unlabeled rows out first could stitch samples on either side together.
        rows = [{**row, "action_label": row["action_label"] if row["action_label"] in allowed else ""}
                for row in rows]
        if use_angles:
            rows = [{**row, "valid": row["valid"] and row["calibrated"]
                     and row["relative_orientation_valid"] and bool(row["calibration_id"])
                     and row["calibration_method_version"] >= 1}
                    for row in rows]
        if not use_angles:
            # Explicitly raw-only: never use uncalibrated relative angle zeroes.
            pass
        result.extend(samples_to_windows(rows, seconds, float(config["sampling_hz"]),
                                         float(config["prediction"]["gap_reset_seconds"]), use_angles))
    return result, ignored_files


def make_estimator(name: str, seed: int):
    if name == "random_forest":
        return RandomForestClassifier(n_estimators=300, max_depth=10, min_samples_leaf=2,
                                      class_weight="balanced_subsample", random_state=seed,
                                      n_jobs=1)
    return make_pipeline(StandardScaler(), SVC(C=2.0, kernel="rbf", class_weight="balanced",
                                               probability=True, random_state=seed))


def split_groups(windows: list[dict], target: str):
    labels = np.asarray([row[target] for row in windows])
    subjects = {row["subject_id"] for row in windows if row["subject_id"]}
    if len(subjects) >= 2 and all(any(row["subject_id"] == subject for row in windows) for subject in subjects):
        groups = np.asarray([row["subject_id"] or row["trial_id"] for row in windows])
        group_level = "subject_id"
    else:
        if any(row["session_id"] for row in windows):
            groups = np.asarray([row["session_id"] or row["trial_id"] for row in windows])
            group_level = "session_id"
        else:
            groups = np.asarray([row["trial_id"] for row in windows])
            group_level = "trial_id"
    unique_groups = len(set(groups))
    if unique_groups < 2 or len(set(labels)) < 2:
        return [], group_level, f"Need at least two groups and two classes; found {unique_groups} groups and {len(set(labels))} classes"
    folds = min(5, unique_groups)
    splitter = StratifiedGroupKFold(n_splits=folds, shuffle=True, random_state=42)
    valid = []
    for train, test in splitter.split(np.zeros(len(labels)), labels, groups):
        if len(set(labels[train])) < 2 or not len(test):
            continue
        valid.append((train, test))
    if not valid:
        return [], group_level, "No group-separated folds have enough classes to evaluate"
    return valid, group_level, ""


def evaluate_model(windows: list[dict], target: str, feature_cols: list[str],
                   name: str, seed: int = 42):
    folds, group_level, reason = split_groups(windows, target)
    labels = sorted({row[target] for row in windows})
    if not folds:
        return {"available": False, "reason": reason, "grouped_by": group_level,
                "labels": labels, "window_count": len(windows)}
    all_true, all_pred = [], []
    missing_test_labels = set()
    for train, test in folds:
        y = np.asarray([windows[index][target] for index in train])
        test_y = np.asarray([windows[index][target] for index in test])
        absent = set(labels) - set(y)
        if absent:
            missing_test_labels.update(absent)
            continue
        estimator = make_estimator(name, seed)
        x_train = np.asarray([[windows[index]["features"].get(col, 0.0) for col in feature_cols]
                              for index in train], dtype=float)
        x_test = np.asarray([[windows[index]["features"].get(col, 0.0) for col in feature_cols]
                             for index in test], dtype=float)
        estimator.fit(x_train, y)
        all_true.extend(test_y.tolist())
        all_pred.extend(estimator.predict(x_test).tolist())
    if not all_true:
        return {"available": False,
                "reason": "No group-separated fold retained every target class in training; collect more independent trials per class",
                "grouped_by": group_level, "labels": labels, "window_count": len(windows),
                "classes_missing_from_train_folds": sorted(missing_test_labels)}
    precision, recall, f1, support = precision_recall_fscore_support(
        all_true, all_pred, labels=labels, zero_division=0)
    return {
        "available": True,
        "grouped_by": group_level,
        "fold_count": len(folds),
        "window_count": len(windows),
        "accuracy": float(np.mean(np.asarray(all_true) == np.asarray(all_pred))),
        "macro_f1": float(f1_score(all_true, all_pred, labels=labels, average="macro", zero_division=0)),
        "per_class": {label: {"precision": float(precision[i]), "recall": float(recall[i]),
                              "f1": float(f1[i]), "support": int(support[i])}
                      for i, label in enumerate(labels)},
        "confusion_matrix": confusion_matrix(all_true, all_pred, labels=labels).tolist(),
        "labels": labels,
    }


def feature_units(columns: list[str]) -> dict[str, str]:
    units = {}
    for name in columns:
        base = name.split("__at_", 1)[0]
        if base.startswith("accel_"):
            units[name] = "m/s^3" if base.endswith("_slope_per_s") else "m/s^2"
        elif base.startswith("gyro_"):
            units[name] = "rad/s^2" if base.endswith("_slope_per_s") else "rad/s"
        elif base.startswith("relative_roll_deg_") or base.startswith("relative_pitch_deg_"):
            units[name] = "deg/s" if base.endswith("_slope_per_s") else "deg"
        else:
            units[name] = "unitless"
    return units


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="*", type=Path, help="Recording CSV files; defaults to whist/*.csv")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=ROOT / "models" / "pose_model")
    parser.add_argument("--window", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--model-version", default=None,
                        help="Optional stable model identifier recorded in the bundle and report")
    args = parser.parse_args()
    config = load_config(args.config)
    inputs = args.inputs or sorted((ROOT / "whist").glob("*.csv"))
    inputs = [path.resolve() for path in inputs if path.exists()]
    if not inputs:
        parser.error("No input CSV files found")
    windows, ignored = load_windows(inputs, config, args.window)
    action_windows = [row for row in windows if row["action_label"]]
    if not action_windows:
        print("No trainable labeled windows. Add action_label rows or configured legacy file mappings.", file=sys.stderr)
        return 2

    # Raw-only old captures cannot supply calibrated orientation features.
    use_angles = any(float(row["features"].get("relative_orientation_valid", 0)) == 1 for row in action_windows)
    sites = {row["mount_site"] for row in action_windows}
    bodies = {row["body_part"] for row in action_windows}
    sides = {row["side"] for row in action_windows}
    columns = feature_names(sites, use_angles, bodies, sides)
    stage_windows = [row for row in action_windows if row["phase_label"] in
                     {"neutral", "approaching", "holding", "returning"}]

    if use_angles:
        # Do not mix calibrated and raw-only windows under an angle-enabled schema.
        action_windows = [row for row in action_windows
                          if float(row["features"].get("relative_orientation_valid", 0.0)) == 1.0]
        stage_windows = [row for row in stage_windows
                         if float(row["features"].get("relative_orientation_valid", 0.0)) == 1.0]
    action_labels = sorted({row["action_label"] for row in action_windows})
    stage_labels = sorted({row["phase_label"] for row in stage_windows})
    if not action_windows:
        print("No eligible action windows after calibration and boundary filtering.", file=sys.stderr)
        return 2

    evaluations = {}
    for target, eligible in (("action_label", action_windows), ("phase_label", stage_windows)):
        if target == "phase_label" and len(stage_labels) < 2:
            evaluations[target] = {"available": False,
                                    "reason": "Fewer than two explicitly annotated phase classes; phase output will remain unknown",
                                    "labels": stage_labels, "window_count": len(stage_windows)}
            continue
        evaluations[target] = {
            model_name: evaluate_model(eligible, target, columns, model_name, args.seed)
            for model_name in ("random_forest", "svm_rbf")
        }

    def select_model(target):
        comparisons = evaluations.get(target, {})
        available = [(name, metrics) for name, metrics in comparisons.items()
                     if isinstance(metrics, dict) and metrics.get("available")]
        if not available:
            return "random_forest", "grouped evaluation unavailable; RF retained as prototype"
        best_score = max(metrics["macro_f1"] for _, metrics in available)
        # RF wins exact ties so the initial prototype remains the simple default.
        best = next((name for name, metrics in available
                     if metrics["macro_f1"] == best_score and name == "random_forest"), None)
        if best is None:
            best = next(name for name, metrics in available if metrics["macro_f1"] == best_score)
        return best, "selected by group-separated macro-F1; RF preferred on ties"

    selected_action_model, action_selection_reason = select_model("action_label")
    selected_phase_model, phase_selection_reason = select_model("phase_label")
    phase_evaluation_available = any(
        metrics.get("available") for metrics in evaluations.get("phase_label", {}).values()
        if isinstance(metrics, dict)
    )

    # Fit selected estimators on all eligible data after grouped comparison.
    action_model = make_estimator(selected_action_model, args.seed)
    action_x = np.asarray([[row["features"].get(col, 0.0) for col in columns]
                           for row in action_windows], dtype=float)
    action_y = [row["action_label"] for row in action_windows]
    action_model.fit(action_x, action_y)
    phase_model = None
    phase_estimator_name = None
    if len(stage_labels) >= 2 and phase_evaluation_available:
        phase_estimator_name = selected_phase_model
        phase_model = make_estimator(phase_estimator_name, args.seed)
        phase_x = np.asarray([[row["features"].get(col, 0.0) for col in columns]
                              for row in stage_windows], dtype=float)
        phase_model.fit(phase_x, [row["phase_label"] for row in stage_windows])

    calibrated_angles = use_angles
    calibration_versions = sorted({row["calibration_method_version"] for row in action_windows
                                    if row["calibrated"] and row["calibration_method_version"] > 0})
    bundle = {
        "bundle_version": 1,
        "feature_version": FEATURE_VERSION,
        "action_model": action_model,
        "phase_model": phase_model,
        "selected_action_estimator": selected_action_model,
        "selected_phase_estimator": phase_estimator_name,
        "model_selection": {"action": action_selection_reason,
                            "phase": phase_selection_reason},
        "feature_columns": columns,
        "feature_units": feature_units(columns),
        "action_labels": action_labels,
        "phase_labels": stage_labels if phase_model else [],
        "phase_model_available": phase_model is not None,
        "phase_annotation_policy": (
            "New recorder CSV phases require phase_annotation_reviewed=true; "
            "legacy CSVs without that column keep explicit phase labels"
        ),
        "mount_sites": sorted(sites),
        "body_parts": sorted(bodies),
        "sides": sorted(sides),
        "supported_pairs": sorted({(row["body_part"], row["mount_site"]) for row in action_windows}),
        "supported_contexts": sorted({(row["body_part"], row["mount_site"], row["side"])
                                       for row in action_windows}),
        "window_seconds": args.window,
        "sampling_hz": float(config["sampling_hz"]),
        "requires_calibration": calibrated_angles,
        "calibration_requirements": {
            "required": calibrated_angles,
            "orientation_field": "relative_roll_deg, relative_pitch_deg",
            "requires_calibration_id": calibrated_angles,
            "method_versions": calibration_versions,
        },
        "units": {"accel": "m/s^2", "gyro": "rad/s", "orientation": "degrees"},
        "score_kind": "uncalibrated estimator score; not a guaranteed probability",
        "model_version": args.model_version or f"pose-{selected_action_model.replace('_', '-')}-v1",
        "config": config,
        "evaluation": evaluations,
        "phase_unavailable_reason": None if phase_model else (
            evaluations.get("phase_label", {}).get("reason") or
            "No independent group-separated phase evaluation; stage will remain unknown"),
    }
    args.output.mkdir(parents=True, exist_ok=True)
    joblib.dump(bundle, args.output / "model_bundle.joblib")
    report = {
        "model_version": bundle["model_version"],
        "window_seconds": args.window,
        "sampling_hz": config["sampling_hz"],
        "action_windows": len(action_windows),
        "phase_windows": len(stage_windows),
        "action_labels": action_labels,
        "phase_labels": stage_labels,
        "phase_model_available": phase_model is not None,
        "phase_annotation_policy": bundle["phase_annotation_policy"],
        "requires_calibration": calibrated_angles,
        "selected_action_estimator": selected_action_model,
        "selected_phase_estimator": phase_estimator_name,
        "model_selection": bundle["model_selection"],
        "evaluation": evaluations,
        "ignored_files": ignored,
        "limitation": "Scores are not calibrated probabilities. Independent evaluation is unavailable wherever group-separated folds cannot cover all classes.",
    }
    (args.output / "evaluation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
