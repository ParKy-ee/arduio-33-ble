"""Generate a synthetic one-IMU neck-plank recording for pipeline prototyping.

The CSV is illustrative only. It is not participant data and does not reproduce
the Xsens measurements or classifier reported by Chen et al.
"""

import argparse
import csv
import math
import random
from pathlib import Path


G = 9.80665
RATE_HZ = 20  # Match the current ESP32 firmware's 50 ms sample interval.
CALIBRATION_S = 2.0
ANGLE_LIMIT_DEG = 12.0  # Same provisional threshold as src/main.cpp.
STATUS_HOLD_S = 0.5
FLEXION_SIGN = -1.0

# Time bounds are seconds. Head flexion maps to negative sensor tilt for the
# mounting convention documented in src/main.cpp; extension maps to positive.
PHASES = [
    (0.0, 2.0, "calibration", "CAL", 0.0, 0.0),
    (2.0, 12.0, "normal_01", "PC", 0.0, 0.0),
    (12.0, 12.6, "transition_01", "transition", 0.0, -18.0),
    (12.6, 22.6, "htfn_01", "HTFN", -18.0, -18.0),
    (22.6, 23.2, "transition_02", "transition", -18.0, 18.0),
    (23.2, 33.2, "htbn_01", "HTBN", 18.0, 18.0),
    (33.2, 33.8, "transition_03", "transition", 18.0, 0.0),
    (33.8, 43.8, "normal_02", "PC", 0.0, 0.0),
]


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def phase_at(time_s):
    for start, end, trial_id, label, start_deg, end_deg in PHASES:
        if start <= time_s < end:
            progress = (time_s - start) / (end - start)
            angle = start_deg + (end_deg - start_deg) * smoothstep(progress)
            return start, end, trial_id, label, angle
    return PHASES[-1][0], PHASES[-1][1], *PHASES[-1][2:5]


def wrap_deg(angle):
    return math.degrees(math.atan2(math.sin(math.radians(angle)),
                                   math.cos(math.radians(angle))))


def collect_samples(seed):
    rng = random.Random(seed)
    count = round(PHASES[-1][1] * RATE_HZ)
    samples = []
    previous_angle = 0.0

    for seq in range(count):
        time_s = seq / RATE_HZ
        start, end, trial_id, label, target_deg = phase_at(time_s)
        # Small simulated postural sway plus sensor noise.
        sway = 0.7 * math.sin(2.0 * math.pi * 0.22 * time_s)
        true_angle = target_deg + sway
        sensor_angle = true_angle + rng.gauss(0.0, 0.25)
        dt = 1.0 / RATE_HZ
        # Gyro measures physical rotation; don't differentiate accelerometer noise.
        gyro_x_dps = (true_angle - previous_angle) / dt + rng.gauss(0.0, 0.45)
        previous_angle = true_angle

        angle_rad = math.radians(sensor_angle)
        ax_g = rng.gauss(0.0, 0.004)
        ay_g = math.sin(angle_rad) + rng.gauss(0.0, 0.004)
        az_g = math.cos(angle_rad) + rng.gauss(0.0, 0.004)
        gx = math.radians(gyro_x_dps)
        gy = math.radians(rng.gauss(0.0, 0.25))
        gz = math.radians(rng.gauss(0.0, 0.25))
        accel_norm_g = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
        gyro_norm_dps = math.degrees(math.sqrt(gx * gx + gy * gy + gz * gz))
        tilt_deg = math.degrees(math.atan2(ay_g, az_g))
        trial_elapsed = time_s - start

        samples.append({
            "sensor_id": "neck",
            "trial_id": trial_id,
            "host_time_s": f"{time_s:.3f}",
            "boot_ms": round(time_s * 1000),
            "seq": seq + 1,
            "label": label,
            "accel_x": ax_g * G,
            "accel_y": ay_g * G,
            "accel_z": az_g * G,
            "gyro_x": gx,
            "gyro_y": gy,
            "gyro_z": gz,
            "tilt_deg": tilt_deg,
            "accel_norm_g": accel_norm_g,
            "gyro_norm_dps": gyro_norm_dps,
            "trial_elapsed_s": trial_elapsed,
            "steady": accel_norm_g >= 0.90 and accel_norm_g <= 1.10 and gyro_norm_dps <= 10.0,
        })

    calibration_count = round(CALIBRATION_S * RATE_HZ)
    sin_sum = sum(math.sin(math.radians(row["tilt_deg"]))
                  for row in samples[:calibration_count])
    cos_sum = sum(math.cos(math.radians(row["tilt_deg"]))
                  for row in samples[:calibration_count])
    neutral_deg = math.degrees(math.atan2(sin_sum, cos_sum))

    candidate_status = "normal"
    candidate_count = 0
    status = "calibrating"
    for row in samples:
        row["neutral_tilt_deg"] = neutral_deg
        row["neck_flex_deg"] = FLEXION_SIGN * wrap_deg(row["tilt_deg"] - neutral_deg)
        time_s = row["boot_ms"] / 1000.0
        row["calibration_samples"] = min(row["seq"], calibration_count)
        row["calibrated"] = time_s >= CALIBRATION_S

        if not row["calibrated"]:
            status = "calibrating"
            candidate_count = 0
        elif not row["steady"]:
            status = "moving"
            candidate_count = 0
        else:
            flex = row["neck_flex_deg"]
            candidate = "neck_flexion" if flex >= ANGLE_LIMIT_DEG else (
                "neck_extension" if flex <= -ANGLE_LIMIT_DEG else "normal")
            if candidate == candidate_status:
                candidate_count = min(candidate_count + 1, round(STATUS_HOLD_S * RATE_HZ))
            else:
                candidate_status = candidate
                candidate_count = 1
            if candidate_count >= round(STATUS_HOLD_S * RATE_HZ):
                status = candidate

        row["status"] = status
        row["valid"] = True
        row["study_window"] = (
            row["label"] in {"PC", "HTFN", "HTBN"}
            and 4.0 <= row["trial_elapsed_s"] < 7.0
        )

    return samples


def save_csv(samples, path):
    fields = [
        "sensor_id", "trial_id", "host_time_s", "boot_ms", "seq", "label",
        "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
        "tilt_deg", "neutral_tilt_deg", "neck_flex_deg", "accel_norm_g",
        "gyro_norm_dps", "trial_elapsed_s", "steady", "calibrated",
        "calibration_samples", "valid", "status", "study_window",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for row in samples:
            writer.writerow({key: row[key] for key in fields})


def save_plot(samples, path):
    import matplotlib.pyplot as plt

    times = [float(row["host_time_s"]) for row in samples]
    angles = [row["neck_flex_deg"] for row in samples]
    ay = [row["accel_y"] / G for row in samples]
    az = [row["accel_z"] / G for row in samples]
    path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True,
                             constrained_layout=True)
    axes[0].plot(times, angles, color="#245f86", linewidth=1.4,
                 label="Simulated neck_flex_deg")
    axes[0].axhline(ANGLE_LIMIT_DEG, color="#b44e3c", linestyle="--",
                    label="Prototype threshold (+/-12 deg)")
    axes[0].axhline(-ANGLE_LIMIT_DEG, color="#b44e3c", linestyle="--")
    axes[0].set_ylabel("Angle relative to neutral (deg)")
    axes[0].set_title("Synthetic neck IMU: normal and two instructed deviations")
    axes[0].legend(loc="upper right", frameon=False)
    axes[0].grid(alpha=0.2)

    axes[1].plot(times, ay, linewidth=1.1, label="accel_y (g)")
    axes[1].plot(times, az, linewidth=1.1, label="accel_z (g)")
    axes[1].set_xlabel("Time since boot (s)")
    axes[1].set_ylabel("Acceleration (g)")
    axes[1].legend(loc="upper right", frameon=False)
    axes[1].grid(alpha=0.2)
    for start, end, _, label, _, _ in PHASES:
        if label in {"PC", "HTFN", "HTBN"}:
            for axis in axes:
                axis.axvspan(start, end, color="#8aa98c", alpha=0.10, linewidth=0)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path,
                        default=Path("imu_output/simulated_neck.csv"))
    parser.add_argument("--plot", type=Path,
                        default=Path("imu_output/simulated_neck.png"))
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()

    samples = collect_samples(args.seed)
    save_csv(samples, args.csv)
    save_plot(samples, args.plot)
    print(f"Wrote {len(samples)} synthetic samples at {RATE_HZ} Hz to {args.csv}")
    print(f"Wrote plot to {args.plot}")
    print("Labels: PC=normal, HTFN=forward neck bend, HTBN=backward neck bend")
    print("All values are synthetic; they are not measured or copied from the paper.")


if __name__ == "__main__":
    main()
