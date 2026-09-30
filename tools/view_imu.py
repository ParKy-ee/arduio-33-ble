"""Animate the orientation measured by one IMU capture CSV."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path


G = 9.80665
AXES = "xyz"


def read_capture(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No samples in {path}")

    samples = []
    previous_device_ms = None
    elapsed = 0.0
    first_host_time = None
    for row in rows:
        try:
            acceleration = [float(row[f"accel_{axis}"]) for axis in AXES]
            gyro = [float(row[f"gyro_{axis}"]) for axis in AXES]
        except (KeyError, TypeError, ValueError):
            continue

        if row.get("boot_ms"):
            device_ms = int(row["boot_ms"])
            if previous_device_ms is not None:
                elapsed += ((device_ms - previous_device_ms) & 0xFFFFFFFF) / 1000.0
            previous_device_ms = device_ms
            timestamp = elapsed
        else:
            host_time = float(row["host_time_s"])
            if first_host_time is None:
                first_host_time = host_time
            timestamp = host_time - first_host_time

        try:
            mag = [float(row[f"mag_{axis}"]) for axis in AXES]
        except (KeyError, TypeError, ValueError):
            mag = None

        samples.append({
            "time": timestamp,
            "accel": acceleration,
            "gyro": gyro,
            "mag": mag,
        })

    if not samples:
        raise ValueError("CSV needs accel_x/y/z and gyro_x/y/z columns")
    return samples


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def accel_angles(accel):
    ax, ay, az = accel
    return math.atan2(ay, az), math.atan2(-ax, math.sqrt(ay * ay + az * az))


def magnetic_yaw(mag, roll, pitch):
    mx, my, mz = mag
    mx_horizontal = mx * math.cos(pitch) + mz * math.sin(pitch)
    my_horizontal = (mx * math.sin(roll) * math.sin(pitch)
                     + my * math.cos(roll)
                     - mz * math.sin(roll) * math.cos(pitch))
    return math.atan2(-my_horizontal, mx_horizontal)


def estimate_angles(samples):
    roll, pitch = accel_angles(samples[0]["accel"])
    latest_mag = samples[0]["mag"]
    yaw = magnetic_yaw(latest_mag, roll, pitch) if latest_mag else 0.0
    angles = [[roll, pitch, yaw]]

    for previous, sample in zip(samples, samples[1:]):
        dt = max(0.0, sample["time"] - previous["time"])
        gx, gy, gz = sample["gyro"]  # Captured in rad/s.
        cos_pitch = math.cos(pitch)
        safe_cos_pitch = math.copysign(max(abs(cos_pitch), 0.05), cos_pitch or 1.0)
        tangent_pitch = math.sin(pitch) / safe_cos_pitch
        roll_rate = gx + math.sin(roll) * tangent_pitch * gy \
            + math.cos(roll) * tangent_pitch * gz
        pitch_rate = math.cos(roll) * gy - math.sin(roll) * gz
        yaw_rate = (math.sin(roll) * gy + math.cos(roll) * gz) / safe_cos_pitch
        roll = wrap(roll + roll_rate * dt)
        pitch = wrap(pitch + pitch_rate * dt)
        yaw = wrap(yaw + yaw_rate * dt)

        # Gravity corrects tilt only when acceleration is close to 1 g.
        accel = sample["accel"]
        accel_norm = math.sqrt(sum(value * value for value in accel))
        if 0.75 * G <= accel_norm <= 1.25 * G:
            measured_roll, measured_pitch = accel_angles(accel)
            correction = 1.0 - math.exp(-dt / 0.45)
            roll = wrap(roll + correction * wrap(measured_roll - roll))
            pitch = wrap(pitch + correction * wrap(measured_pitch - pitch))

        if sample["mag"] is not None:
            latest_mag = sample["mag"]
        if latest_mag and math.sqrt(sum(value * value for value in latest_mag)) > 1e-6:
            measured_yaw = magnetic_yaw(latest_mag, roll, pitch)
            correction = 1.0 - math.exp(-dt / 1.5)
            yaw = wrap(yaw + correction * wrap(measured_yaw - yaw))

        angles.append([roll, pitch, yaw])
    return angles


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="Per-board CSV from tools/capture.py")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (default: 1)")
    parser.add_argument("--json-output", type=Path,
                        help="Write numeric [roll, pitch, yaw] rows in radians")
    parser.add_argument("--export-only", action="store_true",
                        help="Write JSON without opening the animation window")
    parser.add_argument("--autoplay", action="store_true",
                        help="Start playback as soon as the animation window opens")
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed must be positive")
    if args.export_only and not args.json_output:
        parser.error("--export-only requires --json-output")

    try:
        samples = read_capture(args.csv)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    angles = estimate_angles(samples)
    times = [sample["time"] for sample in samples]
    has_mag = any(sample["mag"] is not None for sample in samples)

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        numeric_rows = [[round(value, 6) for value in row] for row in angles]
        args.json_output.write_text(json.dumps(numeric_rows, indent=2) + "\n",
                                    encoding="utf-8")
        print(f"Wrote {len(numeric_rows)} numeric [roll, pitch, yaw] rows (radians) to {args.json_output}")
    if args.export_only:
        return

    try:
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.widgets import Button, Slider
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError as exc:
        raise SystemExit("Install plotting dependencies: python -m pip install matplotlib numpy") from exc

    half = (0.48, 0.30, 0.10)
    vertices = np.array([
        [-half[0], -half[1], -half[2]], [half[0], -half[1], -half[2]],
        [half[0], half[1], -half[2]], [-half[0], half[1], -half[2]],
        [-half[0], -half[1], half[2]], [half[0], -half[1], half[2]],
        [half[0], half[1], half[2]], [-half[0], half[1], half[2]],
    ])
    faces = [[0, 1, 2, 3], [4, 7, 6, 5], [0, 4, 5, 1],
             [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    slider_ax = fig.add_axes([0.17, 0.11, 0.68, 0.035])
    play_ax = fig.add_axes([0.04, 0.095, 0.10, 0.065])
    slider = Slider(slider_ax, "Time (s)", 0.0, max(times[-1], 0.01), valinit=0.0)
    button = Button(play_ax, "Pause" if args.autoplay else "Play")
    playing = args.autoplay
    previous_tick = [None]

    def draw(time_value):
        play_time = max(0.0, min(times[-1], float(time_value)))
        right = bisect.bisect_right(times, play_time)
        left = max(0, right - 1)
        right = min(len(samples) - 1, right)
        span = times[right] - times[left]
        fraction = (play_time - times[left]) / span if span > 0 else 0.0
        roll, pitch, yaw = [
            angles[left][axis] + fraction * wrap(angles[right][axis] - angles[left][axis])
            for axis in range(3)
        ]
        roll, pitch, yaw = wrap(roll), wrap(pitch), wrap(yaw)
        matrix = np.array(rotation_matrix(roll, pitch, yaw))
        rotated = vertices @ matrix.T
        ax.clear()
        ax.add_collection3d(Poly3DCollection(
            [[rotated[i] for i in face] for face in faces],
            facecolors="#5ca8e8", edgecolors="#18334b", alpha=0.85,
        ))
        for axis, color, label in zip(matrix.T, ("r", "g", "b"), ("X", "Y", "Z")):
            ax.quiver(0, 0, 0, *(axis * 0.85), color=color, linewidth=2)
            ax.text(*(axis * 0.98), label, color=color, fontsize=11)
        ax.set(xlim=(-1.1, 1.1), ylim=(-1.1, 1.1), zlim=(-1.1, 1.1),
               xlabel="World X", ylabel="World Y", zlabel="World Z",
               title=(f"IMU orientation — {play_time:.2f} s\n"
                      f"Roll {math.degrees(roll):.1f}°   "
                      f"Pitch {math.degrees(pitch):.1f}°   "
                      f"Yaw {math.degrees(yaw):.1f}°"))
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=24, azim=35)
        fig.canvas.draw_idle()

    def toggle_play(_event):
        nonlocal playing
        playing = not playing
        previous_tick[0] = None
        button.label.set_text("Pause" if playing else "Play")

    def tick():
        if not playing:
            return
        import time
        now = time.monotonic()
        current_time = float(slider.val)
        if previous_tick[0] is None:
            previous_tick[0] = now
            return
        target_time = current_time + (now - previous_tick[0]) * args.speed
        previous_tick[0] = now
        if target_time >= times[-1]:
            slider.set_val(times[-1])
            toggle_play(None)
            return
        slider.set_val(target_time)

    slider.on_changed(draw)
    button.on_clicked(toggle_play)
    timer = fig.canvas.new_timer(interval=30)
    timer.add_callback(tick)
    timer.start()
    draw(0)
    note = "Magnetic heading correction enabled" if has_mag else \
        "No magnetometer samples: yaw can drift over time"
    fig.text(0.17, 0.045, note, fontsize=9)
    plt.show()


if __name__ == "__main__":
    main()
