"""Show the simulated body and IMU magnitude agreement at each recorded point."""

import argparse
import csv
import math
from pathlib import Path


EDGES = [
    ("head", "neck"), ("neck", "chest"), ("chest", "lumbar"),
    ("lumbar", "pelvis"),
    ("chest", "left_upper_arm"), ("left_upper_arm", "left_forearm"),
    ("chest", "right_upper_arm"), ("right_upper_arm", "right_forearm"),
    ("pelvis", "left_thigh"), ("left_thigh", "left_shin"),
    ("pelvis", "right_thigh"), ("right_thigh", "right_shin"),
]


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def vector(row, prefix, point):
    try:
        return [float(row[f"{prefix}_{point}_{axis}"]) for axis in "xyz"]
    except (KeyError, ValueError):
        return None


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation", required=True, type=Path,
                        help="Simulation CSV containing pos_* and accel_*/gyro_* columns")
    parser.add_argument("--hardware", required=True, type=Path,
                        help="Wide CSV produced by merge.py")
    parser.add_argument("--offset", type=float, default=0,
                        help="Simulation time corresponding to hardware time zero, in seconds")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider
    except ImportError as exc:
        raise SystemExit("Install matplotlib first: python -m pip install matplotlib") from exc

    sim = read_csv(args.simulation)
    hw = read_csv(args.hardware)
    if not sim or not hw:
        parser.error("Both CSV files must contain samples")
    sim_times = [float(row["time_s"]) for row in sim]
    hw_times = [float(row["time_s"]) for row in hw]
    hw_points = {key[len("accel_"):-2] for key in hw[0]
                 if key.startswith("accel_") and key.endswith("_x")}
    sim_points = {key[len("pos_"):-2] for key in sim[0]
                  if key.startswith("pos_") and key.endswith("_x")}
    points = sorted(hw_points & sim_points)
    if not points:
        parser.error("No matching hardware point and simulation pos_* columns")

    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_axes([0.05, 0.16, 0.65, 0.78], projection="3d")
    note = fig.add_axes([0.73, 0.20, 0.25, 0.70])
    note.axis("off")
    slider_ax = fig.add_axes([0.15, 0.07, 0.70, 0.03])
    slider = Slider(slider_ax, "HW time (s)", hw_times[0], hw_times[-1],
                    valinit=hw_times[0])

    def nearest_index(values, target):
        return min(range(len(values)), key=lambda i: abs(values[i] - target))

    def draw(hw_time):
        hw_row = hw[nearest_index(hw_times, hw_time)]
        sim_time = hw_time + args.offset
        sim_row = sim[nearest_index(sim_times, sim_time)]
        ax.clear()
        locations = {point: vector(sim_row, "pos", point) for point in sim_points}
        for first, second in EDGES:
            a, b = locations.get(first), locations.get(second)
            if a and b:
                ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]],
                        color="0.55", linewidth=3)
        lines = []
        for point in points:
            pos = locations[point]
            measured = vector(hw_row, "accel", point)
            expected = vector(sim_row, "accel", point)
            if pos is None or measured is None or expected is None:
                continue
            difference = abs(norm(measured) - norm(expected))
            color = "tab:green" if difference < 2 else "tab:orange" if difference < 5 else "tab:red"
            ax.scatter(*pos, color=color, s=90)
            lines.append(f"{point}: |accel| diff {difference:.1f} m/s²")
        ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
               title=f"Simulation body at {sim_time:.2f} s")
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-0.5, 2.2)
        ax.set_zlim(0, 2.2)
        note.clear()
        note.axis("off")
        note.text(0, 1, "Hardware vs simulation\n" + "\n".join(lines) +
                  "\n\nGreen <2, orange <5, red ≥5 m/s²\n"
                  "Markers use simulated positions.\n"
                  "Align motions with --offset.", va="top", fontsize=9)
        fig.canvas.draw_idle()

    slider.on_changed(draw)
    draw(hw_times[0])
    plt.show()


if __name__ == "__main__":
    main()
