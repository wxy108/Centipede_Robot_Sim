"""
Sensor Data Viewer / Plotter for Centipede Simulation
======================================================
Loads .npz sensor files produced by test_models.py and generates
diagnostic plots for locomotion analysis.

Usage:
    python scripts/plot_sensors.py output/sensors/centipede_5seg_sensors.npz
    python scripts/plot_sensors.py output/sensors/centipede_5seg_sensors.npz --save
    python scripts/plot_sensors.py output/sensors/*_sensors.npz --compare
"""

import argparse
import sys
from pathlib import Path

import numpy as np


def load_sensor_file(filepath):
    """Load .npz sensor file and return dict + metadata."""
    d = dict(np.load(filepath, allow_pickle=True))
    n_seg = int(d.pop("n_segments", 0))
    return d, n_seg


def plot_single(sensor_data, n_segments, title="", save_path=None):
    """Plot comprehensive sensor dashboard for a single model run."""
    import matplotlib.pyplot as plt

    t = sensor_data["time"]
    pose = sensor_data["base_pose"]  # (N, 7)

    fig, axes = plt.subplots(4, 2, figsize=(16, 14))
    fig.suptitle(f"Centipede {n_segments}-Segment Locomotion — {title}",
                 fontsize=14, fontweight="bold")

    # --- 1. Trajectory XY ---
    ax = axes[0, 0]
    ax.plot(pose[:, 0], pose[:, 1], "b-", linewidth=0.8)
    ax.plot(pose[0, 0], pose[0, 1], "go", markersize=8, label="start")
    ax.plot(pose[-1, 0], pose[-1, 1], "r*", markersize=10, label="end")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory (top-down)")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # --- 2. Z-height over time ---
    ax = axes[0, 1]
    ax.plot(t, pose[:, 2], "b-", linewidth=0.8)
    ax.axhline(pose[0, 2], color="g", linestyle="--", alpha=0.5, label="start Z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Z (m)")
    ax.set_title("Body height (Z)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 3. IMU accelerometer ---
    ax = axes[1, 0]
    for ch, label in [(0, "ax"), (1, "ay"), (2, "az")]:
        key = f"imu_acc_{ch}"
        if key in sensor_data:
            ax.plot(t, sensor_data[key], linewidth=0.5, label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("IMU Accelerometer")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 4. IMU gyroscope ---
    ax = axes[1, 1]
    for ch, label in [(0, "gx"), (1, "gy"), (2, "gz")]:
        key = f"imu_gyro_{ch}"
        if key in sensor_data:
            ax.plot(t, sensor_data[key], linewidth=0.5, label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular vel (rad/s)")
    ax.set_title("IMU Gyroscope")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 5. Joint positions (shoulders only, for clarity) ---
    ax = axes[2, 0]
    for seg in range(1, min(n_segments + 1, 7)):
        key = f"jpos_Sh{seg}R"
        if key in sensor_data:
            ax.plot(t, np.degrees(sensor_data[key]),
                    linewidth=0.5, label=f"Sh{seg}R")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Shoulder Joint Positions (R)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- 6. Actuator forces (shoulders) ---
    ax = axes[2, 1]
    for seg in range(1, min(n_segments + 1, 7)):
        key = f"afrc_Sh{seg}R"
        if key in sensor_data:
            ax.plot(t, sensor_data[key], linewidth=0.5, label=f"Sh{seg}R")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.set_title("Shoulder Actuator Forces (R)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- 7. Foot contact (touch sensors) ---
    ax = axes[3, 0]
    for seg in range(1, min(n_segments + 1, 9)):
        for side in ["R", "L"]:
            key = f"touch_{seg}{side}"
            if key in sensor_data:
                offset = (seg - 1) * 2 + (0 if side == "R" else 1)
                ax.fill_between(t, offset, offset + sensor_data[key] * 0.8,
                                alpha=0.6, label=f"{seg}{side}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Foot (stacked)")
    ax.set_title("Foot Contact Pattern")
    ax.set_yticks([])
    ax.grid(True, alpha=0.3, axis="x")

    # --- 8. Link yaw joints (spine undulation) ---
    ax = axes[3, 1]
    for seg in range(1, n_segments):
        key = f"jpos_Lk{seg}Y"
        if key in sensor_data:
            ax.plot(t, np.degrees(sensor_data[key]),
                    linewidth=0.5, label=f"Lk{seg}Y")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Inter-Segment Link Yaw")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Plot saved: {save_path}")
    else:
        plt.show()


def plot_compare(files, save_path=None):
    """Compare trajectory and z-height across multiple models."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle("Multi-Model Comparison", fontsize=14, fontweight="bold")

    for fpath in files:
        data, n_seg = load_sensor_file(fpath)
        t = data["time"]
        pose = data["base_pose"]
        label = f"{n_seg}-seg"

        # XY trajectory
        axes[0].plot(pose[:, 0], pose[:, 1], linewidth=0.8, label=label)

        # Z over time
        axes[1].plot(t, pose[:, 2], linewidth=0.8, label=label)

        # Distance over time
        dx = pose[:, 0] - pose[0, 0]
        dy = pose[:, 1] - pose[0, 1]
        dist = np.sqrt(dx**2 + dy**2)
        axes[2].plot(t, dist, linewidth=0.8, label=label)

    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].set_title("Trajectory (XY)")
    axes[0].set_aspect("equal")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Z (m)")
    axes[1].set_title("Body Height")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Distance (m)")
    axes[2].set_title("Cumulative Travel Distance")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Comparison plot saved: {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot centipede sensor data")
    parser.add_argument("files", nargs="+", help=".npz sensor data files")
    parser.add_argument("--save", action="store_true",
                        help="Save plots as PNG instead of showing")
    parser.add_argument("--compare", action="store_true",
                        help="Compare multiple models side-by-side")
    args = parser.parse_args()

    files = [Path(f) for f in args.files]
    for f in files:
        if not f.exists():
            print(f"Error: {f} not found")
            sys.exit(1)

    output_dir = files[0].parent

    if args.compare and len(files) > 1:
        save_path = output_dir / "comparison.png" if args.save else None
        plot_compare(files, save_path)
    else:
        for fpath in files:
            data, n_seg = load_sensor_file(fpath)
            save_path = fpath.with_suffix(".png") if args.save else None
            plot_single(data, n_seg, title=fpath.stem, save_path=save_path)
