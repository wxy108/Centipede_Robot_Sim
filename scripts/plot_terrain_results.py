#!/usr/bin/env python3
"""
plot_terrain_results.py — Bar plots of terrain benchmark metrics.

Reads trial_results.csv produced by test_terrain.py and plots, for each
centipede model (segment count):

    - mean velocity  (m/s)      = distance / duration
    - mean pitch deviation (°)
    - mean roll  deviation (°)
    - composite score           = distance / (1 + (pitch+roll)/10)

Error bars are std across seeds (0 when only 1 seed was run).

Usage:
    python scripts/plot_terrain_results.py                       # default run
    python scripts/plot_terrain_results.py --base 1.0 --levels 3
    python scripts/plot_terrain_results.py --duration 10.0
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_DIR = PROJECT_ROOT / "output" / "terrain_tests"


def load_results(csv_path):
    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            if r.get("status") != "OK":
                continue
            rows.append(r)
    return rows


def aggregate(rows, duration):
    """Return dict: n_seg -> (mean, std) for velocity/pitch/roll/score."""
    by_seg = {}
    for r in rows:
        n = int(r["segments"])
        by_seg.setdefault(n, []).append(r)

    stats = {}
    for n, trials in sorted(by_seg.items()):
        dist = np.array([float(t["distance"]) for t in trials])
        pitch = np.array([float(t["mean_pitch_dev"]) for t in trials])
        roll = np.array([float(t["mean_roll_dev"]) for t in trials])
        score = np.array([float(t["composite_score"]) for t in trials])
        vel = dist / duration
        stats[n] = {
            "velocity":  (vel.mean(),   vel.std()),
            "pitch":     (pitch.mean(), pitch.std()),
            "roll":      (roll.mean(),  roll.std()),
            "score":     (score.mean(), score.std()),
            "n_trials":  len(trials),
        }
    return stats


def plot_all(stats, out_dir, title_suffix=""):
    """One line chart per metric, saved as a separate PNG in out_dir."""
    segs = sorted(stats.keys())
    x = np.array(segs, dtype=float)  # use segment count on X so spacing is truthful

    def mean_std(key):
        m = np.array([stats[s][key][0] for s in segs])
        e = np.array([stats[s][key][1] for s in segs])
        return m, e

    metrics = [
        ("velocity", "Mean velocity (m/s)",      "tab:blue",   "velocity_vs_segments.png",   "{:.3f}"),
        ("pitch",    "Mean pitch deviation (°)", "tab:orange", "pitch_vs_segments.png",      "{:.1f}"),
        ("roll",     "Mean roll deviation (°)",  "tab:green",  "roll_vs_segments.png",       "{:.1f}"),
        ("score",    "Composite score",          "tab:red",    "score_vs_segments.png",      "{:.3f}"),
    ]

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for key, ylabel, color, fname, fmt in metrics:
        m, e = mean_std(key)

        fig, ax = plt.subplots(figsize=(7, 4.8))
        ax.errorbar(x, m, yerr=e, fmt="-o", color=color,
                    markersize=8, linewidth=2, capsize=5,
                    markerfacecolor=color, markeredgecolor="black",
                    markeredgewidth=0.8)

        # annotate each point
        for xi, yi in zip(x, m):
            ax.annotate(fmt.format(yi), xy=(xi, yi),
                        xytext=(0, 10), textcoords="offset points",
                        ha="center", fontsize=9)

        ax.set_xlabel("Number of segments")
        ax.set_ylabel(ylabel)
        ax.set_title(f"{ylabel} vs segment count{title_suffix}",
                     fontsize=12, fontweight="bold")
        ax.set_xticks(x)
        ax.set_xticklabels([f"{int(s)}" for s in x])
        ax.grid(True, alpha=0.3)

        # extra headroom for the annotations
        ymin, ymax = ax.get_ylim()
        ax.set_ylim(ymin, ymax + 0.08 * (ymax - ymin))

        if key == "score":
            fig.text(0.5, 0.01,
                     r"score = distance / (1 + (pitch_dev + roll_dev) / 10)",
                     ha="center", fontsize=9, style="italic", color="gray")
            plt.tight_layout(rect=[0, 0.04, 1, 1])
        else:
            plt.tight_layout()

        out_path = out_dir / fname
        plt.savefig(out_path, dpi=150)
        print(f"  Saved: {out_path}")
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", type=float, default=1.0,
                    help="Terrain base height (inch) used when running test_terrain.py")
    ap.add_argument("--levels", type=int, default=3)
    ap.add_argument("--duration", type=float, default=10.0,
                    help="Trial duration (s) used in test_terrain.py (default 10)")
    ap.add_argument("--csv", type=str, default=None,
                    help="Explicit path to trial_results.csv (overrides base/levels)")
    args = ap.parse_args()

    if args.csv:
        csv_path = Path(args.csv)
        level_str = csv_path.parent.name
    else:
        level_str = f"base{args.base:.1f}in_{args.levels}levels"
        csv_path = OUTPUT_DIR / level_str / "trial_results.csv"

    if not csv_path.exists():
        print(f"ERROR: {csv_path} not found. Run test_terrain.py first.")
        return

    rows = load_results(csv_path)
    if not rows:
        print("ERROR: no OK trials in CSV.")
        return

    stats = aggregate(rows, duration=args.duration)

    print("=" * 70)
    print(f"Loaded {sum(s['n_trials'] for s in stats.values())} trials from {csv_path}")
    print("=" * 70)
    print(f"{'Seg':>4} {'Vel (m/s)':>14} {'Pitch (°)':>14} {'Roll (°)':>14} {'Score':>14}")
    print("-" * 70)
    for n, s in stats.items():
        v = s["velocity"]; p = s["pitch"]; r = s["roll"]; sc = s["score"]
        print(f"{n:>4} "
              f"{v[0]:>7.3f}±{v[1]:<5.3f} "
              f"{p[0]:>7.2f}±{p[1]:<5.2f} "
              f"{r[0]:>7.2f}±{r[1]:<5.2f} "
              f"{sc[0]:>7.3f}±{sc[1]:<5.3f}")
    print("-" * 70)
    print("score = distance / (1 + (pitch_dev + roll_dev)/10)")
    print("=" * 70)

    # Save the 4 separate line charts into <run_dir>/figures/
    figures_dir = csv_path.parent / "figures"
    plot_all(stats, figures_dir, title_suffix=f"  ({level_str})")
    print(f"\n  Figures folder: {figures_dir}")


if __name__ == "__main__":
    main()
