from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    with args.trials.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))

    groups: dict[tuple[float, int], list[float]] = defaultdict(list)
    q_values = set()
    for row in rows:
        wave = float(row["N_wave_ref10"])
        n_segments = int(row["n_segments"])
        groups[(wave, n_segments)].append(float(row["forward_displacement_m"]))
        q_values.add(float(row["q_commanded"]))

    waves = sorted({key[0] for key in groups})
    segments = sorted({key[1] for key in groups})
    if q_values != {0.5}:
        raise RuntimeError(f"Expected q=0.5 only, found {sorted(q_values)}")
    if len(groups) != len(waves) * len(segments):
        raise RuntimeError("Incomplete wave-by-segment grid")
    if any(len(values) != 512 for values in groups.values()):
        raise RuntimeError("Every histogram must contain exactly 512 trials")

    args.output.mkdir(parents=True, exist_ok=True)
    audit = []
    for wave in waves:
        wave_values = {
            n_segments: np.asarray(groups[(wave, n_segments)], dtype=float)
            for n_segments in segments
        }
        pooled = np.concatenate(list(wave_values.values()))
        x_min = float(np.min(pooled))
        x_max = float(np.max(pooled))
        x_padding = max(0.02 * (x_max - x_min), 1e-6)
        common_edges = np.linspace(x_min - x_padding, x_max + x_padding, 25)
        common_y_max = max(
            int(np.histogram(values, bins=common_edges)[0].max())
            for values in wave_values.values()
        )

        fig, axes = plt.subplots(2, 3, figsize=(9.0, 5.6), sharex=True, sharey=True)
        for axis, n_segments in zip(axes.reshape(-1), segments):
            values = wave_values[n_segments]
            axis.hist(
                values,
                bins=common_edges,
                color="#237f8d",
                alpha=0.86,
                edgecolor="white",
                linewidth=0.35,
            )
            axis.axvline(statistics.fmean(values), color="#d95f02", linewidth=1.2)
            axis.set_xlim(common_edges[0], common_edges[-1])
            axis.set_ylim(0.0, 1.08 * common_y_max)
            axis.set_title(f"N={n_segments}", fontsize=10)
            axis.grid(axis="y", alpha=0.15)

        fig.supxlabel("Signed forward displacement (m)")
        fig.supylabel("Trial count")
        fig.suptitle(
            rf"$N_{{wave}}^{{(10)}}={wave:g}$, IID contact erasure ($q=0.5$)",
            fontsize=12,
        )
        fig.tight_layout()
        tag = f"{wave:.1f}".replace(".", "p")
        path = args.output / f"histograms_Nwave10_{tag}.png"
        fig.savefig(path, dpi=260)
        plt.close(fig)

        audit.append(
            {
                "N_wave_ref10": wave,
                "segments": segments,
                "trials_per_panel": 512,
                "bin_count": len(common_edges) - 1,
                "common_x_min_m": float(common_edges[0]),
                "common_x_max_m": float(common_edges[-1]),
                "common_y_max_count": common_y_max,
                "std_forward_displacement_m": {
                    str(n): statistics.stdev(wave_values[n]) for n in segments
                },
                "output": str(path),
            }
        )

    print(json.dumps({"passed": True, "figures": audit}, indent=2))


if __name__ == "__main__":
    main()
