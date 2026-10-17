"""Run a uniform-seed IID sweep on the center-rooted physical Centipede robot.

This is the production counterpart of the uniform-512 symmetric-null sweep.
Only the already audited center-root representation is sampled here; the
smaller matched head/center/tail experiment remains the root-bias audit.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import generate_center_rooted_physical_robot_models as generator  # noqa: E402
import run_center_rooted_physical_robot_iid as physical  # noqa: E402


SEGMENTS = tuple(range(4, 11))
VARIANT = generator.VARIANT


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _prepare_models(source_dir: Path, output_dir: Path, segments: tuple[int, ...]) -> list[dict]:
    destination_dir = output_dir / "models" / VARIANT
    destination_dir.mkdir(parents=True, exist_ok=True)
    manifest = []
    for n_segments in segments:
        name = f"centipede_{n_segments}seg_{VARIANT}.xml"
        source = source_dir / name
        destination = destination_dir / name
        if not source.is_file():
            raise FileNotFoundError(source)
        source_hash = _sha256(source)
        if destination.is_file() and _sha256(destination) != source_hash:
            raise RuntimeError(f"Existing model differs from audited source: {destination}")
        if not destination.is_file():
            shutil.copy2(source, destination)
        manifest.append(
            {
                "n_segments": n_segments,
                "source_model": str(source.resolve()),
                "experiment_model": str(destination.resolve()),
                "sha256": source_hash,
                "root_representation": "center_rooted",
                "fore_aft_geometry_symmetrized": False,
                "source_stl_and_inertials_preserved": True,
            }
        )
    return manifest


def _bootstrap_ci(values: np.ndarray, random: np.random.Generator) -> tuple[float, float]:
    samples = random.choice(values, size=(5000, values.size), replace=True)
    means = np.mean(samples, axis=1)
    return float(np.quantile(means, 0.025)), float(np.quantile(means, 0.975))


def _aggregate(trials: list[dict], segments: tuple[int, ...]) -> list[dict]:
    rows = []
    random = np.random.default_rng(260830)
    for n_segments in segments:
        selected_all = [row for row in trials if int(row["n_segments"]) == n_segments]
        selected = [row for row in selected_all if row["status"] == "PASS"]
        values = np.asarray([float(row["forward_displacement_m"]) for row in selected])
        if values.size == 0:
            continue
        low, high = _bootstrap_ci(values, random)
        rows.append(
            {
                "n_segments": n_segments,
                "n_trials": len(selected_all),
                "n_pass": len(selected),
                "mean_forward_displacement_m": float(np.mean(values)),
                "sd_forward_displacement_m": (
                    float(np.std(values, ddof=1)) if values.size > 1 else 0.0
                ),
                "bootstrap_ci95_low_m": low,
                "bootstrap_ci95_high_m": high,
                "median_forward_displacement_m": float(np.median(values)),
                "q05_forward_displacement_m": float(np.quantile(values, 0.05)),
                "q95_forward_displacement_m": float(np.quantile(values, 0.95)),
                "mean_absolute_displacement_m": float(np.mean(np.abs(values))),
                "positive_fraction": float(np.mean(values > 0.0)),
                "mean_actual_contact_fraction": float(
                    np.mean([float(row["actual_contact_fraction"]) for row in selected])
                ),
            }
        )
    return rows


def _compare_with_null(
    output_dir: Path,
    robot_trials: list[dict],
    null_dir: Path | None,
    segments: tuple[int, ...],
) -> tuple[list[dict], dict | None]:
    if null_dir is None or not (null_dir / "iid_trials.csv").is_file():
        return [], None
    with (null_dir / "iid_trials.csv").open(newline="", encoding="utf-8") as stream:
        null_trials = list(csv.DictReader(stream))

    rng = np.random.default_rng(261830)
    n_bootstrap = 5000
    rows = []
    robot_bootstrap_means = []
    null_bootstrap_means = []
    difference_bootstrap_means = []
    command_fields = (
        "commanded_down_fraction",
        "both_down_pair_fraction",
        "both_up_pair_fraction",
        "one_down_pair_fraction",
        "mean_abs_pairwise_state_correlation",
        "max_abs_pairwise_state_correlation",
    )
    for n_segments in segments:
        robot_by_seed = {
            int(row["seed"]): row
            for row in robot_trials
            if int(row["n_segments"]) == n_segments and row["status"] == "PASS"
        }
        null_by_seed = {
            int(row["seed"]): row
            for row in null_trials
            if int(row["n_segments"]) == n_segments
            and str(row["status"]).upper() == "PASS"
        }
        if set(robot_by_seed) != set(null_by_seed):
            raise RuntimeError(f"Robot/null seed sets differ for N={n_segments}")
        seeds = sorted(robot_by_seed)
        for seed in seeds:
            for field in command_fields:
                if not math.isclose(
                    float(robot_by_seed[seed][field]),
                    float(null_by_seed[seed][field]),
                    rel_tol=0.0,
                    abs_tol=1.0e-12,
                ):
                    raise RuntimeError(
                        f"Matched command audit failed for N={n_segments}, "
                        f"seed={seed}, field={field}"
                    )
        robot_values = np.asarray([
            float(robot_by_seed[seed]["forward_displacement_m"]) for seed in seeds
        ])
        null_values = np.asarray([
            float(null_by_seed[seed]["forward_displacement_m"]) for seed in seeds
        ])
        if robot_values.size != 512 or null_values.size != 512:
            raise RuntimeError(
                f"Expected 512 robot and null trials for N={n_segments}; "
                f"found {robot_values.size} and {null_values.size}"
            )
        bootstrap_indices = rng.integers(
            0, robot_values.size, size=(n_bootstrap, robot_values.size)
        )
        robot_boot = np.mean(robot_values[bootstrap_indices], axis=1)
        null_boot = np.mean(null_values[bootstrap_indices], axis=1)
        difference_values = robot_values - null_values
        difference_boot = np.mean(difference_values[bootstrap_indices], axis=1)
        robot_bootstrap_means.append(robot_boot)
        null_bootstrap_means.append(null_boot)
        difference_bootstrap_means.append(difference_boot)
        rows.append(
            {
                "n_segments": n_segments,
                "robot_trials": int(robot_values.size),
                "null_trials": int(null_values.size),
                "robot_mean_displacement_m": float(np.mean(robot_values)),
                "null_mean_displacement_m": float(np.mean(null_values)),
                "robot_minus_null_mean_m": float(np.mean(robot_values) - np.mean(null_values)),
                "difference_ci95_low_m": float(np.quantile(difference_boot, 0.025)),
                "difference_ci95_high_m": float(np.quantile(difference_boot, 0.975)),
                "robot_positive_fraction": float(np.mean(robot_values > 0.0)),
                "null_positive_fraction": float(np.mean(null_values > 0.0)),
            }
        )

    x = np.asarray(segments, dtype=float)
    centered_x = x - np.mean(x)
    denominator = float(np.sum(centered_x**2))
    robot_bootstrap_means = np.asarray(robot_bootstrap_means).T
    null_bootstrap_means = np.asarray(null_bootstrap_means).T
    difference_bootstrap_means = np.asarray(difference_bootstrap_means).T
    robot_slopes = robot_bootstrap_means @ centered_x / denominator
    null_slopes = null_bootstrap_means @ centered_x / denominator
    slope_difference = difference_bootstrap_means @ centered_x / denominator
    pooled_difference_bootstrap = np.mean(difference_bootstrap_means, axis=1)
    observed_robot_means = np.asarray([row["robot_mean_displacement_m"] for row in rows])
    observed_null_means = np.asarray([row["null_mean_displacement_m"] for row in rows])
    statistics = {
        "bootstrap_replicates": n_bootstrap,
        "equal_trial_count_per_N": 512,
        "robot_null_pairing": "matched by N and seed",
        "matched_command_fields": list(command_fields),
        "matched_command_audit_pass": True,
        "pooled_robot_mean_displacement_m": float(np.mean(observed_robot_means)),
        "pooled_null_mean_displacement_m": float(np.mean(observed_null_means)),
        "pooled_robot_minus_null_mean_m": float(
            np.mean(observed_robot_means - observed_null_means)
        ),
        "pooled_difference_ci95_low_m": float(
            np.quantile(pooled_difference_bootstrap, 0.025)
        ),
        "pooled_difference_ci95_high_m": float(
            np.quantile(pooled_difference_bootstrap, 0.975)
        ),
        "robot_mean_vs_N_slope_m_per_segment": float(
            np.sum(centered_x * observed_robot_means) / denominator
        ),
        "robot_slope_ci95_low_m_per_segment": float(np.quantile(robot_slopes, 0.025)),
        "robot_slope_ci95_high_m_per_segment": float(np.quantile(robot_slopes, 0.975)),
        "null_mean_vs_N_slope_m_per_segment": float(
            np.sum(centered_x * observed_null_means) / denominator
        ),
        "null_slope_ci95_low_m_per_segment": float(np.quantile(null_slopes, 0.025)),
        "null_slope_ci95_high_m_per_segment": float(np.quantile(null_slopes, 0.975)),
        "robot_minus_null_slope_m_per_segment": float(
            np.sum(centered_x * (observed_robot_means - observed_null_means)) / denominator
        ),
        "slope_difference_ci95_low_m_per_segment": float(
            np.quantile(slope_difference, 0.025)
        ),
        "slope_difference_ci95_high_m_per_segment": float(
            np.quantile(slope_difference, 0.975)
        ),
        "interpretation_guardrail": (
            "A positive robot-null offset supports retained physical fore-aft polarity; "
            "a segment-number slope is a separate claim and requires its CI to exclude zero."
        ),
    }
    physical._write_csv(output_dir / "robot_null_comparison.csv", rows)
    (output_dir / "comparison_statistics.json").write_text(
        json.dumps(statistics, indent=2), encoding="utf-8"
    )
    return rows, statistics


def _plots(
    output_dir: Path,
    summary: list[dict],
    trials: list[dict],
    null_dir: Path | None,
) -> None:
    import csv
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    ordered = sorted(summary, key=lambda row: row["n_segments"])
    x = np.asarray([row["n_segments"] for row in ordered])
    mean = np.asarray([row["mean_forward_displacement_m"] for row in ordered])
    low = np.asarray([row["bootstrap_ci95_low_m"] for row in ordered])
    high = np.asarray([row["bootstrap_ci95_high_m"] for row in ordered])

    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.errorbar(x, mean, yerr=np.vstack((mean - low, high - mean)), marker="o", capsize=3,
                linestyle="none", color="#b45f46")
    ax.axhline(0.0, color="#666666", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Mean signed displacement (m)")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_mean_signed_displacement_vs_N.png", dpi=240)
    plt.close(fig)

    fig, axes = plt.subplots(2, 4, figsize=(12.0, 6.3), sharex=True, sharey=True)
    all_values = np.asarray([float(row["forward_displacement_m"]) for row in trials])
    minimum = float(np.min(all_values))
    maximum = float(np.max(all_values))
    if math.isclose(minimum, maximum):
        padding = max(1.0e-6, 0.05 * max(abs(minimum), 1.0e-3))
        minimum -= padding
        maximum += padding
    bins = np.linspace(minimum, maximum, 15)
    maximum_density = 0.0
    histograms = []
    for axis, n_segments in zip(axes.flat, x.astype(int)):
        values = np.asarray([
            float(row["forward_displacement_m"])
            for row in trials
            if int(row["n_segments"]) == n_segments and row["status"] == "PASS"
        ])
        density, edges, patches = axis.hist(values, bins=bins, density=True, color="#b45f46",
                                             alpha=0.86, edgecolor="white")
        maximum_density = max(maximum_density, float(np.max(density)))
        histograms.append(axis)
        axis.axvline(0.0, color="#555555", linewidth=0.8)
        axis.set_title(f"N={n_segments} (n={values.size})")
        axis.grid(alpha=0.15)
    for axis in histograms:
        axis.set_ylim(0.0, 1.08 * maximum_density)
    axes.flat[-1].axis("off")
    fig.supxlabel("Signed displacement (m)")
    fig.supylabel("Probability density")
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_displacement_histograms.png", dpi=240)
    plt.close(fig)

    absolute_mean = np.asarray([row["mean_absolute_displacement_m"] for row in ordered])
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.plot(x, absolute_mean, "o-", color="#b45f46")
    ax.set_xticks(x)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Mean absolute displacement (m)")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_mean_absolute_displacement_vs_N.png", dpi=240)
    plt.close(fig)

    contact_fraction = np.asarray([row["mean_actual_contact_fraction"] for row in ordered])
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.plot(x, contact_fraction, "o-", color="#4f6f9f")
    ax.set_xticks(x)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Mean actual foot-contact fraction")
    ax.set_ylim(0.0, 1.0)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_mean_actual_contact_fraction_vs_N.png", dpi=240)
    plt.close(fig)

    fractions = np.asarray([row["positive_fraction"] for row in ordered])
    counts = np.asarray([row["n_pass"] for row in ordered], dtype=float)
    halfwidth = 1.96 * np.sqrt(fractions * (1.0 - fractions) / counts)
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.errorbar(x, fractions, yerr=halfwidth, marker="o", capsize=3, linestyle="none",
                color="#8b5a9b")
    ax.axhline(0.5, color="#666666", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_ylim(0.3, 0.8)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Fraction of positive trials")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_positive_fraction_vs_N.png", dpi=240)
    plt.close(fig)

    if null_dir is None:
        return
    null_summary_path = null_dir / "iid_summary.csv"
    null_trials_path = null_dir / "iid_trials.csv"
    if not null_summary_path.is_file() or not null_trials_path.is_file():
        return
    with null_summary_path.open(newline="", encoding="utf-8") as stream:
        null_summary = list(csv.DictReader(stream))
    selected_segments = set(x.astype(int).tolist())
    null_summary = [
        row for row in null_summary
        if int(row["n_segments"]) in selected_segments
    ]
    null_summary.sort(key=lambda row: int(row["n_segments"]))
    null_mean = np.asarray([float(row["mean_forward_displacement_m"]) for row in null_summary])
    null_low = np.asarray([float(row["bootstrap_ci95_low_m"]) for row in null_summary])
    null_high = np.asarray([float(row["bootstrap_ci95_high_m"]) for row in null_summary])
    fig, ax = plt.subplots(figsize=(7.4, 4.7))
    ax.errorbar(x - 0.05, null_mean, yerr=np.vstack((null_mean-null_low, null_high-null_mean)),
                marker="o", capsize=3, linestyle="none", color="#287a78", label="Symmetric null")
    ax.errorbar(x + 0.05, mean, yerr=np.vstack((mean-low, high-mean)), marker="o", capsize=3,
                linestyle="none", color="#b45f46", label="Physical robot")
    ax.axhline(0.0, color="#666666", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Mean signed displacement (m)")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "physical_robot_vs_symmetric_null_mean.png", dpi=240)
    plt.close(fig)

    comparison_path = output_dir / "robot_null_comparison.csv"
    if comparison_path.is_file():
        with comparison_path.open(newline="", encoding="utf-8") as stream:
            comparison_rows = list(csv.DictReader(stream))
        comparison_rows.sort(key=lambda row: int(row["n_segments"]))
        difference = np.asarray([
            float(row["robot_minus_null_mean_m"]) for row in comparison_rows
        ])
        difference_low = np.asarray([
            float(row["difference_ci95_low_m"]) for row in comparison_rows
        ])
        difference_high = np.asarray([
            float(row["difference_ci95_high_m"]) for row in comparison_rows
        ])
        fig, ax = plt.subplots(figsize=(7.4, 4.7))
        ax.errorbar(
            x, difference,
            yerr=np.vstack((difference - difference_low, difference_high - difference)),
            marker="o", capsize=3, linestyle="none", color="#7d4e91",
        )
        ax.axhline(0.0, color="#666666", linewidth=0.8)
        ax.set_xticks(x)
        ax.set_xlabel("Segment number, N")
        ax.set_ylabel("Matched mean difference: physical robot - null (m)")
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(figure_dir / "physical_robot_minus_symmetric_null_vs_N.png", dpi=240)
        plt.close(fig)


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    source_model_dir = args.source_model_dir.resolve()
    segments = tuple(sorted(set(args.segments)))
    output_dir.mkdir(parents=True, exist_ok=True)
    model_manifest = _prepare_models(source_model_dir, output_dir, segments)
    (output_dir / "model_manifest.json").write_text(
        json.dumps(model_manifest, indent=2), encoding="utf-8"
    )

    tasks = [
        (
            str(repo_root), str(output_dir), n_segments, seed, "center_rooted",
            args.warmup_cycles, args.evaluation_cycles, args.sample_hz, args.force,
        )
        for n_segments in segments
        for seed in range(args.replays)
    ]
    trials = []
    if args.workers == 1:
        for completed, task in enumerate(tasks, start=1):
            trials.append(physical._worker(task))
            if completed % 25 == 0 or completed == len(tasks):
                print(f"completed {completed}/{len(tasks)} physical-center trials", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {executor.submit(physical._worker, task): task for task in tasks}
            for completed, future in enumerate(as_completed(futures), start=1):
                trials.append(future.result())
                if completed % 25 == 0 or completed == len(tasks):
                    print(f"completed {completed}/{len(tasks)} physical-center trials", flush=True)

    trials.sort(key=lambda row: (int(row["n_segments"]), int(row["seed"])))
    summary = _aggregate(trials, segments)
    physical._write_csv(output_dir / "iid_trials.csv", trials)
    physical._write_csv(output_dir / "iid_summary.csv", summary)
    null_dir = args.null_dir.resolve() if args.null_dir else None
    _compare_with_null(output_dir, trials, null_dir, segments)
    x = np.asarray([row["n_segments"] for row in summary], dtype=float)
    y = np.asarray([row["mean_forward_displacement_m"] for row in summary])
    seed_audit = {
        str(n_segments): {
            "count": len([row for row in trials if int(row["n_segments"]) == n_segments]),
            "unique_seed_count": len({int(row["seed"]) for row in trials if int(row["n_segments"]) == n_segments}),
            "missing_seeds": sorted(set(range(args.replays)) - {int(row["seed"]) for row in trials if int(row["n_segments"]) == n_segments}),
        }
        for n_segments in segments
    }
    validation = {
        "all_trials_pass": all(row["status"] == "PASS" for row in trials),
        "ordinary_trial_count": len(trials),
        "ordinary_trials_by_N": {str(n): seed_audit[str(n)]["count"] for n in segments},
        "seed_audit": seed_audit,
        "center_rooted": all(bool(row["model_variant"] == VARIANT) for row in trials),
        "strict_independent_rng": all(row["rng_cross_n_mode"] == "strict_independent" for row in trials),
        "source_stl_and_inertials_preserved": all(bool(row["source_stl_and_inertials_preserved"]) for row in trials),
        "geometric_fore_aft_symmetry_imposed": False,
        "maximum_warning_count": max(int(row["mujoco_warning_total"]) for row in trials),
        "signed_mean_slope_m_per_segment": (
            float(np.polyfit(x, y, 1)[0]) if x.size > 1 else None
        ),
        "maximum_absolute_ensemble_mean_m": float(np.max(np.abs(y))),
        "pooled_mean_signed_displacement_m": float(np.mean([float(row["forward_displacement_m"]) for row in trials])),
    }
    (output_dir / "validation.json").write_text(
        json.dumps(validation, indent=2), encoding="utf-8"
    )
    configuration = {
        "segments": list(segments),
        "ordinary_trials_requested_per_N": args.replays,
        "warmup_cycles": args.warmup_cycles,
        "evaluation_cycles": args.evaluation_cycles,
        "sample_hz": args.sample_hz,
        "body_yaw_half_amplitude_deg": 15.0,
        "spatial_phase_increment_rad": -math.pi,
        "leg_yaw_locked": True,
        "pitch_process": "IID Bernoulli(0.5) per physical leg per half-cycle",
        "initialization": "straight all-down body-wave-off settle, smooth bridge",
        "model": "center-rooted physical robot; measured STL and inertials preserved",
        "head_tail_audit": "Use step26_center_rooted_physical_robot; not repeated in this production sweep",
    }
    (output_dir / "run_configuration.json").write_text(
        json.dumps(configuration, indent=2), encoding="utf-8"
    )
    _plots(output_dir, summary, trials, null_dir)


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--source-model-dir", type=Path,
        default=repo_root / "exp_data" / "reliability" / "step26_center_rooted_physical_robot" / "models" / VARIANT,
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=repo_root / "exp_data" / "reliability" / "step26b_center_rooted_physical_robot_uniform512",
    )
    parser.add_argument(
        "--null-dir", type=Path,
        default=repo_root / "exp_data" / "reliability" / "step25b_center_rooted_dynamic_null_uniform512",
    )
    parser.add_argument("--segments", type=int, nargs="+", default=list(SEGMENTS))
    parser.add_argument("--replays", type=int, default=512)
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--sample-hz", type=float, default=100.0)
    parser.add_argument("--workers", type=int, default=max(1, min(6, (os.cpu_count() or 2) // 2)))
    parser.add_argument("--force", action="store_true")
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
