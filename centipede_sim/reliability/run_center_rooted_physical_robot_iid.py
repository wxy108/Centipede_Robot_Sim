"""Run matched head/center/tail-root IID trials on the measured STL robot."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import generate_center_rooted_physical_robot_models as generator  # noqa: E402
import run_center_rooted_dynamic_null_iid as center  # noqa: E402
import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402


VARIANTS = (
    "head_endpoint",
    "center_rooted",
    "tail_rooted",
)
MODEL_VARIANT = {
    "head_endpoint": generator.REFERENCE_VARIANT,
    "center_rooted": generator.VARIANT,
    "tail_rooted": generator.TAIL_VARIANT,
}


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _model_path(output_dir: Path, n_segments: int, variant: str) -> Path:
    model_variant = MODEL_VARIANT[variant]
    return (
        output_dir
        / "models"
        / model_variant
        / f"centipede_{n_segments}seg_{model_variant}.xml"
    )


def _record_path(output_dir: Path, n_segments: int, seed: int, variant: str) -> Path:
    return output_dir / "trial_records" / variant / f"N{n_segments}_seed{seed:03d}.json"


def _run_one(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    seed: int,
    variant: str,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
    force: bool,
) -> dict:
    record_path = _record_path(output_dir, n_segments, seed, variant)
    if record_path.is_file() and not force:
        return json.loads(record_path.read_text(encoding="utf-8"))
    model_path = _model_path(output_dir, n_segments, variant)
    stability_body_name = None if variant == "head_endpoint" else "Chasis1"
    with center._center_trial_context(
        model_path,
        complement=False,
        stability_body_name=stability_body_name,
    ):
        result = base._run_trial(
            repo_root,
            n_segments=n_segments,
            seed=seed,
            mode=base.RANDOM_MODE,
            warmup_cycles=warmup_cycles,
            evaluation_cycles=evaluation_cycles,
            sample_hz=sample_hz,
            trajectory_dir=output_dir / "trajectories" / variant,
        )
    result.update(
        {
            "variant": variant,
            "model_variant": MODEL_VARIANT[variant],
            "model_path": str(model_path),
            "source_stl_and_inertials_preserved": True,
            "geometric_fore_aft_symmetry_imposed": False,
            "explicit_center_out_contact_pairs": True,
            "symmetric_initialization": True,
            "rng_cross_n_mode": "strict_independent",
            "spatial_phase_increment_rad": -math.pi,
            "leg_yaw_locked": True,
            "stability_reference_body": stability_body_name or "freejoint_root",
        }
    )
    record_path.parent.mkdir(parents=True, exist_ok=True)
    record_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def _worker(task: tuple) -> dict:
    (
        repo_root_text,
        output_dir_text,
        n_segments,
        seed,
        variant,
        warmup_cycles,
        evaluation_cycles,
        sample_hz,
        force,
    ) = task
    return _run_one(
        Path(repo_root_text),
        Path(output_dir_text),
        n_segments=n_segments,
        seed=seed,
        variant=variant,
        warmup_cycles=warmup_cycles,
        evaluation_cycles=evaluation_cycles,
        sample_hz=sample_hz,
        force=force,
    )


def _bootstrap_ci(values: np.ndarray, random: np.random.Generator) -> tuple[float, float]:
    means = np.asarray(
        [
            np.mean(random.choice(values, size=values.size, replace=True))
            for _ in range(5000)
        ]
    )
    return float(np.quantile(means, 0.025)), float(np.quantile(means, 0.975))


def _aggregate(trials: list[dict], segments: tuple[int, ...]) -> list[dict]:
    rows = []
    random = np.random.default_rng(260827)
    for variant in VARIANTS:
        for n_segments in segments:
            selected_all = [
                row
                for row in trials
                if row["variant"] == variant and int(row["n_segments"]) == n_segments
            ]
            selected = [row for row in selected_all if row["status"] == "PASS"]
            values = np.asarray(
                [float(row["forward_displacement_m"]) for row in selected], dtype=float
            )
            if values.size == 0:
                continue
            low, high = _bootstrap_ci(values, random)
            rows.append(
                {
                    "variant": variant,
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
                    "mean_absolute_displacement_m": float(np.mean(np.abs(values))),
                    "positive_fraction": float(np.mean(values > 0.0)),
                    "mean_actual_contact_fraction": float(
                        np.mean([float(row["actual_contact_fraction"]) for row in selected])
                    ),
                }
            )
    return rows


def _paired_rows(trials: list[dict], segments: tuple[int, ...]) -> list[dict]:
    lookup = {
        (row["variant"], int(row["n_segments"]), int(row["seed"])): row
        for row in trials
    }
    rows = []
    for n_segments in segments:
        seeds = sorted(
            {
                int(row["seed"])
                for row in trials
                if int(row["n_segments"]) == n_segments
            }
        )
        for seed in seeds:
            keys = [(variant, n_segments, seed) for variant in VARIANTS]
            if not all(key in lookup for key in keys):
                continue
            head = lookup[("head_endpoint", n_segments, seed)]
            middle = lookup[("center_rooted", n_segments, seed)]
            tail = lookup[("tail_rooted", n_segments, seed)]
            d_head = float(head["forward_displacement_m"])
            d_center = float(middle["forward_displacement_m"])
            d_tail = float(tail["forward_displacement_m"])
            rows.append(
                {
                    "n_segments": n_segments,
                    "seed": seed,
                    "head_displacement_m": d_head,
                    "center_displacement_m": d_center,
                    "tail_displacement_m": d_tail,
                    "head_minus_tail_m": d_head - d_tail,
                    "center_minus_endpoint_average_m": d_center - 0.5 * (d_head + d_tail),
                    "root_spread_m": max(d_head, d_center, d_tail) - min(d_head, d_center, d_tail),
                    "all_pass": all(row["status"] == "PASS" for row in (head, middle, tail)),
                }
            )
    return rows


def _paired_summary(rows: list[dict], segments: tuple[int, ...]) -> list[dict]:
    result = []
    random = np.random.default_rng(261827)
    for n_segments in segments:
        selected = [row for row in rows if row["n_segments"] == n_segments]
        head_tail = np.asarray([row["head_minus_tail_m"] for row in selected])
        center_residual = np.asarray(
            [row["center_minus_endpoint_average_m"] for row in selected]
        )
        endpoint_average = np.asarray(
            [
                0.5 * (row["head_displacement_m"] + row["tail_displacement_m"])
                for row in selected
            ]
        )
        root_bias = 0.5 * head_tail
        spread = np.asarray([row["root_spread_m"] for row in selected])
        ht_low, ht_high = _bootstrap_ci(head_tail, random)
        cr_low, cr_high = _bootstrap_ci(center_residual, random)
        ea_low, ea_high = _bootstrap_ci(endpoint_average, random)
        rb_low, rb_high = _bootstrap_ci(root_bias, random)
        result.append(
            {
                "n_segments": n_segments,
                "n_paired_trials": len(selected),
                "mean_head_minus_tail_m": float(np.mean(head_tail)),
                "head_minus_tail_ci95_low_m": ht_low,
                "head_minus_tail_ci95_high_m": ht_high,
                "mean_center_minus_endpoint_average_m": float(np.mean(center_residual)),
                "center_residual_ci95_low_m": cr_low,
                "center_residual_ci95_high_m": cr_high,
                "mean_debiased_endpoint_average_m": float(np.mean(endpoint_average)),
                "debiased_endpoint_average_ci95_low_m": ea_low,
                "debiased_endpoint_average_ci95_high_m": ea_high,
                "mean_root_bias_amplitude_m": float(np.mean(root_bias)),
                "root_bias_amplitude_ci95_low_m": rb_low,
                "root_bias_amplitude_ci95_high_m": rb_high,
                "mean_root_spread_m": float(np.mean(spread)),
                "max_root_spread_m": float(np.max(spread)),
            }
        )
    return result


def _plots(
    output_dir: Path,
    summary: list[dict],
    paired_summary: list[dict],
    trials: list[dict],
    segments: tuple[int, ...],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    colors = {
        "head_endpoint": "#ba5a45",
        "center_rooted": "#287a78",
        "tail_rooted": "#5967a9",
    }
    labels = {
        "head_endpoint": "Head root",
        "center_rooted": "Center root",
        "tail_rooted": "Tail root",
    }
    fig, ax = plt.subplots(figsize=(7.4, 4.7))
    for variant in VARIANTS:
        selected = sorted(
            [row for row in summary if row["variant"] == variant],
            key=lambda row: row["n_segments"],
        )
        x = np.asarray([row["n_segments"] for row in selected])
        y = np.asarray([row["mean_forward_displacement_m"] for row in selected])
        low = np.asarray([row["bootstrap_ci95_low_m"] for row in selected])
        high = np.asarray([row["bootstrap_ci95_high_m"] for row in selected])
        ax.errorbar(
            x,
            y,
            yerr=np.vstack((y - low, high - y)),
            marker="o",
            capsize=3,
            linestyle="none",
            color=colors[variant],
            label=labels[variant],
        )
    ax.axhline(0.0, color="#666666", linewidth=0.8)
    ax.set_xticks(list(segments))
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Signed displacement (m)")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "root_location_mean_displacement_vs_N.png", dpi=240)
    plt.close(fig)

    ordered = sorted(paired_summary, key=lambda row: row["n_segments"])
    x = np.asarray([row["n_segments"] for row in ordered])
    y = np.asarray([row["mean_center_minus_endpoint_average_m"] for row in ordered])
    low = np.asarray([row["center_residual_ci95_low_m"] for row in ordered])
    high = np.asarray([row["center_residual_ci95_high_m"] for row in ordered])
    fig, ax = plt.subplots(figsize=(7.4, 4.7))
    ax.errorbar(
        x,
        y,
        yerr=np.vstack((y - low, high - y)),
        marker="o",
        capsize=3,
        linestyle="none",
        color="#287a78",
    )
    ax.axhline(0.0, color="#666666", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Center minus endpoint average (m)")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "center_root_representation_residual_vs_N.png", dpi=240)
    plt.close(fig)

    endpoint_mean = np.asarray(
        [row["mean_debiased_endpoint_average_m"] for row in ordered]
    )
    endpoint_low = np.asarray(
        [row["debiased_endpoint_average_ci95_low_m"] for row in ordered]
    )
    endpoint_high = np.asarray(
        [row["debiased_endpoint_average_ci95_high_m"] for row in ordered]
    )
    center_rows = sorted(
        [row for row in summary if row["variant"] == "center_rooted"],
        key=lambda row: row["n_segments"],
    )
    center_mean = np.asarray(
        [row["mean_forward_displacement_m"] for row in center_rows]
    )
    center_low = np.asarray([row["bootstrap_ci95_low_m"] for row in center_rows])
    center_high = np.asarray([row["bootstrap_ci95_high_m"] for row in center_rows])
    fig, ax = plt.subplots(figsize=(7.4, 4.7))
    ax.errorbar(
        x - 0.05,
        endpoint_mean,
        yerr=np.vstack((endpoint_mean - endpoint_low, endpoint_high - endpoint_mean)),
        marker="o",
        capsize=3,
        linestyle="none",
        color="#6b6b6b",
        label="Endpoint-average debiased",
    )
    ax.errorbar(
        x + 0.05,
        center_mean,
        yerr=np.vstack((center_mean - center_low, center_high - center_mean)),
        marker="o",
        capsize=3,
        linestyle="none",
        color="#287a78",
        label="Center root",
    )
    ax.axhline(0.0, color="#666666", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Root-debiased signed displacement (m)")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "debiased_endpoint_average_vs_center.png", dpi=240)
    plt.close(fig)

    bias_mean = np.asarray([row["mean_root_bias_amplitude_m"] for row in ordered])
    bias_low = np.asarray([row["root_bias_amplitude_ci95_low_m"] for row in ordered])
    bias_high = np.asarray([row["root_bias_amplitude_ci95_high_m"] for row in ordered])
    fig, ax = plt.subplots(figsize=(7.4, 4.7))
    ax.errorbar(
        x,
        bias_mean,
        yerr=np.vstack((bias_mean - bias_low, bias_high - bias_mean)),
        marker="o",
        capsize=3,
        linestyle="none",
        color="#a66a3f",
    )
    ax.axhline(0.0, color="#666666", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Root-bias amplitude (m)")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "root_bias_amplitude_vs_N.png", dpi=240)
    plt.close(fig)

    fig, axes = plt.subplots(2, 4, figsize=(12.0, 6.3), sharex=True, sharey=True)
    for axis, n_segments in zip(axes.flat, segments):
        values = [
            float(row["forward_displacement_m"])
            for row in trials
            if row["variant"] == "center_rooted"
            and int(row["n_segments"]) == n_segments
            and row["status"] == "PASS"
        ]
        axis.hist(values, bins=12, density=True, color="#4b918d", alpha=0.88, edgecolor="white")
        axis.axvline(0.0, color="#555555", linewidth=0.8)
        axis.set_title(f"N={n_segments}")
        axis.grid(alpha=0.15)
    axes.flat[-1].axis("off")
    fig.supxlabel("Center-root signed displacement (m)")
    fig.supylabel("Probability density")
    fig.tight_layout()
    fig.savefig(figure_dir / "center_root_iid_displacement_histograms.png", dpi=240)
    plt.close(fig)


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    segments = tuple(sorted(set(args.segments)))
    tasks = [
        (
            str(repo_root),
            str(output_dir),
            n_segments,
            seed,
            variant,
            args.warmup_cycles,
            args.evaluation_cycles,
            args.sample_hz,
            args.force,
        )
        for n_segments in segments
        for seed in range(args.replays)
        for variant in VARIANTS
    ]
    trials = []
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures = {executor.submit(_worker, task): task for task in tasks}
        for completed, future in enumerate(as_completed(futures), start=1):
            trials.append(future.result())
            if completed % 20 == 0 or completed == len(tasks):
                print(f"completed {completed}/{len(tasks)} physical-root trials", flush=True)

    trials.sort(key=lambda row: (row["variant"], row["n_segments"], row["seed"]))
    summary = _aggregate(trials, segments)
    paired = _paired_rows(trials, segments)
    paired_summary = _paired_summary(paired, segments)
    _write_csv(output_dir / "iid_root_trials.csv", trials)
    _write_csv(output_dir / "iid_root_summary.csv", summary)
    _write_csv(output_dir / "root_paired_trials.csv", paired)
    _write_csv(output_dir / "root_paired_summary.csv", paired_summary)
    validation = {
        "all_trials_pass": all(row["status"] == "PASS" for row in trials),
        "trial_count": len(trials),
        "trials_per_root_per_N": args.replays,
        "matched_seed_design": True,
        "source_stl_and_inertials_preserved": True,
        "geometric_fore_aft_symmetry_imposed": False,
        "maximum_warning_count": max(int(row["mujoco_warning_total"]) for row in trials),
        "maximum_mean_root_spread_m": max(row["mean_root_spread_m"] for row in paired_summary),
        "maximum_absolute_center_residual_mean_m": max(
            abs(row["mean_center_minus_endpoint_average_m"]) for row in paired_summary
        ),
    }
    (output_dir / "iid_root_validation.json").write_text(
        json.dumps(validation, indent=2), encoding="utf-8"
    )
    (output_dir / "iid_root_run_configuration.json").write_text(
        json.dumps(
            {
                "segments": list(segments),
                "variants": list(VARIANTS),
                "trials_per_root_per_N": args.replays,
                "warmup_cycles": args.warmup_cycles,
                "evaluation_cycles": args.evaluation_cycles,
                "sample_hz": args.sample_hz,
                "body_yaw_half_amplitude_deg": 15.0,
                "spatial_phase_increment_rad": -math.pi,
                "leg_yaw_locked": True,
                "pitch_process": "IID Bernoulli(0.5) per physical leg per half-cycle",
                "initialization": "straight all-down body-wave-off settle, smooth bridge",
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    _plots(output_dir, summary, paired_summary, trials, segments)


def build_parser() -> argparse.ArgumentParser:
    repo_root = generator._repo_root_from_script()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            repo_root
            / "exp_data"
            / "reliability"
            / "step26_center_rooted_physical_robot"
        ),
    )
    parser.add_argument("--segments", type=int, nargs="+", default=list(generator.SEGMENTS))
    parser.add_argument("--replays", type=int, default=32)
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--sample-hz", type=float, default=100.0)
    parser.add_argument("--workers", type=int, default=max(1, min(6, (os.cpu_count() or 2) // 2)))
    parser.add_argument("--force", action="store_true")
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
