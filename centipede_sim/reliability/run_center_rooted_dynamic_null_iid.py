"""Run the IID pitch experiment on the center-rooted dynamic null.

Unlike earlier runners, every trial starts from a reflection-symmetric,
straight, all-pitch-down, body-wave-off state.  This prevents the canonical
gait initialization from injecting a signed momentum before the IID process.
An ordinary IID ensemble is reported separately from a smaller set of exact
bitwise-complement pairs used only as a symmetry diagnostic.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import contextmanager
from pathlib import Path

import numpy as np


if os.environ.get("CENTIPEDE_SKIP_OPTIONAL_MUJOCO_PLUGINS") == "1":
    # The center-rooted null models contain no plugin elements.  On managed
    # Windows hosts, Application Control may nevertheless block MuJoCo's eager
    # loading of its optional bundled plugin DLLs.  Keep this opt-in so the
    # default MuJoCo import behavior remains unchanged for all other models.
    import ctypes

    _original_cdll = ctypes.CDLL

    def _cdll_without_optional_mujoco_plugins(name, *args, **kwargs):
        normalized = os.path.normcase(os.fspath(name))
        plugin_marker = os.path.normcase(f"{os.sep}mujoco{os.sep}plugin{os.sep}")
        if plugin_marker in normalized and normalized.endswith(".dll"):
            return object()
        return _original_cdll(name, *args, **kwargs)

    ctypes.CDLL = _cdll_without_optional_mujoco_plugins


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import run_body_wave_only_random_pitch as iid  # noqa: E402
import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    BRIDGE_TIME_S,
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    OMEGA_BODY,
    SETTLE_TIME_S,
    _leg_body_map,
)


SEGMENTS = tuple(range(4, 11))
VARIANT = "center_rooted_dynamic_null"
MODE = iid.MODE_RANDOM


class ComplementableIIDController(iid.BodyWaveOnlyIIDController):
    def __init__(self, *args, complement: bool, **kwargs):
        super().__init__(*args, **kwargs)
        self.complement = bool(complement)

    def stance_flags(self, decision_time_s: float) -> np.ndarray:
        state = super().stance_flags(decision_time_s)
        return np.logical_not(state) if self.complement else state


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


def _model_path(output_dir: Path, n_segments: int) -> Path:
    return (
        output_dir
        / "models"
        / VARIANT
        / f"centipede_{n_segments}seg_{VARIANT}.xml"
    )


def _symmetric_initial_target(n_ctrl: int, indices) -> np.ndarray:
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = indices
    ctrl = np.zeros(n_ctrl, dtype=float)
    ctrl[shoulder_r] = math.radians(LEG_OFFSET_DEG)
    ctrl[shoulder_l] = -math.radians(LEG_OFFSET_DEG)
    ctrl[leg_r] = math.radians(LEG_DOWN_DEG)
    ctrl[leg_l] = math.radians(LEG_DOWN_DEG)
    ctrl[link_y] = 0.0
    ctrl[link_p] = 0.0
    return ctrl


def _prepare_center_simulation(
    repo_root: Path,
    model_path: Path,
    *,
    n_segments: int,
    seed: int,
    warmup_cycles: int,
    complement: bool,
    stability_body_name: str | None = None,
) -> base.PreparedSimulation:
    import mujoco

    build_actuator_indices, canonical_compute_ctrl = base._install_project_imports(
        repo_root
    )
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    expected_ctrl = n_segments * 4 + (n_segments - 1) * 2
    if model.nu != expected_ctrl:
        raise RuntimeError(
            f"{model_path.name}: expected {expected_ctrl} actuators, found {model.nu}"
        )
    indices = build_actuator_indices(n_segments)
    controller = ComplementableIIDController(
        n_segments,
        model.nu,
        indices,
        canonical_compute_ctrl,
        seed=seed,
        stance_probability=0.5,
        rng_cross_n_mode="strict_independent",
        complement=complement,
    )
    root_body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
    )
    if root_body_id < 0:
        raise KeyError("robot_root body is missing")
    stability_body_id = None
    if stability_body_name is not None:
        stability_body_id = int(
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, stability_body_name
            )
        )
        if stability_body_id < 0:
            raise KeyError(stability_body_name)
    body_map = _leg_body_map(model, n_segments, mujoco)
    dt = float(model.opt.timestep)
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    warmup_steps = int(math.ceil(warmup_cycles * gait_period_s / dt))
    warmup_time_s = warmup_steps * dt

    initial_target = _symmetric_initial_target(model.nu, indices)
    settle_steps = int(math.ceil(SETTLE_TIME_S / dt))
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial_target
        mujoco.mj_step(model, data)

    bridge_target = controller.control(0.0, -warmup_time_s)
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / dt))
    for step in range(bridge_steps):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial_target + alpha * bridge_target
        mujoco.mj_step(model, data)

    for step in range(warmup_steps):
        body_time_s = step * dt
        decision_time_s = body_time_s - warmup_time_s
        data.ctrl[:] = controller.control(body_time_s, decision_time_s)
        mujoco.mj_step(model, data)

    return base.PreparedSimulation(
        model=model,
        data=data,
        controller=controller,
        root_body_id=root_body_id,
        body_map=body_map,
        dt=dt,
        warmup_time_s=warmup_time_s,
        stability_body_id=stability_body_id,
    )


@contextmanager
def _center_trial_context(
    model_path: Path,
    *,
    complement: bool,
    stability_body_name: str | None = None,
):
    original_prepare = base._prepare_simulation
    original_collision_check = base._model_collision_check

    def prepare(repo_root, *, n_segments, seed, mode, warmup_cycles):
        if mode != base.RANDOM_MODE:
            raise ValueError(mode)
        return _prepare_center_simulation(
            repo_root,
            model_path,
            n_segments=n_segments,
            seed=seed,
            warmup_cycles=warmup_cycles,
            complement=complement,
            stability_body_name=stability_body_name,
        )

    base._prepare_simulation = prepare
    # Ground collision is represented by explicit contact pairs; the legacy
    # bit-mask-only gate cannot detect them.
    base._model_collision_check = lambda *_args: True
    try:
        yield
    finally:
        base._prepare_simulation = original_prepare
        base._model_collision_check = original_collision_check


def _record_path(
    output_dir: Path,
    n_segments: int,
    seed: int,
    complement: bool,
) -> Path:
    family = "complement" if complement else "ordinary"
    return output_dir / "trial_records" / family / f"N{n_segments}_seed{seed:03d}.json"


def _run_one(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    seed: int,
    complement: bool,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
    force: bool,
) -> dict:
    record_path = _record_path(output_dir, n_segments, seed, complement)
    if record_path.is_file() and not force:
        return json.loads(record_path.read_text(encoding="utf-8"))
    model_path = _model_path(output_dir, n_segments)
    family = "complement" if complement else "ordinary"
    trajectory_dir = output_dir / "trajectories" / family
    with _center_trial_context(model_path, complement=complement):
        result = base._run_trial(
            repo_root,
            n_segments=n_segments,
            seed=seed,
            mode=base.RANDOM_MODE,
            warmup_cycles=warmup_cycles,
            evaluation_cycles=evaluation_cycles,
            sample_hz=sample_hz,
            trajectory_dir=trajectory_dir,
        )
    result.update(
        {
            "variant": VARIANT,
            "model_path": str(model_path),
            "center_rooted": True,
            "symmetric_initialization": True,
            "explicit_mirror_contact_pairs": True,
            "complemented_iid_bits": complement,
            "rng_cross_n_mode": "strict_independent",
            "spatial_phase_increment_rad": -math.pi,
            "leg_yaw_locked": True,
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
        complement,
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
        complement=complement,
        warmup_cycles=warmup_cycles,
        evaluation_cycles=evaluation_cycles,
        sample_hz=sample_hz,
        force=force,
    )


def _aggregate(trials: list[dict], segments: tuple[int, ...]) -> list[dict]:
    rows = []
    bootstrap_rng = np.random.default_rng(250827)
    for n_segments in segments:
        selected_all = [
            row
            for row in trials
            if int(row["n_segments"]) == n_segments
            and not bool(row["complemented_iid_bits"])
        ]
        selected = [row for row in selected_all if row["status"] == "PASS"]
        values = np.asarray(
            [float(row["forward_displacement_m"]) for row in selected], dtype=float
        )
        if values.size == 0:
            continue
        bootstrap_means = np.asarray(
            [
                np.mean(bootstrap_rng.choice(values, size=values.size, replace=True))
                for _ in range(5000)
            ]
        )
        rows.append(
            {
                "n_segments": n_segments,
                "n_trials": len(selected_all),
                "n_pass": len(selected),
                "mean_forward_displacement_m": float(np.mean(values)),
                "sd_forward_displacement_m": float(np.std(values, ddof=1)),
                "bootstrap_ci95_low_m": float(np.quantile(bootstrap_means, 0.025)),
                "bootstrap_ci95_high_m": float(np.quantile(bootstrap_means, 0.975)),
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


def _complement_pairs(
    trials: list[dict],
    segments: tuple[int, ...],
    paired_replays: int,
) -> list[dict]:
    lookup = {
        (
            int(row["n_segments"]),
            int(row["seed"]),
            bool(row["complemented_iid_bits"]),
        ): row
        for row in trials
    }
    rows = []
    for n_segments in segments:
        for seed in range(paired_replays):
            ordinary = lookup[(n_segments, seed, False)]
            complement = lookup[(n_segments, seed, True)]
            d0 = float(ordinary["forward_displacement_m"])
            d1 = float(complement["forward_displacement_m"])
            rows.append(
                {
                    "n_segments": n_segments,
                    "seed": seed,
                    "ordinary_displacement_m": d0,
                    "complement_displacement_m": d1,
                    "pair_bias_m": 0.5 * (d0 + d1),
                    "pair_antisymmetric_component_m": 0.5 * (d0 - d1),
                    "both_pass": ordinary["status"] == complement["status"] == "PASS",
                }
            )
    return rows


def _trend(summary: list[dict]) -> dict:
    x = np.asarray([row["n_segments"] for row in summary], dtype=float)
    y = np.asarray([row["mean_forward_displacement_m"] for row in summary])
    return {
        "signed_mean_slope_m_per_segment": float(np.polyfit(x, y, 1)[0]),
        "maximum_absolute_ensemble_mean_m": float(np.max(np.abs(y))),
    }


def _plots(
    output_dir: Path,
    summary: list[dict],
    ordinary_trials: list[dict],
    complement_pairs: list[dict],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    ordered = sorted(summary, key=lambda row: row["n_segments"])

    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    x = np.asarray([row["n_segments"] for row in ordered])
    y = np.asarray([row["mean_forward_displacement_m"] for row in ordered])
    low = np.asarray([row["bootstrap_ci95_low_m"] for row in ordered])
    high = np.asarray([row["bootstrap_ci95_high_m"] for row in ordered])
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
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Signed displacement (m)")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_mean_signed_displacement_vs_N.png", dpi=240)
    plt.close(fig)

    fig, axes = plt.subplots(2, 4, figsize=(12.0, 6.3), sharex=True, sharey=True)
    for axis, n_segments in zip(axes.flat, range(4, 11)):
        values = [
            float(row["forward_displacement_m"])
            for row in ordinary_trials
            if int(row["n_segments"]) == n_segments and row["status"] == "PASS"
        ]
        axis.hist(
            values,
            bins=14,
            density=True,
            color="#4b918d",
            alpha=0.88,
            edgecolor="white",
        )
        axis.axvline(0.0, color="#555555", linewidth=0.8)
        axis.set_title(f"N={n_segments}")
        axis.grid(alpha=0.15)
    axes.flat[-1].axis("off")
    fig.supxlabel("Signed displacement (m)")
    fig.supylabel("Probability density")
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_displacement_histograms.png", dpi=240)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    pair_by_n = []
    for n_segments in range(4, 11):
        values = [
            float(row["pair_bias_m"])
            for row in complement_pairs
            if int(row["n_segments"]) == n_segments
        ]
        pair_by_n.append(values)
    ax.boxplot(pair_by_n, tick_labels=[str(value) for value in range(4, 11)], showfliers=False)
    ax.axhline(0.0, color="#666666", linewidth=0.8)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Complement-pair bias (m)")
    ax.grid(axis="y", alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "complement_pair_bias.png", dpi=240)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    fractions = np.asarray([row["positive_fraction"] for row in ordered])
    counts = np.asarray([row["n_pass"] for row in ordered], dtype=float)
    binomial_halfwidth = 1.96 * np.sqrt(0.25 / counts)
    ax.errorbar(
        x,
        fractions,
        yerr=binomial_halfwidth,
        marker="o",
        capsize=3,
        linestyle="none",
        color="#8b5a9b",
    )
    ax.axhline(0.5, color="#666666", linewidth=0.8)
    ax.set_xlabel("Segment number, N")
    ax.set_ylabel("Fraction of positive trials")
    ax.set_xticks(x)
    ax.set_ylim(0.3, 0.7)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_positive_fraction_vs_N.png", dpi=240)
    plt.close(fig)


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    segments = tuple(sorted(set(args.segments)))
    output_dir.mkdir(parents=True, exist_ok=True)
    iid.ACTIVE_SPATIAL_PHASE_INCREMENT_RAD = -math.pi
    iid.ACTIVE_RNG_CROSS_N_MODE = "strict_independent"

    tasks = []
    for n_segments in segments:
        for seed in range(args.replays):
            tasks.append(
                (
                    str(repo_root),
                    str(output_dir),
                    n_segments,
                    seed,
                    False,
                    args.warmup_cycles,
                    args.evaluation_cycles,
                    args.sample_hz,
                    args.force,
                )
            )
        for seed in range(args.paired_replays):
            tasks.append(
                (
                    str(repo_root),
                    str(output_dir),
                    n_segments,
                    seed,
                    True,
                    args.warmup_cycles,
                    args.evaluation_cycles,
                    args.sample_hz,
                    args.force,
                )
            )

    trials = []
    if args.workers == 1:
        # Run directly in the parent process.  Besides being useful for
        # debugging, this is the reliable resume path on Windows hosts whose
        # application-control policy blocks MuJoCo plugin DLL loading inside
        # newly spawned multiprocessing workers.
        for completed, task in enumerate(tasks, start=1):
            trials.append(_worker(task))
            if completed % 25 == 0 or completed == len(tasks):
                print(f"completed {completed}/{len(tasks)} center-null trials", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {executor.submit(_worker, task): task for task in tasks}
            for completed, future in enumerate(as_completed(futures), start=1):
                trials.append(future.result())
                if completed % 25 == 0 or completed == len(tasks):
                    print(f"completed {completed}/{len(tasks)} center-null trials", flush=True)

    # Include completed extension trials already present in this same formal
    # output directory.  This supports targeted sample-size increases for one
    # N without discarding the original 128-trial ensembles for other N.
    ordinary_lookup = {
        (int(row["n_segments"]), int(row["seed"])): row
        for row in trials
        if not row["complemented_iid_bits"]
    }
    for path in (output_dir / "trial_records" / "ordinary").glob("N*_seed*.json"):
        row = json.loads(path.read_text(encoding="utf-8"))
        key = (int(row["n_segments"]), int(row["seed"]))
        if key[0] in segments:
            ordinary_lookup[key] = row
    complement_lookup = {
        (int(row["n_segments"]), int(row["seed"])): row
        for row in trials
        if row["complemented_iid_bits"]
    }
    for path in (output_dir / "trial_records" / "complement").glob("N*_seed*.json"):
        row = json.loads(path.read_text(encoding="utf-8"))
        key = (int(row["n_segments"]), int(row["seed"]))
        if key[0] in segments and key[1] < args.paired_replays:
            complement_lookup[key] = row
    ordinary = [ordinary_lookup[key] for key in sorted(ordinary_lookup)]
    complements = [complement_lookup[key] for key in sorted(complement_lookup)]
    trials = ordinary + complements
    summary = _aggregate(trials, segments)
    complement_pairs = _complement_pairs(trials, segments, args.paired_replays)
    trend = _trend(summary)
    pair_bias = np.asarray([row["pair_bias_m"] for row in complement_pairs])
    validation = {
        "all_trials_pass": all(row["status"] == "PASS" for row in trials),
        "ordinary_trial_count": len(ordinary),
        "ordinary_trials_by_N": {
            str(n_segments): sum(
                int(row["n_segments"]) == n_segments for row in ordinary
            )
            for n_segments in segments
        },
        "complement_pairs_per_N": args.paired_replays,
        "symmetric_initialization": True,
        "center_rooted": True,
        "explicit_mirror_contact_pairs": True,
        "mean_complement_pair_bias_m": float(np.mean(pair_bias)),
        "max_abs_N_mean_complement_pair_bias_m": float(
            max(
                abs(
                    np.mean(
                        [
                            row["pair_bias_m"]
                            for row in complement_pairs
                            if row["n_segments"] == n_segments
                        ]
                    )
                )
                for n_segments in segments
            )
        ),
        **trend,
    }
    _write_csv(output_dir / "iid_trials.csv", sorted(ordinary, key=lambda row: (row["n_segments"], row["seed"])))
    _write_csv(output_dir / "iid_summary.csv", summary)
    _write_csv(output_dir / "complement_pairs.csv", complement_pairs)
    (output_dir / "validation.json").write_text(
        json.dumps(validation, indent=2), encoding="utf-8"
    )
    (output_dir / "run_configuration.json").write_text(
        json.dumps(
            {
                "segments": list(segments),
                "ordinary_trials_requested_per_N": args.replays,
                "ordinary_trials_by_N": {
                    str(n_segments): sum(
                        int(row["n_segments"]) == n_segments for row in ordinary
                    )
                    for n_segments in segments
                },
                "complement_pairs_per_N": args.paired_replays,
                "warmup_cycles": args.warmup_cycles,
                "evaluation_cycles": args.evaluation_cycles,
                "sample_hz": args.sample_hz,
                "body_yaw_half_amplitude_deg": 15.0,
                "spatial_phase_increment_rad": -math.pi,
                "leg_yaw_locked": True,
                "pitch_process": "IID Bernoulli(0.5) per leg per half-cycle",
                "initialization": "straight all-down, body-wave-off symmetric settle",
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    _plots(output_dir, summary, ordinary, complement_pairs)


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            repo_root
            / "exp_data"
            / "reliability"
            / "step25_center_rooted_dynamic_null"
        ),
    )
    parser.add_argument("--segments", type=int, nargs="+", default=list(SEGMENTS))
    parser.add_argument("--replays", type=int, default=128)
    parser.add_argument("--paired-replays", type=int, default=16)
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--sample-hz", type=float, default=100.0)
    parser.add_argument("--workers", type=int, default=max(1, min(6, (os.cpu_count() or 2) // 2)))
    parser.add_argument("--force", action="store_true")
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
