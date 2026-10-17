"""Validate the center-rooted dynamic null before scientific use."""

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

import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    BRIDGE_TIME_S,
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    OMEGA_BODY,
    SETTLE_TIME_S,
    YAW_HALF_AMPLITUDE_DEG,
    _actual_contact_flags,
    _leg_body_map,
)


VARIANT = "center_rooted_dynamic_null"
PHASES = (0.0, math.pi)


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _model_path(output_dir: Path, n_segments: int) -> Path:
    return (
        output_dir
        / "models"
        / VARIANT
        / f"centipede_{n_segments}seg_{VARIANT}.xml"
    )


def _control(
    n_ctrl: int,
    indices,
    n_segments: int,
    time_s: float,
    *,
    phase_rad: float,
    wave_enabled: bool,
) -> np.ndarray:
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = indices
    ctrl = np.zeros(n_ctrl, dtype=float)
    ctrl[shoulder_r] = math.radians(LEG_OFFSET_DEG)
    ctrl[shoulder_l] = -math.radians(LEG_OFFSET_DEG)
    ctrl[leg_r] = math.radians(LEG_DOWN_DEG)
    ctrl[leg_l] = math.radians(LEG_DOWN_DEG)
    if wave_enabled:
        ctrl[link_y] = math.radians(YAW_HALF_AMPLITUDE_DEG) * np.sin(
            OMEGA_BODY * time_s
            + np.arange(n_segments - 1, dtype=float) * (-math.pi)
            + phase_rad
        )
    ctrl[link_p] = 0.0
    return ctrl


def _deterministic_worker(task: tuple) -> dict:
    (
        repo_root_text,
        output_dir_text,
        n_segments,
        phase_rad,
        wave_enabled,
        warmup_cycles,
        evaluation_cycles,
    ) = task
    import mujoco

    repo_root = Path(repo_root_text)
    output_dir = Path(output_dir_text)
    model_path = _model_path(output_dir, n_segments)
    build_indices, _canonical = base._install_project_imports(repo_root)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    indices = build_indices(n_segments)
    root_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root"))
    body_map = _leg_body_map(model, n_segments, mujoco)
    dt = float(model.opt.timestep)
    gait_period = 2.0 * math.pi / OMEGA_BODY
    initial = _control(
        model.nu,
        indices,
        n_segments,
        0.0,
        phase_rad=0.0,
        wave_enabled=False,
    )

    settle_steps = int(math.ceil(SETTLE_TIME_S / dt))
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial
        mujoco.mj_step(model, data)
    target = _control(
        model.nu,
        indices,
        n_segments,
        0.0,
        phase_rad=phase_rad,
        wave_enabled=wave_enabled,
    )
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / dt))
    for step in range(bridge_steps):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial + alpha * target
        mujoco.mj_step(model, data)
    warmup_steps = int(math.ceil(warmup_cycles * gait_period / dt))
    for step in range(warmup_steps):
        data.ctrl[:] = _control(
            model.nu,
            indices,
            n_segments,
            step * dt,
            phase_rad=phase_rad,
            wave_enabled=wave_enabled,
        )
        mujoco.mj_step(model, data)

    start_com = np.asarray(data.subtree_com[root_id]).copy()
    evaluation_steps = int(math.ceil(evaluation_cycles * gait_period / dt))
    contact_sum = 0
    contact_count = 0
    finite = True
    for step in range(evaluation_steps):
        time_s = warmup_cycles * gait_period + step * dt
        data.ctrl[:] = _control(
            model.nu,
            indices,
            n_segments,
            time_s,
            phase_rad=phase_rad,
            wave_enabled=wave_enabled,
        )
        mujoco.mj_step(model, data)
        finite &= bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
        if step % 10 == 0:
            contacts = _actual_contact_flags(model, data, body_map)
            contact_sum += int(np.sum(contacts))
            contact_count += contacts.size
    final_com = np.asarray(data.subtree_com[root_id]).copy()
    warnings = int(sum(data.warning[index].number for index in range(len(data.warning))))
    return {
        "n_segments": n_segments,
        "phase_rad": phase_rad,
        "phase_deg": math.degrees(phase_rad),
        "wave_enabled": wave_enabled,
        "forward_displacement_m": float(final_com[1] - start_com[1]),
        "lateral_displacement_m": float(final_com[0] - start_com[0]),
        "actual_contact_fraction": contact_sum / contact_count,
        "all_finite": finite,
        "warning_total": warnings,
        "status": "PASS" if finite and warnings == 0 else "FAIL",
    }


def _structure_audit(output_dir: Path, segments: tuple[int, ...]) -> list[dict]:
    import mujoco

    rows = []
    for n_segments in segments:
        model = mujoco.MjModel.from_xml_path(str(_model_path(output_dir, n_segments)))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        sites = []
        for segment_index in range(1, n_segments + 1):
            site_id = int(
                mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_SITE,
                    f"chasis{segment_index}_site",
                )
            )
            sites.append(np.asarray(data.site_xpos[site_id]).copy())
        sites = np.asarray(sites)
        centered_y = sites[:, 1] - np.mean(sites[:, 1])
        expected_mass = 0.417 * n_segments + 0.1172 * (n_segments - 1)
        rows.append(
            {
                "n_segments": n_segments,
                "total_mass_kg": float(model.body_subtreemass[1]),
                "expected_mass_kg": expected_mass,
                "mass_error_kg": float(model.body_subtreemass[1] - expected_mass),
                "whole_robot_com_y_m": float(data.subtree_com[1, 1]),
                "static_y_mirror_error_m": float(
                    np.max(np.abs(centered_y + centered_y[::-1]))
                ),
                "static_x_mirror_error_m": float(
                    np.max(np.abs(sites[:, 0] - sites[::-1, 0]))
                ),
                "actuator_count": int(model.nu),
                "expected_actuator_count": 4 * n_segments + 2 * (n_segments - 1),
                "explicit_contact_pair_count": int(model.npair),
                "equality_count": int(model.neq),
            }
        )
    return rows


def run(args: argparse.Namespace) -> None:
    output_dir = args.output_dir.resolve()
    segments = tuple(sorted(set(args.segments)))
    structure = _structure_audit(output_dir, segments)
    tasks = []
    for n_segments in segments:
        for phase_rad in PHASES:
            tasks.append(
                (
                    str(args.repo_root.resolve()),
                    str(output_dir),
                    n_segments,
                    phase_rad,
                    True,
                    args.warmup_cycles,
                    args.evaluation_cycles,
                )
            )
        tasks.append(
            (
                str(args.repo_root.resolve()),
                str(output_dir),
                n_segments,
                0.0,
                False,
                args.warmup_cycles,
                args.evaluation_cycles,
            )
        )
    deterministic = []
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures = [executor.submit(_deterministic_worker, task) for task in tasks]
        for future in as_completed(futures):
            deterministic.append(future.result())
    deterministic.sort(
        key=lambda row: (row["n_segments"], not row["wave_enabled"], row["phase_rad"])
    )
    _write_csv(output_dir / "validation" / "structure.csv", structure)
    _write_csv(output_dir / "validation" / "deterministic_all_down.csv", deterministic)

    wave_on = [row for row in deterministic if row["wave_enabled"]]
    wave_off = [row for row in deterministic if not row["wave_enabled"]]
    payload = {
        "all_structure_mass_checks_pass": all(
            abs(row["mass_error_kg"]) < 1.0e-10 for row in structure
        ),
        "all_structure_actuator_checks_pass": all(
            row["actuator_count"] == row["expected_actuator_count"]
            for row in structure
        ),
        "max_static_mirror_error_m": max(
            max(row["static_y_mirror_error_m"], row["static_x_mirror_error_m"])
            for row in structure
        ),
        "all_deterministic_trials_pass": all(
            row["status"] == "PASS" for row in deterministic
        ),
        "maximum_absolute_wave_on_displacement_m": max(
            abs(row["forward_displacement_m"]) for row in wave_on
        ),
        "maximum_absolute_wave_off_displacement_m": max(
            abs(row["forward_displacement_m"]) for row in wave_off
        ),
        "mean_wave_on_displacement_m": float(
            np.mean([row["forward_displacement_m"] for row in wave_on])
        ),
    }
    validation_dir = output_dir / "validation"
    validation_dir.mkdir(parents=True, exist_ok=True)
    (validation_dir / "validation_summary.json").write_text(
        json.dumps(payload, indent=2), encoding="utf-8"
    )


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
    parser.add_argument("--segments", type=int, nargs="+", default=list(range(4, 11)))
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--workers", type=int, default=max(1, min(6, (os.cpu_count() or 2) // 2)))
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
