"""Render physical-robot wave-number demos with an explicit pitch audit.

This adapter reuses the locked Step-30 controller without changing the null
model or its data.  It substitutes only the audited center-rooted measured-STL
robot and uses each physical leg geom as the contact/slip visualization object.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import render_center_rooted_dynamic_null_wave_flip as shared_renderer  # noqa: E402
import run_center_rooted_dynamic_null_iid as center  # noqa: E402
import run_center_rooted_dynamic_null_wave_flip as experiment  # noqa: E402
import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    LEG_DOWN_DEG,
    LEG_UP_DEG,
    OMEGA_BODY,
    _actual_contact_flags,
    _leg_body_map,
    _root_roll_pitch_deg,
)


OUTPUT_NAME = "step31_physical_robot_wave_number_demos"
PHYSICAL_VARIANT = "center_rooted_physical_robot"
DEFAULT_N_WAVE = tuple(0.5 * value for value in range(1, 9))
CONTROLLER_CLASS = experiment.CoupledContactPatternController


def _repo_root() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def _output_dir(repo_root: Path) -> Path:
    return repo_root / "exp_data" / "reliability" / OUTPUT_NAME


def _model_path(repo_root: Path, n_segments: int) -> Path:
    return (
        repo_root
        / "exp_data"
        / "reliability"
        / "step26_center_rooted_physical_robot"
        / "models"
        / PHYSICAL_VARIANT
        / f"centipede_{n_segments}seg_{PHYSICAL_VARIANT}.xml"
    )


def _leg_geom_ids(model, n_segments: int, mujoco) -> np.ndarray:
    ids = np.empty((n_segments, 2), dtype=int)
    for segment_index in range(n_segments):
        for side_index, side in enumerate(("R", "L")):
            name = f"leg_geom_{segment_index + 1}_{side}"
            geom_id = int(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            )
            if geom_id < 0:
                raise KeyError(name)
            ids[segment_index, side_index] = geom_id
    return ids


def prepare_physical_simulation(
    repo_root: Path,
    *,
    n_segments: int,
    n_wave: float,
    phase_offset_deg: float,
    mode: str,
    warmup_cycles: int,
) -> experiment.Prepared:
    """Prepare the measured-STL robot with the exact Step-30 controller."""
    import mujoco

    build_indices, compute_ctrl = base._install_project_imports(repo_root)
    model_path = _model_path(repo_root, n_segments)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    indices = build_indices(n_segments)
    controller = CONTROLLER_CLASS(
        n_segments,
        model.nu,
        indices,
        compute_ctrl,
        n_wave=n_wave,
        global_phase_deg=phase_offset_deg,
        flipped=mode == experiment.MODE_FLIPPED,
    )
    root_body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
    )
    if root_body_id < 0:
        raise KeyError("robot_root")
    body_map = _leg_body_map(model, n_segments, mujoco)
    contact_geom_ids = _leg_geom_ids(model, n_segments, mujoco)
    dt = float(model.opt.timestep)
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    warmup_steps = int(math.ceil(warmup_cycles * gait_period_s / dt))
    warmup_time_s = warmup_steps * dt

    initial_target = center._symmetric_initial_target(model.nu, indices)
    settle_steps = int(math.ceil(experiment.SETTLE_TIME_S / dt))
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial_target
        mujoco.mj_step(model, data)

    bridge_target = controller.control(0.0)
    bridge_steps = int(math.ceil(experiment.BRIDGE_TIME_S / dt))
    for step in range(bridge_steps):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial_target + alpha * bridge_target
        mujoco.mj_step(model, data)

    for step in range(warmup_steps):
        body_time_s = step * dt
        data.ctrl[:] = controller.control(body_time_s)
        mujoco.mj_step(model, data)

    return experiment.Prepared(
        model=model,
        data=data,
        controller=controller,
        indices=indices,
        root_body_id=root_body_id,
        body_map=body_map,
        foot_geom_ids=contact_geom_ids,
        dt=dt,
        warmup_time_s=warmup_time_s,
    )


def _mesh_min_z(model, data, geom_id: int) -> float:
    data_id = int(model.geom_dataid[geom_id])
    if data_id < 0:
        return float(data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 0])
    address = int(model.mesh_vertadr[data_id])
    count = int(model.mesh_vertnum[data_id])
    vertices = np.asarray(model.mesh_vert[address : address + count])
    rotation = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
    world = np.asarray(data.geom_xpos[geom_id]) + vertices @ rotation.T
    return float(np.min(world[:, 2]))


def _kinematic_pitch_audit(repo_root: Path, n_segments: int) -> dict:
    """Verify from geometry that +5 deg is down and -40 deg is up."""
    import mujoco

    model_path = _model_path(repo_root, n_segments)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    rows = []
    for segment_index in range(n_segments):
        for side in ("R", "L"):
            samples = {}
            for label, target_deg in (
                ("up", float(LEG_UP_DEG)),
                ("down", float(LEG_DOWN_DEG)),
            ):
                data.qpos[:] = model.qpos0
                joint_name = f"Leg{segment_index + 1}_{side}"
                joint_id = int(
                    mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                    )
                )
                qpos_address = int(model.jnt_qposadr[joint_id])
                data.qpos[qpos_address] = math.radians(target_deg)
                mujoco.mj_forward(model, data)
                geom_id = int(
                    mujoco.mj_name2id(
                        model,
                        mujoco.mjtObj.mjOBJ_GEOM,
                        f"leg_geom_{segment_index + 1}_{side}",
                    )
                )
                samples[label] = _mesh_min_z(model, data, geom_id)
            rows.append(
                {
                    "segment": segment_index + 1,
                    "side": side,
                    "up_target_deg": float(LEG_UP_DEG),
                    "down_target_deg": float(LEG_DOWN_DEG),
                    "up_min_leg_geom_z_m": samples["up"],
                    "down_min_leg_geom_z_m": samples["down"],
                    "lift_margin_m": samples["up"] - samples["down"],
                    "down_is_lower": samples["down"] < samples["up"],
                }
            )
    margins = np.asarray([row["lift_margin_m"] for row in rows], dtype=float)
    return {
        "model_path": str(model_path),
        "method": "MuJoCo forward kinematics over every physical leg mesh",
        "rows": rows,
        "n_legs": len(rows),
        "all_down_targets_lower_leg_geometry": bool(
            all(row["down_is_lower"] for row in rows)
        ),
        "minimum_lift_margin_m": float(np.min(margins)),
        "mean_lift_margin_m": float(np.mean(margins)),
        "maximum_lift_margin_m": float(np.max(margins)),
    }


def _leg_qpos_addresses(model, indices) -> tuple[np.ndarray, np.ndarray]:
    _shoulder_r, leg_r, _shoulder_l, leg_l, _link_y, _link_p = indices

    def addresses(actuator_indices) -> np.ndarray:
        joint_ids = np.asarray(model.actuator_trnid[actuator_indices, 0], dtype=int)
        return np.asarray(model.jnt_qposadr[joint_ids], dtype=int)

    return addresses(leg_r), addresses(leg_l)


def _dynamic_pitch_audit(
    repo_root: Path,
    *,
    n_segments: int,
    n_wave: float,
    phase_offset_deg: float,
    warmup_cycles: int,
    duration_cycles: float,
) -> dict:
    """Measure pitch tracking and contact occupancy on the physical robot."""
    import mujoco

    prepared = prepare_physical_simulation(
        repo_root,
        n_segments=n_segments,
        n_wave=n_wave,
        phase_offset_deg=phase_offset_deg,
        mode=experiment.MODE_CANONICAL,
        warmup_cycles=warmup_cycles,
    )
    model, data, controller = prepared.model, prepared.data, prepared.controller
    leg_r_qpos, leg_l_qpos = _leg_qpos_addresses(model, prepared.indices)
    _shoulder_r, leg_r, _shoulder_l, leg_l, _link_y, _link_p = prepared.indices
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    steps = int(math.ceil(duration_cycles * gait_period_s / prepared.dt))
    contact_and_down = 0
    contact_and_up = 0
    down_count = 0
    up_count = 0
    complement_count = 0
    pair_count = 0
    errors = []
    max_abs_roll_deg = 0.0
    max_abs_pitch_deg = 0.0
    finite = True
    stability_body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Chasis1")
    )
    if stability_body_id < 0:
        raise KeyError("Chasis1")
    for step in range(steps):
        local_time_s = step * prepared.dt
        body_time_s = prepared.warmup_time_s + local_time_s
        target = controller.control(body_time_s)
        data.ctrl[:] = target
        mujoco.mj_step(model, data)
        stance = controller.stance_flags(body_time_s)
        contact = _actual_contact_flags(model, data, prepared.body_map)
        actual_pitch = np.column_stack((data.qpos[leg_r_qpos], data.qpos[leg_l_qpos]))
        target_pitch = np.column_stack((target[leg_r], target[leg_l]))
        errors.extend(np.abs(actual_pitch - target_pitch).ravel())
        contact_and_down += int(np.sum(contact & stance))
        contact_and_up += int(np.sum(contact & ~stance))
        down_count += int(np.sum(stance))
        up_count += int(np.sum(~stance))
        complement_count += int(np.sum(np.logical_xor(stance[:, 0], stance[:, 1])))
        pair_count += n_segments
        stability_pose = np.zeros(7, dtype=float)
        stability_pose[3:7] = np.asarray(data.xquat[stability_body_id])
        roll_deg, pitch_deg = _root_roll_pitch_deg(stability_pose)
        max_abs_roll_deg = max(max_abs_roll_deg, abs(roll_deg))
        max_abs_pitch_deg = max(max_abs_pitch_deg, abs(pitch_deg))
        finite &= bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and np.all(np.isfinite(data.qacc))
        )
    errors_deg = np.degrees(np.asarray(errors, dtype=float))
    return {
        "N_wave": float(n_wave),
        "phase_offset_deg": float(phase_offset_deg),
        "duration_cycles": float(duration_cycles),
        "commanded_down_fraction": float(down_count / (down_count + up_count)),
        "left_right_complement_fraction": float(complement_count / pair_count),
        "contact_given_down": float(contact_and_down / down_count),
        "contact_given_up": float(contact_and_up / up_count),
        "contact_selectivity": float(
            contact_and_down / down_count - contact_and_up / up_count
        ),
        "mean_abs_pitch_tracking_error_deg": float(np.mean(errors_deg)),
        "p95_abs_pitch_tracking_error_deg": float(np.quantile(errors_deg, 0.95)),
        "max_abs_pitch_tracking_error_deg": float(np.max(errors_deg)),
        "max_abs_root_roll_deg": max_abs_roll_deg,
        "max_abs_root_pitch_deg": max_abs_pitch_deg,
        "all_states_finite": finite,
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _plot_pitch_audit(output_dir: Path, kinematic: dict, dynamic: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    margins_mm = 1000.0 * np.asarray(
        [row["lift_margin_m"] for row in kinematic["rows"]], dtype=float
    )
    waves = np.asarray([row["N_wave"] for row in dynamic], dtype=float)
    down = np.asarray([row["contact_given_down"] for row in dynamic], dtype=float)
    up = np.asarray([row["contact_given_up"] for row in dynamic], dtype=float)
    tracking = np.asarray(
        [row["mean_abs_pitch_tracking_error_deg"] for row in dynamic], dtype=float
    )

    fig, axes = plt.subplots(3, 1, figsize=(8.0, 10.0), constrained_layout=True)
    axes[0].plot(range(1, len(margins_mm) + 1), margins_mm, "o", color="#24796f")
    axes[0].axhline(0.0, color="#555555", linewidth=1.0)
    axes[0].set_ylabel("Up-minus-down clearance (mm)")
    axes[0].set_title("Physical-leg pitch geometry audit")
    axes[0].grid(alpha=0.25)

    axes[1].plot(waves, down, "o-", label="Contact | pitch down", color="#24796f")
    axes[1].plot(waves, up, "s-", label="Contact | pitch up", color="#c4513f")
    axes[1].set_ylabel("Conditional contact fraction")
    axes[1].set_xticks(waves)
    axes[1].legend(frameon=False)
    axes[1].grid(alpha=0.25)

    axes[2].plot(waves, tracking, "o-", color="#4c6a9b")
    axes[2].set_xlabel(r"Spatial wave number, $N_{wave}$")
    axes[2].set_ylabel("Mean pitch tracking error (deg)")
    axes[2].set_xticks(waves)
    axes[2].grid(alpha=0.25)
    fig.savefig(figure_dir / "pitch_geometry_and_tracking_audit.png", dpi=240)
    plt.close(fig)


def _write_readme(output_dir: Path, kinematic: dict, dynamic: list[dict]) -> None:
    min_margin_mm = 1000.0 * float(kinematic["minimum_lift_margin_m"])
    mean_selectivity = float(np.mean([row["contact_selectivity"] for row in dynamic]))
    mean_tracking = float(
        np.mean([row["mean_abs_pitch_tracking_error_deg"] for row in dynamic])
    )
    mean_p95_tracking = float(
        np.mean([row["p95_abs_pitch_tracking_error_deg"] for row in dynamic])
    )
    largest_up_contact = max(dynamic, key=lambda row: row["contact_given_up"])
    text = f"""# Step 31: measured physical robot wave-number demos

This directory contains visualization diagnostics, not a performance sweep.
The controller is bitwise the Step-30 coupling law; only the model is replaced
by the audited center-rooted measured-STL robot.

## Pitch audit

- Up target: {LEG_UP_DEG:g} deg; down target: {LEG_DOWN_DEG:g} deg.
- All {kinematic['n_legs']} physical legs passed the geometry-direction check.
- Minimum up-minus-down leg-mesh clearance: {min_margin_mm:.2f} mm.
- Mean dynamic contact selectivity `P(contact|down)-P(contact|up)`:
  {mean_selectivity:+.3f}.
- Mean pitch tracking error across tested wave numbers: {mean_tracking:.2f} deg.
- Mean 95th-percentile pitch tracking error: {mean_p95_tracking:.2f} deg.
- Largest pitch-up contact fraction: {largest_up_contact['contact_given_up']:.3f}
  at `N_wave={largest_up_contact['N_wave']:g}`.

The physical XML mirrors the left and right link frames through body
orientations. Identical textual joint-axis entries therefore do not imply
identical world-frame rotations. Direct MuJoCo kinematics confirms that
`-40 deg` raises and `+5 deg` lowers both sides.

The remaining pitch issue is temporal: the binary target changes by 45 deg at
each zero crossing, while the torque-limited physical joint cannot change
angle instantaneously. The large transition-tail tracking error is therefore
real. It must not be confused with a reversed pitch convention. Smoothing or
rate-limiting this transition would define a different controller and is not
applied in these demos.

## Demos

Each video compares canonical pitch timing with an exact left/right pitch
exchange at one fixed `N_wave`. Green denotes commanded pitch-down, red
commanded pitch-up, and yellow measured ground contact. The videos use global phase
0 deg, four warm-up cycles, and the same 0.4-rad body-yaw half-amplitude as
Step 30.

See `demos/demo_manifest.csv`, `pitch_kinematic_audit.json`,
`pitch_dynamic_audit.csv`, and `figures/pitch_geometry_and_tracking_audit.png`.
"""
    (output_dir / "README.md").write_text(text, encoding="utf-8")


def _write_validation(
    output_dir: Path,
    kinematic: dict,
    dynamic: list[dict],
    demo_rows: list[dict],
) -> dict:
    expected_waves = sorted(float(row["N_wave"]) for row in dynamic)
    rendered_waves = sorted(float(row["N_wave"]) for row in demo_rows)
    gates = {
        "all_physical_legs_have_correct_pitch_direction": bool(
            kinematic["all_down_targets_lower_leg_geometry"]
        ),
        "all_dynamic_states_finite": bool(
            all(row["all_states_finite"] for row in dynamic)
        ),
        "left_right_commands_are_exactly_complementary": bool(
            all(
                math.isclose(
                    float(row["left_right_complement_fraction"]),
                    1.0,
                    abs_tol=1.0e-12,
                )
                for row in dynamic
            )
        ),
        "commanded_down_fraction_is_one_half": bool(
            all(
                math.isclose(
                    float(row["commanded_down_fraction"]),
                    0.5,
                    abs_tol=1.0e-12,
                )
                for row in dynamic
            )
        ),
        "one_video_per_requested_wave_number": expected_waves == rendered_waves,
        "all_video_files_exist_and_are_nonempty": bool(
            all(
                Path(row["video"]).is_file()
                and Path(row["video"]).stat().st_size > 0
                for row in demo_rows
            )
        ),
    }
    warnings = []
    for row in dynamic:
        if float(row["contact_given_up"]) > 0.05:
            warnings.append(
                {
                    "N_wave": float(row["N_wave"]),
                    "issue": "non-negligible contact while pitch-up is commanded",
                    "contact_given_up": float(row["contact_given_up"]),
                }
            )
    payload = {
        "passed": bool(all(gates.values())),
        "gates": gates,
        "warnings": warnings,
        "pitch_transition_diagnostic": {
            "interpretation": (
                "The 45-deg binary target jump creates a transient tracking tail; "
                "this is not a pitch-sign failure."
            ),
            "mean_p95_abs_tracking_error_deg": float(
                np.mean(
                    [row["p95_abs_pitch_tracking_error_deg"] for row in dynamic]
                )
            ),
            "maximum_abs_tracking_error_deg": float(
                max(row["max_abs_pitch_tracking_error_deg"] for row in dynamic)
            ),
        },
    }
    (output_dir / "validation_summary.json").write_text(
        json.dumps(payload, indent=2), encoding="utf-8"
    )
    return payload


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = _model_path(repo_root, args.n_segments)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)

    kinematic = _kinematic_pitch_audit(repo_root, args.n_segments)
    (output_dir / "pitch_kinematic_audit.json").write_text(
        json.dumps(kinematic, indent=2), encoding="utf-8"
    )
    if not kinematic["all_down_targets_lower_leg_geometry"]:
        raise RuntimeError("physical pitch direction audit failed")

    dynamic = []
    for n_wave in sorted(set(float(value) for value in args.n_wave)):
        row = _dynamic_pitch_audit(
            repo_root,
            n_segments=args.n_segments,
            n_wave=n_wave,
            phase_offset_deg=args.phase_offset_deg,
            warmup_cycles=args.warmup_cycles,
            duration_cycles=args.audit_duration_cycles,
        )
        dynamic.append(row)
        print(
            f"pitch audit N_wave={n_wave:g}: "
            f"P(contact|down)={row['contact_given_down']:.3f}, "
            f"P(contact|up)={row['contact_given_up']:.3f}, "
            f"mean tracking error={row['mean_abs_pitch_tracking_error_deg']:.2f} deg",
            flush=True,
        )
    _write_csv(output_dir / "pitch_dynamic_audit.csv", dynamic)
    (output_dir / "pitch_dynamic_audit.json").write_text(
        json.dumps(dynamic, indent=2), encoding="utf-8"
    )
    _plot_pitch_audit(output_dir, kinematic, dynamic)

    original_prepare = experiment.prepare_simulation
    experiment.prepare_simulation = prepare_physical_simulation
    try:
        demo_rows = []
        for n_wave in sorted(set(float(value) for value in args.n_wave)):
            row = shared_renderer._render_comparison(
                repo_root,
                output_dir,
                n_segments=args.n_segments,
                n_wave=n_wave,
                phase_offset_deg=args.phase_offset_deg,
                warmup_cycles=args.warmup_cycles,
                duration_cycles=args.duration_cycles,
                fps=args.fps,
            )
            row.update(
                {
                    "model_family": "measured_stl_physical_robot",
                    "model_variant": PHYSICAL_VARIANT,
                    "model_path": str(model_path),
                }
            )
            demo_rows.append(row)
            print(
                f"demo N_wave={n_wave:g}: "
                f"canonical={1000*row['canonical_demo_displacement_m']:+.1f} mm, "
                f"flipped={1000*row['flipped_demo_displacement_m']:+.1f} mm",
                flush=True,
            )
    finally:
        experiment.prepare_simulation = original_prepare

    demo_dir = output_dir / "demos"
    _write_csv(demo_dir / "demo_manifest.csv", demo_rows)
    (demo_dir / "demo_manifest.json").write_text(
        json.dumps(demo_rows, indent=2), encoding="utf-8"
    )
    _write_validation(output_dir, kinematic, dynamic, demo_rows)
    _write_readme(output_dir, kinematic, dynamic)
    (output_dir / "run_configuration.json").write_text(
        json.dumps(
            {
                "model_family": "measured_stl_physical_robot",
                "model_variant": PHYSICAL_VARIANT,
                "model_path": str(model_path),
                "n_segments": args.n_segments,
                "N_wave": list(args.n_wave),
                "phase_offset_deg": args.phase_offset_deg,
                "warmup_cycles": args.warmup_cycles,
                "audit_duration_cycles": args.audit_duration_cycles,
                "demo_duration_cycles": args.duration_cycles,
                "fps": args.fps,
                "controller_source": (
                    "run_center_rooted_dynamic_null_wave_flip.py; exact Step-30 coupling"
                ),
                "body_wave_half_amplitude_rad": (
                    experiment.BODY_WAVE_HALF_AMPLITUDE_RAD
                ),
                "leg_yaw_body_phase_offset_rad": (
                    experiment.LEG_YAW_BODY_PHASE_OFFSET_RAD
                ),
                "leg_pitch_body_phase_offset_rad": (
                    experiment.LEG_PITCH_BODY_PHASE_OFFSET_RAD
                ),
                "left_right_pitch_phase_offset_rad": (
                    experiment.LEFT_RIGHT_PITCH_PHASE_OFFSET_RAD
                ),
                "pitch_targets_deg": {
                    "up": LEG_UP_DEG,
                    "down": LEG_DOWN_DEG,
                },
            },
            indent=2,
        ),
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    repo_root = _repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument("--output-dir", type=Path, default=_output_dir(repo_root))
    parser.add_argument("--n-segments", type=int, default=10)
    parser.add_argument(
        "--n-wave", type=float, nargs="+", default=DEFAULT_N_WAVE
    )
    parser.add_argument("--phase-offset-deg", type=float, default=0.0)
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--audit-duration-cycles", type=float, default=2.0)
    parser.add_argument("--duration-cycles", type=float, default=4.0)
    parser.add_argument("--fps", type=int, default=30)
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
