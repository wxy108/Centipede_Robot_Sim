"""Validate physical equivalence and stability of center-rooted STL robots."""

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

import generate_center_rooted_physical_robot_models as generator  # noqa: E402
import run_center_rooted_dynamic_null_iid as center  # noqa: E402
import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _rotation(mujoco, quat: np.ndarray) -> np.ndarray:
    flat = np.empty(9, dtype=float)
    mujoco.mju_quat2Mat(flat, np.asarray(quat, dtype=float))
    return flat.reshape(3, 3)


VARIANTS = (
    "head_endpoint",
    "center_rooted",
    "tail_rooted",
)


def _root_node(n_segments: int, variant: str) -> str:
    if variant == "head_endpoint":
        return "C1"
    if variant == "tail_rooted":
        return f"C{n_segments}"
    if variant != "center_rooted":
        raise ValueError(variant)
    if n_segments % 2:
        return f"C{(n_segments + 1) // 2}"
    return f"Y{n_segments // 2}"


def _body_name_map(
    n_segments: int,
    variant: str,
    source_model,
    mujoco,
) -> dict[str, str]:
    if variant == "head_endpoint":
        return {
            mujoco.mj_id2name(source_model, mujoco.mjtObj.mjOBJ_BODY, body_id):
            mujoco.mj_id2name(source_model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            for body_id in range(1, source_model.nbody)
        }
    root_source_name = generator._source_body_name(
        _root_node(n_segments, variant)
    )
    mapping = {}
    for body_id in range(1, source_model.nbody):
        name = mujoco.mj_id2name(source_model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name == root_source_name:
            output_name = "robot_root"
        elif name == "robot_root":
            output_name = "Chasis1"
        else:
            output_name = name
        mapping[name] = output_name
    return mapping


def _relative_body_pose(model, data, body_id: int, root_id: int, mujoco):
    root_rotation = _rotation(mujoco, data.xquat[root_id])
    body_rotation = _rotation(mujoco, data.xquat[body_id])
    position = root_rotation.T @ (data.xpos[body_id] - data.xpos[root_id])
    rotation = root_rotation.T @ body_rotation
    return position, rotation


def _set_named_configuration(model, data, values: dict[str, float], mujoco) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    for name, value in values.items():
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        if joint_id < 0:
            raise KeyError(name)
        address = int(model.jnt_qposadr[joint_id])
        data.qpos[address] = value
    mujoco.mj_forward(model, data)


def _structure_and_fk(
    repo_root: Path,
    output_dir: Path,
    n_segments: int,
    variant: str,
) -> tuple[dict, list[dict]]:
    import mujoco

    source_path = repo_root / "centipede_sim" / "model" / f"centipede_{n_segments}seg.xml"
    model_variant = {
        "head_endpoint": generator.REFERENCE_VARIANT,
        "center_rooted": generator.VARIANT,
        "tail_rooted": generator.TAIL_VARIANT,
    }[variant]
    rebuilt_path = (
        output_dir
        / "models"
        / model_variant
        / f"centipede_{n_segments}seg_{model_variant}.xml"
    )
    source = mujoco.MjModel.from_xml_path(str(source_path))
    rebuilt = mujoco.MjModel.from_xml_path(str(rebuilt_path))
    source_data = mujoco.MjData(source)
    rebuilt_data = mujoco.MjData(rebuilt)
    mapping = _body_name_map(n_segments, variant, source, mujoco)

    max_mass_error = 0.0
    max_inertia_error = 0.0
    max_inertial_position_error = 0.0
    for source_name, rebuilt_name in mapping.items():
        source_id = int(mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_BODY, source_name))
        rebuilt_id = int(mujoco.mj_name2id(rebuilt, mujoco.mjtObj.mjOBJ_BODY, rebuilt_name))
        if rebuilt_id < 0:
            raise KeyError(rebuilt_name)
        max_mass_error = max(
            max_mass_error,
            abs(float(source.body_mass[source_id] - rebuilt.body_mass[rebuilt_id])),
        )
        max_inertia_error = max(
            max_inertia_error,
            float(np.max(np.abs(source.body_inertia[source_id] - rebuilt.body_inertia[rebuilt_id]))),
        )
        max_inertial_position_error = max(
            max_inertial_position_error,
            float(np.linalg.norm(source.body_ipos[source_id] - rebuilt.body_ipos[rebuilt_id])),
        )

    source_joint_names = [
        mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        for joint_id in range(source.njnt)
    ]
    rebuilt_joint_names = [
        mujoco.mj_id2name(rebuilt, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        for joint_id in range(rebuilt.njnt)
    ]
    source_actuator_names = [
        mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        for actuator_id in range(source.nu)
    ]
    rebuilt_actuator_names = [
        mujoco.mj_id2name(rebuilt, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        for actuator_id in range(rebuilt.nu)
    ]

    random = np.random.default_rng(260000 + n_segments)
    configurations = []
    for config_index in range(5):
        values = {}
        for joint_name in source_joint_names:
            if joint_name == "root":
                continue
            if joint_name.startswith("Link") and joint_name.endswith("_P"):
                values[joint_name] = 0.0
            elif joint_name.startswith("Link") and joint_name.endswith("_Y"):
                values[joint_name] = float(random.uniform(-0.22, 0.22))
            elif joint_name.startswith("Shoulder"):
                values[joint_name] = float(random.uniform(-0.18, 0.18))
            elif joint_name.startswith("Leg"):
                values[joint_name] = float(random.uniform(-0.12, 0.12))
        _set_named_configuration(source, source_data, values, mujoco)
        _set_named_configuration(rebuilt, rebuilt_data, values, mujoco)
        source_root_name = generator._source_body_name(
            _root_node(n_segments, variant)
        )
        source_root_id = int(
            mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_BODY, source_root_name)
        )
        rebuilt_root_id = int(
            mujoco.mj_name2id(rebuilt, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
        )
        max_position_error = 0.0
        max_rotation_error = 0.0
        for source_name, rebuilt_name in mapping.items():
            source_id = int(
                mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_BODY, source_name)
            )
            rebuilt_id = int(
                mujoco.mj_name2id(rebuilt, mujoco.mjtObj.mjOBJ_BODY, rebuilt_name)
            )
            source_position, source_rotation = _relative_body_pose(
                source, source_data, source_id, source_root_id, mujoco
            )
            rebuilt_position, rebuilt_rotation = _relative_body_pose(
                rebuilt, rebuilt_data, rebuilt_id, rebuilt_root_id, mujoco
            )
            max_position_error = max(
                max_position_error,
                float(np.linalg.norm(source_position - rebuilt_position)),
            )
            relative_rotation = source_rotation.T @ rebuilt_rotation
            cosine = float(np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0))
            max_rotation_error = max(max_rotation_error, math.acos(cosine))
        configurations.append(
            {
                "variant": variant,
                "n_segments": n_segments,
                "configuration_index": config_index,
                "max_relative_body_position_error_m": max_position_error,
                "max_relative_body_rotation_error_rad": max_rotation_error,
            }
        )

    structure = {
        "variant": variant,
        "n_segments": n_segments,
        "source_total_mass_kg": float(source.body_subtreemass[1]),
        "rebuilt_total_mass_kg": float(rebuilt.body_subtreemass[1]),
        "mass_error_kg": float(rebuilt.body_subtreemass[1] - source.body_subtreemass[1]),
        "max_rigid_body_mass_error_kg": max_mass_error,
        "max_rigid_body_inertia_error_kg_m2": max_inertia_error,
        "max_rigid_body_inertial_position_error_m": max_inertial_position_error,
        "source_body_count": int(source.nbody),
        "rebuilt_body_count": int(rebuilt.nbody),
        "source_geom_count": int(source.ngeom),
        "rebuilt_geom_count": int(rebuilt.ngeom),
        "source_actuator_count": int(source.nu),
        "rebuilt_actuator_count": int(rebuilt.nu),
        "same_joint_name_set": set(source_joint_names) == set(rebuilt_joint_names),
        "same_actuator_order": source_actuator_names == rebuilt_actuator_names,
        "explicit_ground_pair_count": int(rebuilt.npair),
    }
    return structure, configurations


def _smoke(
    repo_root: Path,
    output_dir: Path,
    n_segments: int,
) -> dict:
    model_path = (
        output_dir
        / "models"
        / generator.VARIANT
        / f"centipede_{n_segments}seg_{generator.VARIANT}.xml"
    )
    with center._center_trial_context(
        model_path,
        complement=False,
        stability_body_name="Chasis1",
    ):
        result = base._run_trial(
            repo_root,
            n_segments=n_segments,
            seed=2600 + n_segments,
            mode=base.RANDOM_MODE,
            warmup_cycles=2,
            evaluation_cycles=2,
            sample_hz=50.0,
            trajectory_dir=output_dir / "validation" / "smoke_trajectories",
        )
    return {
        "n_segments": n_segments,
        "status": result["status"],
        "forward_displacement_m": result["forward_displacement_m"],
        "max_abs_roll_deg": result["max_abs_roll_deg"],
        "max_abs_pitch_deg": result["max_abs_pitch_deg"],
        "minimum_root_up_z": result["minimum_root_up_z"],
        "mujoco_warning_total": result["mujoco_warning_total"],
        "all_finite": result["gates"]["all_states_finite"],
    }


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    structure_rows = []
    fk_rows = []
    smoke_rows = []
    for n_segments in sorted(set(args.segments)):
        for variant in VARIANTS:
            structure, configurations = _structure_and_fk(
                repo_root, output_dir, n_segments, variant
            )
            structure_rows.append(structure)
            fk_rows.extend(configurations)
        smoke_rows.append(_smoke(repo_root, output_dir, n_segments))
        print(f"validated N={n_segments}", flush=True)

    validation_dir = output_dir / "validation"
    _write_csv(validation_dir / "structure_equivalence.csv", structure_rows)
    _write_csv(validation_dir / "forward_kinematics_equivalence.csv", fk_rows)
    _write_csv(validation_dir / "stability_smoke.csv", smoke_rows)
    summary = {
        "all_mass_and_inertia_checks_pass": all(
            abs(row["mass_error_kg"]) < 1e-12
            and row["max_rigid_body_mass_error_kg"] < 1e-12
            and row["max_rigid_body_inertia_error_kg_m2"] < 1e-12
            and row["max_rigid_body_inertial_position_error_m"] < 1e-12
            for row in structure_rows
        ),
        "all_count_and_name_checks_pass": all(
            row["source_body_count"] == row["rebuilt_body_count"]
            and row["source_geom_count"] == row["rebuilt_geom_count"]
            and row["source_actuator_count"] == row["rebuilt_actuator_count"]
            and row["same_joint_name_set"]
            and row["same_actuator_order"]
            for row in structure_rows
        ),
        "max_forward_kinematics_position_error_m": max(
            row["max_relative_body_position_error_m"] for row in fk_rows
        ),
        "max_forward_kinematics_rotation_error_rad": max(
            row["max_relative_body_rotation_error_rad"] for row in fk_rows
        ),
        "all_stability_smoke_trials_pass": all(
            row["status"] == "PASS" and row["all_finite"] for row in smoke_rows
        ),
        "maximum_warning_count": max(
            int(row["mujoco_warning_total"]) for row in smoke_rows
        ),
    }
    (validation_dir / "validation_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )


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
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
