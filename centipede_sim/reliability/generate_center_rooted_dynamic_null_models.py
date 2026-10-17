"""Generate center-rooted, reflection-projected primitive null robots.

The earlier primitive null removed CAD and inertial polarity but retained a
one-ended serial tree.  This generator removes that remaining implementation
polarity:

* odd N: the middle physical segment carries the free joint;
* even N: the middle connector carries the free joint and the central yaw is
  represented by two constrained half-joints (+q/2 and -q/2);
* head and tail are separate, recursively mirrored branches;
* every floor contact is an explicit pair, and mirrored constraints are
  adjacent with alternating first order;
* the Newton/elliptic contact solve uses tight tolerances;
* inter-segment pitch is equality-locked at zero because the experiment uses
  only the planar body-yaw wave.

The controller-facing actuator order remains identical to the source models.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import generate_physical_primitive_null_models as primitive  # noqa: E402


VARIANT = "center_rooted_dynamic_null"
JOINT_DAMPING = 0.0013
JOINT_ARMATURE = 0.0083
JOINT_FRICTIONLOSS = 0.069


def _geom(
    parent: ET.Element,
    name: str,
    geom_type: str,
    *,
    mass: float,
    size: str,
    pos: str | None = None,
    rgba: str,
) -> ET.Element:
    attributes = {
        "name": name,
        "type": geom_type,
        "size": size,
        "mass": f"{mass:.12g}",
        "rgba": rgba,
        "contype": "2",
        "conaffinity": "0",
    }
    if pos is not None:
        attributes["pos"] = pos
    return ET.SubElement(parent, "geom", attributes)


def _add_boundary_parent_halves(
    segment_body: ET.Element,
    boundary_index: int,
    joint_position: str,
) -> None:
    _geom(
        segment_body,
        f"link_yaw_parent_half_geom_{boundary_index}",
        "box",
        mass=0.5 * primitive.LINK_YAW_MASS_KG,
        size=" ".join(f"{value:.9g}" for value in primitive.LINK_YAW_HALF_SIZE_M),
        pos=joint_position,
        rgba="0.34 0.50 0.78 1",
    )
    _geom(
        segment_body,
        f"link_pitch_parent_half_geom_{boundary_index}",
        "sphere",
        mass=0.5 * primitive.LINK_PITCH_MASS_KG,
        size=f"{primitive.LINK_PITCH_RADIUS_M:.9g}",
        pos=joint_position,
        rgba="0.65 0.72 0.88 1",
    )


def _append_head_branch(parent_segment: ET.Element, segment_index: int) -> None:
    """Recurse from a segment toward lower segment indices (+y)."""

    if segment_index <= 1:
        return
    boundary_index = segment_index - 1
    half_spacing = 0.5 * primitive.SEGMENT_SPACING_M
    joint_position = f"0 {half_spacing:.9g} 0"
    _add_boundary_parent_halves(parent_segment, boundary_index, joint_position)

    # Reverse of the source yaw->pitch chain.  Pitch is listed first and both
    # axes are negated because parent and child sides have been exchanged.
    connector_pitch = ET.SubElement(
        parent_segment,
        "body",
        {"name": f"LinkMotor{boundary_index}_R", "pos": joint_position},
    )
    primitive._add_joint(
        connector_pitch,
        f"Link{boundary_index}_P",
        "-1 0 0",
        "-1.5 1.5",
    )
    _geom(
        connector_pitch,
        f"link_pitch_geom_{boundary_index}",
        "sphere",
        mass=0.5 * primitive.LINK_PITCH_MASS_KG,
        size=f"{primitive.LINK_PITCH_RADIUS_M:.9g}",
        rgba="0.72 0.78 0.90 1",
    )
    connector_yaw = ET.SubElement(
        connector_pitch,
        "body",
        {"name": f"LinkMotor{boundary_index}_PnY"},
    )
    primitive._add_joint(
        connector_yaw,
        f"Link{boundary_index}_Y",
        "0 0 -1",
        "-1.5 1.5",
    )
    _geom(
        connector_yaw,
        f"link_yaw_geom_{boundary_index}",
        "box",
        mass=0.5 * primitive.LINK_YAW_MASS_KG,
        size=" ".join(f"{value:.9g}" for value in primitive.LINK_YAW_HALF_SIZE_M),
        rgba="0.45 0.60 0.86 1",
    )
    next_index = segment_index - 1
    next_segment = ET.SubElement(
        connector_yaw,
        "body",
        {
            "name": f"Chasis{next_index}",
            "pos": f"0 {half_spacing:.9g} 0",
        },
    )
    primitive._add_segment_contents(next_segment, next_index)
    _append_head_branch(next_segment, next_index)


def _add_equality_joint(
    equality: ET.Element,
    name: str,
    joint1: str,
    *,
    joint2: str | None = None,
    polycoef: str,
) -> None:
    attributes = {
        "name": name,
        "joint1": joint1,
        "polycoef": polycoef,
        "solref": "0.0002 1",
        "solimp": "0.9999 0.9999 0.0001",
    }
    if joint2 is not None:
        attributes["joint2"] = joint2
    ET.SubElement(equality, "joint", attributes)


def _split_joint(
    parent: ET.Element,
    name: str,
    axis: str,
    joint_range: str,
) -> ET.Element:
    joint = primitive._add_joint(parent, name, axis, joint_range)
    # q_full = 2*q_half.  Two half joints therefore need twice the damping and
    # armature each to reproduce the original full-coordinate quadratic terms.
    joint.set("damping", f"{2.0 * JOINT_DAMPING:.12g}")
    joint.set("armature", f"{2.0 * JOINT_ARMATURE:.12g}")
    joint.set("frictionloss", f"{JOINT_FRICTIONLOSS:.12g}")
    return joint


def _add_even_center(
    robot_root: ET.Element,
    n_segments: int,
    equality: ET.Element,
) -> int:
    """Add the physical middle connector and its two mirror half-joints."""

    center_boundary = n_segments // 2
    half_spacing = 0.5 * primitive.SEGMENT_SPACING_M

    # Four overlapping half-mass primitives reproduce the same complete
    # measured connector at the geometric center while keeping standard names.
    _geom(
        robot_root,
        f"link_yaw_parent_half_geom_{center_boundary}",
        "box",
        mass=0.5 * primitive.LINK_YAW_MASS_KG,
        size=" ".join(f"{value:.9g}" for value in primitive.LINK_YAW_HALF_SIZE_M),
        rgba="0.34 0.50 0.78 1",
    )
    _geom(
        robot_root,
        f"link_yaw_geom_{center_boundary}",
        "box",
        mass=0.0,
        size=" ".join(f"{value:.9g}" for value in primitive.LINK_YAW_HALF_SIZE_M),
        rgba="0.45 0.60 0.86 1",
    )
    _geom(
        robot_root,
        f"link_pitch_parent_half_geom_{center_boundary}",
        "sphere",
        mass=0.5 * primitive.LINK_PITCH_MASS_KG,
        size=f"{primitive.LINK_PITCH_RADIUS_M:.9g}",
        rgba="0.65 0.72 0.88 1",
    )
    _geom(
        robot_root,
        f"link_pitch_geom_{center_boundary}",
        "sphere",
        mass=0.0,
        size=f"{primitive.LINK_PITCH_RADIUS_M:.9g}",
        rgba="0.72 0.78 0.90 1",
    )

    head_yaw = ET.SubElement(robot_root, "body", {"name": "CenterHeadYaw"})
    _split_joint(head_yaw, "CenterHead_Y", "0 0 1", "-0.75 0.75")
    _geom(
        head_yaw,
        "center_head_yaw_mass_geom",
        "box",
        mass=0.25 * primitive.LINK_YAW_MASS_KG,
        size=" ".join(f"{value:.9g}" for value in primitive.LINK_YAW_HALF_SIZE_M),
        rgba="0.40 0.55 0.82 1",
    )
    head_pitch = ET.SubElement(head_yaw, "body", {"name": "CenterHeadPitch"})
    _split_joint(head_pitch, "CenterHead_P", "1 0 0", "-0.75 0.75")
    _geom(
        head_pitch,
        "center_head_pitch_mass_geom",
        "sphere",
        mass=0.25 * primitive.LINK_PITCH_MASS_KG,
        size=f"{primitive.LINK_PITCH_RADIUS_M:.9g}",
        rgba="0.69 0.75 0.89 1",
    )
    head_segment = ET.SubElement(
        head_pitch,
        "body",
        {"name": f"Chasis{center_boundary}", "pos": f"0 {half_spacing:.9g} 0"},
    )
    primitive._add_segment_contents(head_segment, center_boundary)
    _append_head_branch(head_segment, center_boundary)

    tail_yaw = ET.SubElement(robot_root, "body", {"name": "CenterTailYaw"})
    _split_joint(tail_yaw, "CenterTail_Y", "0 0 1", "-0.75 0.75")
    _geom(
        tail_yaw,
        "center_tail_yaw_mass_geom",
        "box",
        mass=0.25 * primitive.LINK_YAW_MASS_KG,
        size=" ".join(f"{value:.9g}" for value in primitive.LINK_YAW_HALF_SIZE_M),
        rgba="0.40 0.55 0.82 1",
    )
    tail_pitch = ET.SubElement(tail_yaw, "body", {"name": "CenterTailPitch"})
    _split_joint(tail_pitch, "CenterTail_P", "1 0 0", "-0.75 0.75")
    _geom(
        tail_pitch,
        "center_tail_pitch_mass_geom",
        "sphere",
        mass=0.25 * primitive.LINK_PITCH_MASS_KG,
        size=f"{primitive.LINK_PITCH_RADIUS_M:.9g}",
        rgba="0.69 0.75 0.89 1",
    )
    tail_segment_index = center_boundary + 1
    tail_segment = ET.SubElement(
        tail_pitch,
        "body",
        {
            "name": f"Chasis{tail_segment_index}",
            "pos": f"0 {-half_spacing:.9g} 0",
        },
    )
    primitive._add_segment_contents(tail_segment, tail_segment_index)
    primitive._append_chain(tail_segment, tail_segment_index, n_segments)

    _add_equality_joint(
        equality,
        "mirror_center_yaw",
        "CenterHead_Y",
        joint2="CenterTail_Y",
        polycoef="0 -1 0 0 0",
    )
    _add_equality_joint(
        equality,
        "mirror_center_pitch",
        "CenterHead_P",
        joint2="CenterTail_P",
        polycoef="0 -1 0 0 0",
    )
    _add_equality_joint(
        equality,
        "lock_center_pitch",
        "CenterTail_P",
        polycoef="0 0 0 0 0",
    )
    return center_boundary


def _mirror_shells(n_segments: int) -> list[tuple[int, ...]]:
    if n_segments % 2:
        center = (n_segments + 1) // 2
        shells = [(center,)]
        shells.extend((center - depth, center + depth) for depth in range(1, center))
        return shells
    left_center = n_segments // 2
    return [
        (left_center - depth, left_center + 1 + depth)
        for depth in range(left_center)
    ]


def _constraint_geom_order(n_segments: int) -> list[str]:
    names: list[str] = []
    pair_index = 0
    segment_components = (
        "chassis_geom_{i}",
        "shoulder_geom_{i}_R",
        "shoulder_geom_{i}_L",
        "leg_geom_{i}_R",
        "leg_geom_{i}_L",
        "foot_geom_{i}_R",
        "foot_geom_{i}_L",
    )
    for shell in _mirror_shells(n_segments):
        for template in segment_components:
            ordered = list(shell)
            if len(ordered) == 2 and pair_index % 2:
                ordered.reverse()
            names.extend(template.format(i=index) for index in ordered)
            pair_index += 1

    boundary_components = (
        "link_yaw_parent_half_geom_{i}",
        "link_yaw_geom_{i}",
        "link_pitch_parent_half_geom_{i}",
        "link_pitch_geom_{i}",
    )
    unvisited = set(range(1, n_segments))
    while unvisited:
        boundary = min(unvisited, key=lambda value: abs(value - 0.5 * n_segments))
        mirror = n_segments - boundary
        shell = (boundary,) if mirror == boundary else (min(boundary, mirror), max(boundary, mirror))
        unvisited.difference_update(shell)
        for template in boundary_components:
            ordered = list(shell)
            if len(ordered) == 2 and pair_index % 2:
                ordered.reverse()
            names.extend(template.format(i=index) for index in ordered)
            pair_index += 1
    if n_segments % 2 == 0:
        center_mass_pairs = (
            ("center_head_yaw_mass_geom", "center_tail_yaw_mass_geom"),
            ("center_head_pitch_mass_geom", "center_tail_pitch_mass_geom"),
        )
        for pair in center_mass_pairs:
            ordered = list(pair)
            if pair_index % 2:
                ordered.reverse()
            names.extend(ordered)
            pair_index += 1
    return names


def _add_explicit_mirror_contacts(root: ET.Element, n_segments: int) -> dict:
    floor = root.find("worldbody/geom[@name='floor']")
    if floor is None:
        raise KeyError("floor geom is missing")
    floor.set("contype", "1")
    floor.set("conaffinity", "0")

    robot_geoms = {
        geom.get("name")
        for geom in root.iter("geom")
        if geom.get("name") and geom.get("name") != "floor"
    }
    order = _constraint_geom_order(n_segments)
    if set(order) != robot_geoms:
        missing = sorted(robot_geoms - set(order))
        extra = sorted(set(order) - robot_geoms)
        raise RuntimeError(f"contact-order mismatch; missing={missing}, extra={extra}")

    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if name == "floor":
            continue
        if name.startswith(("leg_geom_", "foot_geom_")):
            geom.set("contype", "4")
            geom.set("conaffinity", "4")
        else:
            geom.set("contype", "2")
            geom.set("conaffinity", "0")

    contact = ET.SubElement(root, "contact")
    for pair_id, geom_name in enumerate(order):
        ET.SubElement(
            contact,
            "pair",
            {
                "name": f"ground_pair_{pair_id:03d}",
                "geom1": "floor",
                "geom2": geom_name,
                "condim": "4",
                "friction": "0.4 0.4 0.005 0.0001 0.0001",
                "solref": "0.02 1",
                "solimp": "0.98 0.98 0.001",
            },
        )
    return {
        "explicit_ground_pair_count": len(order),
        "constraint_geom_order": order,
        "mirrored_pairs_adjacent": True,
        "within_pair_first_order_alternates": True,
    }


def _position_actuator(
    actuators: ET.Element,
    name: str,
    joint: str,
    ctrlrange: str,
    *,
    gear: float | None = None,
) -> None:
    attributes = {
        "name": name,
        "joint": joint,
        "kp": "32.7670866",
        "kv": "2.08601752",
        "forcelimited": "true",
        "forcerange": "-0.94 0.94",
        "ctrllimited": "true",
        "ctrlrange": ctrlrange,
    }
    if gear is not None:
        attributes["gear"] = f"{gear:.12g}"
    ET.SubElement(actuators, "position", attributes)


def generate_model(output_path: Path, n_segments: int) -> dict:
    if n_segments < 3:
        raise ValueError("center-rooted null requires at least three segments")
    root = ET.Element("mujoco", {"model": f"centipede_{n_segments}seg_{VARIANT}"})
    ET.SubElement(root, "compiler", {"angle": "radian", "inertiafromgeom": "true"})
    ET.SubElement(
        root,
        "option",
        {
            "timestep": "0.001",
            "gravity": "0 0 -9.81",
            "integrator": "implicitfast",
            "solver": "Newton",
            "iterations": "200",
            "tolerance": "1e-12",
            "cone": "elliptic",
            "noslip_iterations": "50",
            "noslip_tolerance": "1e-12",
        },
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "headlight",
        {
            "ambient": "0.45 0.45 0.45",
            "diffuse": "0.8 0.8 0.8",
            "specular": "0.12 0.12 0.12",
        },
    )
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        {
            "name": "grid",
            "type": "2d",
            "builtin": "checker",
            "width": "512",
            "height": "512",
            "rgb1": "0.22 0.22 0.22",
            "rgb2": "0.88 0.88 0.88",
        },
    )
    ET.SubElement(
        asset,
        "material",
        {
            "name": "grid_mat",
            "texture": "grid",
            "texrepeat": "10 10",
            "reflectance": "0.15",
        },
    )
    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "joint",
        {
            "damping": f"{JOINT_DAMPING}",
            "armature": f"{JOINT_ARMATURE}",
            "frictionloss": f"{JOINT_FRICTIONLOSS}",
        },
    )
    ET.SubElement(
        default,
        "geom",
        {
            "condim": "4",
            "friction": "0.4 0.005 0.0001",
            "solref": "0.02 1",
            "solimp": "0.98 0.98 0.001",
        },
    )

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "size": "5 5 0.1",
            "material": "grid_mat",
            "contype": "1",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {"pos": "0 0 3", "dir": "0 0 -1", "diffuse": "0.8 0.8 0.8"},
    )
    robot_root = ET.SubElement(
        worldbody,
        "body",
        {"name": "robot_root", "pos": f"0 0 {primitive.ROOT_HEIGHT_M:.9g}"},
    )
    ET.SubElement(robot_root, "freejoint", {"name": "root"})
    equality = ET.SubElement(root, "equality")

    center_boundary: int | None = None
    if n_segments % 2:
        center_segment = (n_segments + 1) // 2
        primitive._add_segment_contents(robot_root, center_segment)
        _append_head_branch(robot_root, center_segment)
        primitive._append_chain(robot_root, center_segment, n_segments)
    else:
        center_boundary = _add_even_center(robot_root, n_segments, equality)

    for boundary_index in range(1, n_segments):
        if center_boundary is not None and boundary_index == center_boundary:
            continue
        _add_equality_joint(
            equality,
            f"lock_Link{boundary_index}_P",
            f"Link{boundary_index}_P",
            polycoef="0 0 0 0 0",
        )

    actuators = ET.SubElement(root, "actuator")
    for segment_index in range(1, n_segments + 1):
        _position_actuator(
            actuators,
            f"act_Shoulder{segment_index}_R",
            f"Shoulder{segment_index}_R",
            "-0.57 0.8",
        )
        _position_actuator(
            actuators,
            f"act_Leg{segment_index}_R",
            f"Leg{segment_index}_R",
            "-1.5 1.5",
        )
        _position_actuator(
            actuators,
            f"act_Shoulder{segment_index}_L",
            f"Shoulder{segment_index}_L",
            "-0.8 0.57",
        )
        _position_actuator(
            actuators,
            f"act_Leg{segment_index}_L",
            f"Leg{segment_index}_L",
            "-1.5 1.5",
        )
        if segment_index < n_segments:
            if center_boundary is not None and segment_index == center_boundary:
                _position_actuator(
                    actuators,
                    f"act_Link{segment_index}_Y",
                    "CenterTail_Y",
                    "-0.7 0.7",
                    gear=2.0,
                )
                _position_actuator(
                    actuators,
                    f"act_Link{segment_index}_P",
                    "CenterTail_P",
                    "-1.5 1.5",
                    gear=2.0,
                )
            else:
                _position_actuator(
                    actuators,
                    f"act_Link{segment_index}_Y",
                    f"Link{segment_index}_Y",
                    "-0.7 0.7",
                )
                _position_actuator(
                    actuators,
                    f"act_Link{segment_index}_P",
                    f"Link{segment_index}_P",
                    "-1.5 1.5",
                )

    contact_manifest = _add_explicit_mirror_contacts(root, n_segments)
    ET.indent(root, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    return {
        "n_segments": n_segments,
        "model_path": str(output_path.resolve()),
        "root_type": "middle_segment" if n_segments % 2 else "middle_connector",
        "center_segment": (n_segments + 1) // 2 if n_segments % 2 else None,
        "center_boundary": center_boundary,
        **contact_manifest,
    }


def run(args: argparse.Namespace) -> None:
    output_dir = args.output_dir.resolve()
    rows = []
    for n_segments in sorted(set(args.segments)):
        rows.append(
            generate_model(
                output_dir
                / "models"
                / VARIANT
                / f"centipede_{n_segments}seg_{VARIANT}.xml",
                n_segments,
            )
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "model_manifest.json").write_text(
        json.dumps(
            {
                "variant": VARIANT,
                "models": rows,
                "source_models_modified": False,
                "intersegment_pitch_policy": "equality_locked_zero",
                "contact_policy": (
                    "explicit floor pairs; reflection partners adjacent; "
                    "first order alternates across pairs"
                ),
                "solver_policy": "Newton, elliptic cone, tight tolerance",
            },
            indent=2,
        ),
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
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
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
