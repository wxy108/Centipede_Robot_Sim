"""Generate physical primitive null models for the Centipede Robot.

Two deliberately separate model families are generated:

1. ``chassis_centered`` keeps the source kinematic tree, shoulders, legs, and
   inter-segment links, but replaces every chassis STL/inertial with a
   centered sphere of the same measured mass.  This is a chassis-only
   ablation, not a complete null robot.
2. ``full_primitive_null`` is a new physical robot made only from centered,
   mirror-symmetric primitives.  It preserves the measured component masses,
   joint/actuator names and ordering, controller limits, and motor parameters.

The source model files are never modified.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


CHASSIS_MASS_KG = 0.083
SHOULDER_MASS_KG = 0.0982
LEG_STRUCTURE_MASS_KG = 0.0458
FOOT_MASS_KG = 0.023
LINK_YAW_MASS_KG = 0.0982
LINK_PITCH_MASS_KG = 0.019

SEGMENT_SPACING_M = 0.160
CHASSIS_HALF_SIZE_M = (0.038, 0.052, 0.016)
CHASSIS_SPHERE_RADIUS_M = 0.035
SHOULDER_X_M = 0.052
SHOULDER_RADIUS_M = 0.014
SHOULDER_HALF_LENGTH_M = 0.018
LEG_ENDPOINT_X_M = 0.075
LEG_ENDPOINT_Z_M = -0.060
LEG_RADIUS_M = 0.008
FOOT_RADIUS_M = 0.015
LINK_YAW_HALF_SIZE_M = (0.030, 0.018, 0.014)
LINK_PITCH_RADIUS_M = 0.018
ROOT_HEIGHT_M = 0.088


def _repo_root_from_script() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def _indent(element: ET.Element) -> None:
    ET.indent(element, space="  ")


def _site_position(chassis_body: ET.Element, segment_index: int) -> str:
    site = chassis_body.find(f"site[@name='chasis{segment_index}_site']")
    if site is None:
        raise KeyError(f"chasis{segment_index}_site is missing")
    return site.get("pos", "0 0 0")


def _chassis_bodies(root: ET.Element, n_segments: int) -> list[ET.Element]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise KeyError("worldbody is missing")
    robot_root = worldbody.find("body[@name='robot_root']")
    if robot_root is None:
        raise KeyError("robot_root is missing")
    bodies = [robot_root]
    for segment_index in range(2, n_segments + 1):
        body = root.find(f".//body[@name='Chasis{segment_index}']")
        if body is None:
            raise KeyError(f"Chasis{segment_index} is missing")
        bodies.append(body)
    return bodies


def generate_chassis_centered(source_path: Path, output_path: Path, n_segments: int) -> None:
    tree = ET.parse(source_path)
    root = tree.getroot()
    root.set("model", f"centipede_{n_segments}seg_chassis_centered")
    compiler = root.find("compiler")
    if compiler is None:
        raise KeyError("compiler is missing")
    compiler.set("meshdir", str((source_path.parent.parent / "meshes").resolve()))
    sphere_inertia = 0.4 * CHASSIS_MASS_KG * CHASSIS_SPHERE_RADIUS_M**2
    for segment_index, body in enumerate(_chassis_bodies(root, n_segments), start=1):
        center = _site_position(body, segment_index)
        inertial = body.find("inertial")
        if inertial is None:
            raise KeyError(f"chassis {segment_index} inertial is missing")
        inertial.attrib.clear()
        inertial.set("pos", center)
        inertial.set("mass", f"{CHASSIS_MASS_KG:.9g}")
        inertial.set(
            "diaginertia",
            f"{sphere_inertia:.12g} {sphere_inertia:.12g} {sphere_inertia:.12g}",
        )
        geom = body.find("geom")
        if geom is None:
            raise KeyError(f"chassis {segment_index} geom is missing")
        geom.attrib.clear()
        geom.set("name", f"chassis_primitive_{segment_index}")
        geom.set("type", "sphere")
        geom.set("size", f"{CHASSIS_SPHERE_RADIUS_M:.9g}")
        geom.set("pos", center)
        geom.set("mass", "0")
        geom.set("rgba", "0.93 0.55 0.16 1")
        geom.set("contype", "2")
        geom.set("conaffinity", "0")
    _indent(root)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def _add_joint(
    body: ET.Element,
    name: str,
    axis: str,
    joint_range: str,
    *,
    reference: float | None = None,
) -> ET.Element:
    attributes = {
        "name": name,
        "axis": axis,
        "range": joint_range,
        "actuatorfrcrange": "-1.5 1.5",
    }
    if reference is not None:
        attributes["ref"] = f"{reference:.12g}"
    return ET.SubElement(
        body,
        "joint",
        attributes,
    )


def _add_leg(segment_body: ET.Element, segment_index: int, side: str) -> None:
    sign = 1.0 if side == "R" else -1.0
    shoulder = ET.SubElement(
        segment_body,
        "body",
        {
            "name": f"ShoulderMotor{segment_index}_{side}",
            "pos": f"{sign * SHOULDER_X_M:.9g} 0 0",
        },
    )
    neutral_command = math.radians(-5.0 if side == "R" else 5.0)
    _add_joint(
        shoulder,
        f"Shoulder{segment_index}_{side}",
        "0 0 1",
        "-1.5 1.5",
        reference=neutral_command,
    )
    ET.SubElement(
        shoulder,
        "geom",
        {
            "name": f"shoulder_geom_{segment_index}_{side}",
            "type": "cylinder",
            "size": f"{SHOULDER_RADIUS_M:.9g} {SHOULDER_HALF_LENGTH_M:.9g}",
            "quat": "0.707106781187 0 0.707106781187 0",
            "mass": f"{SHOULDER_MASS_KG:.9g}",
            "rgba": "0.45 0.60 0.86 1",
            "contype": "2",
            "conaffinity": "0",
        },
    )
    leg = ET.SubElement(shoulder, "body", {"name": f"Leg{segment_index}_{side}"})
    pitch_axis = "0 1 0" if side == "R" else "0 -1 0"
    _add_joint(leg, f"Leg{segment_index}_{side}", pitch_axis, "-1.5 1.5")
    endpoint = f"{sign * LEG_ENDPOINT_X_M:.9g} 0 {LEG_ENDPOINT_Z_M:.9g}"
    ET.SubElement(
        leg,
        "geom",
        {
            "name": f"leg_geom_{segment_index}_{side}",
            "type": "capsule",
            "fromto": f"0 0 0 {endpoint}",
            "size": f"{LEG_RADIUS_M:.9g}",
            "mass": f"{LEG_STRUCTURE_MASS_KG:.9g}",
            "rgba": "0.92 0.92 0.95 1",
            "contype": "4",
            "conaffinity": "5",
        },
    )
    ET.SubElement(
        leg,
        "geom",
        {
            "name": f"foot_geom_{segment_index}_{side}",
            "type": "sphere",
            "pos": endpoint,
            "size": f"{FOOT_RADIUS_M:.9g}",
            "mass": f"{FOOT_MASS_KG:.9g}",
            "rgba": "0.95 0.95 0.98 1",
            "contype": "4",
            "conaffinity": "5",
        },
    )
    ET.SubElement(
        leg,
        "site",
        {
            "name": f"foot{segment_index}_{side}",
            "pos": endpoint,
            "size": "0.004",
            "rgba": "0.1 0.8 0.2 0.5",
        },
    )


def _add_segment_contents(segment_body: ET.Element, segment_index: int) -> None:
    ET.SubElement(
        segment_body,
        "geom",
        {
            "name": f"chassis_geom_{segment_index}",
            "type": "box",
            "size": " ".join(f"{value:.9g}" for value in CHASSIS_HALF_SIZE_M),
            "mass": f"{CHASSIS_MASS_KG:.9g}",
            "rgba": "0.93 0.55 0.16 1",
            "contype": "2",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        segment_body,
        "site",
        {
            "name": f"chasis{segment_index}_site",
            "pos": "0 0 0",
            "size": "0.003",
        },
    )
    if segment_index == 1:
        ET.SubElement(
            segment_body,
            "site",
            {"name": "imu_site", "pos": "0 0 0.012", "size": "0.003"},
        )
    _add_leg(segment_body, segment_index, "R")
    _add_leg(segment_body, segment_index, "L")


def _append_chain(parent_segment: ET.Element, segment_index: int, n_segments: int) -> None:
    """Append the connector after ``segment_index`` and recurse."""

    if segment_index >= n_segments:
        return
    joint_position = f"0 {-0.5 * SEGMENT_SPACING_M:.9g} 0"
    # A true head-tail null cannot attach the entire connector inertia to the
    # child side of a joint.  Half of each measured link component is rigidly
    # attached to the parent and half to the child.  The two halves occupy the
    # same joint-centered location, so static mass and dynamic parent/child
    # assignments are both reflection symmetric.
    ET.SubElement(
        parent_segment,
        "geom",
        {
            "name": f"link_yaw_parent_half_geom_{segment_index}",
            "type": "box",
            "pos": joint_position,
            "size": " ".join(f"{value:.9g}" for value in LINK_YAW_HALF_SIZE_M),
            "mass": f"{0.5 * LINK_YAW_MASS_KG:.9g}",
            "rgba": "0.34 0.50 0.78 1",
            "contype": "2",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        parent_segment,
        "geom",
        {
            "name": f"link_pitch_parent_half_geom_{segment_index}",
            "type": "sphere",
            "pos": joint_position,
            "size": f"{LINK_PITCH_RADIUS_M:.9g}",
            "mass": f"{0.5 * LINK_PITCH_MASS_KG:.9g}",
            "rgba": "0.65 0.72 0.88 1",
            "contype": "2",
            "conaffinity": "0",
        },
    )
    connector_yaw = ET.SubElement(
        parent_segment,
        "body",
        {
            "name": f"LinkMotor{segment_index}_PnY",
            "pos": joint_position,
        },
    )
    _add_joint(connector_yaw, f"Link{segment_index}_Y", "0 0 1", "-1.5 1.5")
    ET.SubElement(
        connector_yaw,
        "geom",
        {
            "name": f"link_yaw_geom_{segment_index}",
            "type": "box",
            "size": " ".join(f"{value:.9g}" for value in LINK_YAW_HALF_SIZE_M),
            "mass": f"{0.5 * LINK_YAW_MASS_KG:.9g}",
            "rgba": "0.45 0.60 0.86 1",
            "contype": "2",
            "conaffinity": "0",
        },
    )
    connector_pitch = ET.SubElement(
        connector_yaw,
        "body",
        {"name": f"LinkMotor{segment_index}_R"},
    )
    _add_joint(connector_pitch, f"Link{segment_index}_P", "1 0 0", "-1.5 1.5")
    ET.SubElement(
        connector_pitch,
        "geom",
        {
            "name": f"link_pitch_geom_{segment_index}",
            "type": "sphere",
            "size": f"{LINK_PITCH_RADIUS_M:.9g}",
            "mass": f"{0.5 * LINK_PITCH_MASS_KG:.9g}",
            "rgba": "0.72 0.78 0.90 1",
            "contype": "2",
            "conaffinity": "0",
        },
    )
    next_index = segment_index + 1
    next_segment = ET.SubElement(
        connector_pitch,
        "body",
        {
            "name": f"Chasis{next_index}",
            "pos": f"0 {-0.5 * SEGMENT_SPACING_M:.9g} 0",
        },
    )
    _add_segment_contents(next_segment, next_index)
    _append_chain(next_segment, next_index, n_segments)


def _actuator(parent: ET.Element, name: str, joint: str, ctrlrange: str) -> None:
    ET.SubElement(
        parent,
        "position",
        {
            "name": name,
            "joint": joint,
            "kp": "32.7670866",
            "kv": "2.08601752",
            "forcelimited": "true",
            "forcerange": "-0.94 0.94",
            "ctrllimited": "true",
            "ctrlrange": ctrlrange,
        },
    )


def generate_full_primitive_null(output_path: Path, n_segments: int) -> None:
    root = ET.Element("mujoco", {"model": f"centipede_{n_segments}seg_full_primitive_null"})
    ET.SubElement(root, "compiler", {"angle": "radian", "inertiafromgeom": "true"})
    ET.SubElement(root, "option", {"timestep": "0.001", "gravity": "0 0 -9.81", "integrator": "implicitfast"})
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "headlight",
        {"ambient": "0.45 0.45 0.45", "diffuse": "0.8 0.8 0.8", "specular": "0.12 0.12 0.12"},
    )
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        {"name": "grid", "type": "2d", "builtin": "checker", "width": "512", "height": "512", "rgb1": "0.22 0.22 0.22", "rgb2": "0.88 0.88 0.88"},
    )
    ET.SubElement(
        asset,
        "material",
        {"name": "grid_mat", "texture": "grid", "texrepeat": "10 10", "reflectance": "0.15"},
    )
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", {"damping": "0.0013", "armature": "0.0083", "frictionloss": "0.069"})
    ET.SubElement(default, "geom", {"condim": "4", "friction": "0.4 0.005 0.0001", "solref": "0.02 1", "solimp": "0.98 0.98 0.001"})

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {"name": "floor", "type": "plane", "size": "5 5 0.1", "material": "grid_mat", "contype": "1", "conaffinity": "6"},
    )
    ET.SubElement(worldbody, "light", {"pos": "0 0 3", "dir": "0 0 -1", "diffuse": "0.8 0.8 0.8"})
    center_y = 0.5 * (n_segments - 1) * SEGMENT_SPACING_M
    robot_root = ET.SubElement(
        worldbody,
        "body",
        {"name": "robot_root", "pos": f"0 {center_y:.9g} {ROOT_HEIGHT_M:.9g}"},
    )
    ET.SubElement(robot_root, "freejoint", {"name": "root"})
    _add_segment_contents(robot_root, 1)
    _append_chain(robot_root, 1, n_segments)

    actuators = ET.SubElement(root, "actuator")
    for segment_index in range(1, n_segments + 1):
        _actuator(actuators, f"act_Shoulder{segment_index}_R", f"Shoulder{segment_index}_R", "-0.57 0.8")
        _actuator(actuators, f"act_Leg{segment_index}_R", f"Leg{segment_index}_R", "-1.5 1.5")
        _actuator(actuators, f"act_Shoulder{segment_index}_L", f"Shoulder{segment_index}_L", "-0.8 0.57")
        _actuator(actuators, f"act_Leg{segment_index}_L", f"Leg{segment_index}_L", "-1.5 1.5")
        if segment_index < n_segments:
            _actuator(actuators, f"act_Link{segment_index}_Y", f"Link{segment_index}_Y", "-0.7 0.7")
            _actuator(actuators, f"act_Link{segment_index}_P", f"Link{segment_index}_P", "-1.5 1.5")

    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "accelerometer", {"name": "imu_acc", "site": "imu_site"})
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu_site"})
    ET.SubElement(sensor, "framepos", {"name": "pos_head", "objtype": "body", "objname": "robot_root"})
    ET.SubElement(sensor, "framequat", {"name": "quat_head", "objtype": "body", "objname": "robot_root"})
    for segment_index in range(2, n_segments + 1):
        ET.SubElement(sensor, "framepos", {"name": f"pos_chasis{segment_index}", "objtype": "body", "objname": f"Chasis{segment_index}"})
        ET.SubElement(sensor, "framequat", {"name": f"quat_chasis{segment_index}", "objtype": "body", "objname": f"Chasis{segment_index}"})

    _indent(root)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    for n_segments in sorted(set(args.segments)):
        source_path = repo_root / "centipede_sim" / "model" / f"centipede_{n_segments}seg.xml"
        generate_chassis_centered(
            source_path,
            output_dir / "models" / "chassis_centered" / f"centipede_{n_segments}seg_chassis_centered.xml",
            n_segments,
        )
        generate_full_primitive_null(
            output_dir / "models" / "full_primitive_null" / f"centipede_{n_segments}seg_full_primitive_null.xml",
            n_segments,
        )
    manifest = {
        "source_models_modified": False,
        "model_families": {
            "chassis_centered": (
                "Source kinematic tree with every chassis STL/inertial replaced "
                "by a centered sphere of equal measured chassis mass."
            ),
            "full_primitive_null": (
                "Independent procedural MJCF using only centered symmetric "
                "primitives; link masses are split equally across each joint's "
                "parent and child sides."
            ),
        },
        "measured_component_masses_kg": {
            "chassis_per_segment": CHASSIS_MASS_KG,
            "shoulder_per_side_per_segment": SHOULDER_MASS_KG,
            "leg_structure_per_side_per_segment": LEG_STRUCTURE_MASS_KG,
            "foot_per_side_per_segment": FOOT_MASS_KG,
            "yaw_link_per_intersegment_joint": LINK_YAW_MASS_KG,
            "pitch_link_per_intersegment_joint": LINK_PITCH_MASS_KG,
        },
        "full_null_geometry_m": {
            "segment_spacing": SEGMENT_SPACING_M,
            "chassis_box_half_size": CHASSIS_HALF_SIZE_M,
            "shoulder_x": SHOULDER_X_M,
            "shoulder_radius": SHOULDER_RADIUS_M,
            "shoulder_half_length": SHOULDER_HALF_LENGTH_M,
            "leg_endpoint_x_magnitude": LEG_ENDPOINT_X_M,
            "leg_endpoint_z": LEG_ENDPOINT_Z_M,
            "leg_radius": LEG_RADIUS_M,
            "foot_radius": FOOT_RADIUS_M,
            "yaw_link_box_half_size": LINK_YAW_HALF_SIZE_M,
            "pitch_link_radius": LINK_PITCH_RADIUS_M,
        },
        "mass_formula_kg": "0.417*N + 0.1172*(N-1)",
        "inertia_policy": "MuJoCo computes inertia from centered primitive geoms.",
        "joint_and_actuator_policy": (
            "Names, order, control limits, force limits, kp, kv, damping, "
            "armature, and frictionloss match the source controller interface."
        ),
        "leg_yaw_reference_policy": (
            "Right -5 deg and left +5 deg locked commands are defined as the "
            "physical lateral-neutral pose in the full null."
        ),
        "link_mass_split": {
            "yaw_parent_fraction": 0.5,
            "yaw_child_fraction": 0.5,
            "pitch_parent_fraction": 0.5,
            "pitch_child_fraction": 0.5,
        },
        "segments_generated": sorted(set(args.segments)),
    }
    (output_dir / "physical_null_model_definition.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )


def build_parser() -> argparse.ArgumentParser:
    repo_root = _repo_root_from_script()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "exp_data" / "reliability" / "step24_physical_primitive_null",
    )
    parser.add_argument("--segments", type=int, nargs="+", default=list(range(4, 11)))
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
