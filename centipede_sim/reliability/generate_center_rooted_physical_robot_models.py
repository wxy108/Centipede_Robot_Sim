"""Re-root the measured STL centipede at its physical center.

This is deliberately not a geometric null.  Every source mesh, inertial,
joint anchor, and joint axis is retained.  Only representation choices that
can introduce an artificial head-to-tail polarity are changed:

* odd N is rooted at the middle chassis;
* even N is rooted on the middle physical yaw-motor rigid body, with the
  original yaw edge reversed onto the head branch;
* the serial chain is rebuilt as two branches from the center;
* ground contacts are explicit and center-out paired/alternating;
* body-pitch joints retain their measured compliance and are commanded to zero
  by the controller rather than being replaced by hard constraints;
* the solver settings match the center-rooted dynamic-null audit.

Reversed graph edges use the same physical hinge anchor and the negated world
axis, so the original positive controller coordinate is preserved.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np


VARIANT = "center_rooted_physical_robot"
REFERENCE_VARIANT = "endpoint_optimized_reference"
TAIL_VARIANT = "tail_rooted_physical_reference"
SEGMENTS = tuple(range(4, 11))


@dataclass(frozen=True)
class Edge:
    a: str
    b: str
    original_parent: str
    original_child: str
    joint_name: str | None


def _repo_root_from_script() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def _fmt(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _rotation(mujoco, quat: np.ndarray) -> np.ndarray:
    flat = np.empty(9, dtype=float)
    mujoco.mju_quat2Mat(flat, np.asarray(quat, dtype=float))
    return flat.reshape(3, 3)


def _quat_from_rotation(mujoco, rotation: np.ndarray) -> np.ndarray:
    quat = np.empty(4, dtype=float)
    mujoco.mju_mat2Quat(quat, np.asarray(rotation, dtype=float).reshape(9))
    if quat[0] < 0.0:
        quat *= -1.0
    return quat


def _body_pose(model, data, body_name: str, mujoco) -> tuple[np.ndarray, np.ndarray]:
    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name))
    if body_id < 0:
        raise KeyError(body_name)
    return (
        np.asarray(data.xpos[body_id], dtype=float).copy(),
        _rotation(mujoco, np.asarray(data.xquat[body_id], dtype=float)),
    )


def _relative_pose(
    mujoco,
    parent_pose: tuple[np.ndarray, np.ndarray],
    child_pose: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    parent_pos, parent_rot = parent_pose
    child_pos, child_rot = child_pose
    relative_pos = parent_rot.T @ (child_pos - parent_pos)
    relative_rot = parent_rot.T @ child_rot
    return relative_pos, _quat_from_rotation(mujoco, relative_rot)


def _source_body_name(node: str) -> str:
    family = node[0]
    index = int(node[1:])
    if family == "C":
        return "robot_root" if index == 1 else f"Chasis{index}"
    if family == "Y":
        return f"LinkMotor{index}_PnY"
    if family == "P":
        return f"LinkMotor{index}_R"
    raise ValueError(node)


def _output_body_name(node: str) -> str:
    family = node[0]
    index = int(node[1:])
    if family == "C":
        return f"Chasis{index}"
    return _source_body_name(node)


def _source_body_elements(source_root: ET.Element) -> dict[str, ET.Element]:
    result = {}
    for body in source_root.iter("body"):
        name = body.get("name")
        if name:
            result[name] = body
    return result


def _assign_geom_names(body: ET.Element, n_segments: int) -> None:
    for descendant in body.iter("body"):
        name = descendant.get("name", "")
        geoms = list(descendant.findall("geom"))
        for ordinal, geom in enumerate(geoms):
            suffix = "" if len(geoms) == 1 else f"_{ordinal}"
            if name.startswith("ShoulderMotor"):
                side = name.rsplit("_", 1)[1]
                geom.set("name", f"shoulder_geom_{n_segments}_{side}{suffix}")
            elif name.startswith("Leg"):
                side = name.rsplit("_", 1)[1]
                geom.set("name", f"leg_geom_{n_segments}_{side}{suffix}")


def _copy_direct_payload(
    source_body: ET.Element,
    destination: ET.Element,
    *,
    node: str,
) -> None:
    family = node[0]
    index = int(node[1:])
    geom_ordinal = 0
    for child in source_body:
        if child.tag in {"body", "joint", "freejoint"}:
            continue
        clone = copy.deepcopy(child)
        if clone.tag == "geom":
            if family == "C":
                base_name = f"chassis_geom_{index}"
            elif family == "Y":
                base_name = f"link_yaw_geom_{index}"
            elif family == "P":
                base_name = f"link_pitch_geom_{index}"
            else:
                raise ValueError(node)
            clone.set("name", base_name if geom_ordinal == 0 else f"{base_name}_{geom_ordinal}")
            geom_ordinal += 1
        destination.append(clone)

    if family == "C":
        for child in source_body.findall("body"):
            name = child.get("name", "")
            if not name.startswith("ShoulderMotor"):
                continue
            clone = copy.deepcopy(child)
            _assign_geom_names(clone, index)
            destination.append(clone)


def _make_node_body(
    parent: ET.Element,
    node: str,
    parent_pose: tuple[np.ndarray, np.ndarray],
    poses: dict[str, tuple[np.ndarray, np.ndarray]],
    source_bodies: dict[str, ET.Element],
    mujoco,
) -> ET.Element:
    pos, quat = _relative_pose(mujoco, parent_pose, poses[node])
    body = ET.SubElement(
        parent,
        "body",
        {"name": _output_body_name(node), "pos": _fmt(pos), "quat": _fmt(quat)},
    )
    _copy_direct_payload(
        source_bodies[_source_body_name(node)],
        body,
        node=node,
    )
    return body


def _joint_element(source_bodies: dict[str, ET.Element], edge: Edge) -> ET.Element:
    if edge.joint_name is None:
        raise ValueError("fixed edge has no joint")
    source_child = source_bodies[_source_body_name(edge.original_child)]
    joint = source_child.find(f"joint[@name='{edge.joint_name}']")
    if joint is None:
        raise KeyError(edge.joint_name)
    return joint


def _compiled_joint_geometry(model, data, joint_name: str, mujoco):
    joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name))
    if joint_id < 0:
        raise KeyError(joint_name)
    return (
        joint_id,
        np.asarray(data.xanchor[joint_id], dtype=float).copy(),
        np.asarray(data.xaxis[joint_id], dtype=float).copy(),
    )


def _attach_edge_joint(
    child_body: ET.Element,
    *,
    edge: Edge,
    parent_node: str,
    child_node: str,
    poses: dict[str, tuple[np.ndarray, np.ndarray]],
    source_bodies: dict[str, ET.Element],
    model,
    data,
    mujoco,
) -> None:
    if edge.joint_name is None:
        return
    source_joint = _joint_element(source_bodies, edge)
    clone = copy.deepcopy(source_joint)
    if parent_node == edge.original_parent and child_node == edge.original_child:
        child_body.insert(0, clone)
        return

    _, anchor_world, axis_world = _compiled_joint_geometry(
        model, data, edge.joint_name, mujoco
    )
    child_pos, child_rot = poses[child_node]
    local_anchor = child_rot.T @ (anchor_world - child_pos)
    local_axis = -(child_rot.T @ axis_world)
    clone.set("pos", _fmt(local_anchor))
    clone.set("axis", _fmt(local_axis))
    child_body.insert(0, clone)


def _graph(n_segments: int) -> tuple[dict[str, list[tuple[str, Edge]]], list[Edge]]:
    adjacency: dict[str, list[tuple[str, Edge]]] = {}
    edges: list[Edge] = []

    def add(edge: Edge) -> None:
        edges.append(edge)
        adjacency.setdefault(edge.a, []).append((edge.b, edge))
        adjacency.setdefault(edge.b, []).append((edge.a, edge))

    for index in range(1, n_segments + 1):
        adjacency.setdefault(f"C{index}", [])
    for index in range(1, n_segments):
        add(Edge(f"C{index}", f"Y{index}", f"C{index}", f"Y{index}", f"Link{index}_Y"))
        add(Edge(f"Y{index}", f"P{index}", f"Y{index}", f"P{index}", f"Link{index}_P"))
        add(Edge(f"P{index}", f"C{index + 1}", f"P{index}", f"C{index + 1}", None))
    return adjacency, edges


def _populate_branch(
    parent_element: ET.Element,
    *,
    start_node: str,
    blocked_edge: Edge | None,
    root_pose: tuple[np.ndarray, np.ndarray],
    adjacency: dict[str, list[tuple[str, Edge]]],
    poses: dict[str, tuple[np.ndarray, np.ndarray]],
    source_bodies: dict[str, ET.Element],
    model,
    data,
    mujoco,
    entry_joint: ET.Element | None = None,
) -> None:
    start_body = _make_node_body(
        parent_element, start_node, root_pose, poses, source_bodies, mujoco
    )
    if entry_joint is not None:
        start_body.insert(0, entry_joint)
    queue = deque([(start_node, start_body, None)])
    while queue:
        node, node_element, previous = queue.popleft()
        for neighbor, edge in adjacency[node]:
            if blocked_edge is not None and edge == blocked_edge:
                continue
            if neighbor == previous:
                continue
            child = _make_node_body(
                node_element,
                neighbor,
                poses[node],
                poses,
                source_bodies,
                mujoco,
            )
            _attach_edge_joint(
                child,
                edge=edge,
                parent_node=node,
                child_node=neighbor,
                poses=poses,
                source_bodies=source_bodies,
                model=model,
                data=data,
                mujoco=mujoco,
            )
            queue.append((neighbor, child, node))


def _populate_existing_root(
    root_element: ET.Element,
    *,
    root_node: str,
    adjacency: dict[str, list[tuple[str, Edge]]],
    poses: dict[str, tuple[np.ndarray, np.ndarray]],
    source_bodies: dict[str, ET.Element],
    model,
    data,
    mujoco,
) -> None:
    """Fill a physical root body and expand both graph directions from it."""

    _copy_direct_payload(
        source_bodies[_source_body_name(root_node)],
        root_element,
        node=root_node,
    )
    queue = deque([(root_node, root_element, None)])
    while queue:
        node, node_element, previous = queue.popleft()
        for neighbor, edge in adjacency[node]:
            if neighbor == previous:
                continue
            child = _make_node_body(
                node_element,
                neighbor,
                poses[node],
                poses,
                source_bodies,
                mujoco,
            )
            _attach_edge_joint(
                child,
                edge=edge,
                parent_node=node,
                child_node=neighbor,
                poses=poses,
                source_bodies=source_bodies,
                model=model,
                data=data,
                mujoco=mujoco,
            )
            queue.append((neighbor, child, node))


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
    *,
    name: str,
    child_node: str,
    sign: float,
    source_joint: ET.Element,
    joint_id: int,
    anchor_world: np.ndarray,
    axis_world: np.ndarray,
    poses: dict[str, tuple[np.ndarray, np.ndarray]],
    model,
) -> ET.Element:
    child_pos, child_rot = poses[child_node]
    attributes = copy.deepcopy(source_joint.attrib)
    attributes["name"] = name
    attributes["pos"] = _fmt(child_rot.T @ (anchor_world - child_pos))
    attributes["axis"] = _fmt(sign * (child_rot.T @ axis_world))
    original_range = np.asarray(model.jnt_range[joint_id], dtype=float)
    attributes["range"] = _fmt(0.5 * original_range)
    dof_address = int(model.jnt_dofadr[joint_id])
    attributes["damping"] = f"{2.0 * float(model.dof_damping[dof_address]):.12g}"
    attributes["armature"] = f"{2.0 * float(model.dof_armature[dof_address]):.12g}"
    attributes["frictionloss"] = f"{float(model.dof_frictionloss[dof_address]):.12g}"
    return ET.Element("joint", attributes)


def _shells(n_segments: int) -> list[tuple[int, ...]]:
    if n_segments % 2:
        middle = (n_segments + 1) // 2
        return [(middle,)] + [
            (middle - depth, middle + depth) for depth in range(1, middle)
        ]
    middle = n_segments // 2
    return [
        (middle - depth, middle + 1 + depth) for depth in range(middle)
    ]


def _contact_order(n_segments: int) -> list[str]:
    result: list[str] = []
    pair_index = 0
    templates = (
        "chassis_geom_{i}",
        "shoulder_geom_{i}_R",
        "shoulder_geom_{i}_L",
        "leg_geom_{i}_R",
        "leg_geom_{i}_L",
    )
    for shell in _shells(n_segments):
        for template in templates:
            ordered = list(shell)
            if len(ordered) == 2 and pair_index % 2:
                ordered.reverse()
            result.extend(template.format(i=index) for index in ordered)
            pair_index += 1

    unvisited = set(range(1, n_segments))
    while unvisited:
        boundary = min(unvisited, key=lambda value: abs(value - 0.5 * n_segments))
        mirror = n_segments - boundary
        shell = (
            (boundary,)
            if mirror == boundary
            else (min(boundary, mirror), max(boundary, mirror))
        )
        unvisited.difference_update(shell)
        for template in ("link_yaw_geom_{i}", "link_pitch_geom_{i}"):
            ordered = list(shell)
            if len(ordered) == 2 and pair_index % 2:
                ordered.reverse()
            result.extend(template.format(i=index) for index in ordered)
            pair_index += 1
    return result


def _add_contacts(root: ET.Element, n_segments: int) -> dict:
    floor = root.find("worldbody/geom[@name='floor']")
    if floor is None:
        raise KeyError("floor")
    floor.set("contype", "1")
    floor.set("conaffinity", "0")

    robot_geoms = {}
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if not name or name == "floor":
            continue
        robot_geoms[name] = geom
        if name.startswith("leg_geom_"):
            geom.set("contype", "4")
            geom.set("conaffinity", "4")
        else:
            geom.set("contype", "2")
            geom.set("conaffinity", "0")

    order = _contact_order(n_segments)
    if set(order) != set(robot_geoms):
        raise RuntimeError(
            f"contact geometry mismatch: missing={sorted(set(robot_geoms) - set(order))}, "
            f"extra={sorted(set(order) - set(robot_geoms))}"
        )
    contact = ET.SubElement(root, "contact")
    for ordinal, name in enumerate(order):
        ET.SubElement(
            contact,
            "pair",
            {
                "name": f"ground_pair_{ordinal:03d}",
                "geom1": "floor",
                "geom2": name,
                "condim": "4",
                "friction": "0.4 0.4 0.005 0.0001 0.0001",
                "solref": "0.02 1",
                "solimp": "0.98 0.98 0.001",
            },
        )
    return {
        "explicit_ground_pair_count": len(order),
        "contact_order": order,
        "center_out_paired_order": True,
        "within_pair_first_order_alternates": True,
    }


def _name_source_tree_geometries(root: ET.Element) -> None:
    for body in root.iter("body"):
        body_name = body.get("name", "")
        if body_name == "robot_root":
            base_name = "chassis_geom_1"
        elif body_name.startswith("Chasis"):
            base_name = f"chassis_geom_{int(body_name.removeprefix('Chasis'))}"
        elif body_name.startswith("ShoulderMotor"):
            index_side = body_name.removeprefix("ShoulderMotor")
            index, side = index_side.rsplit("_", 1)
            base_name = f"shoulder_geom_{int(index)}_{side}"
        elif body_name.startswith("Leg"):
            index_side = body_name.removeprefix("Leg")
            index, side = index_side.rsplit("_", 1)
            base_name = f"leg_geom_{int(index)}_{side}"
        elif body_name.startswith("LinkMotor") and body_name.endswith("_PnY"):
            index = int(body_name.removeprefix("LinkMotor").removesuffix("_PnY"))
            base_name = f"link_yaw_geom_{index}"
        elif body_name.startswith("LinkMotor") and body_name.endswith("_R"):
            index = int(body_name.removeprefix("LinkMotor").removesuffix("_R"))
            base_name = f"link_pitch_geom_{index}"
        else:
            continue
        geoms = list(body.findall("geom"))
        for ordinal, geom in enumerate(geoms):
            geom.set("name", base_name if ordinal == 0 else f"{base_name}_{ordinal}")


def generate_endpoint_reference(
    repo_root: Path,
    output_path: Path,
    n_segments: int,
) -> dict:
    """Keep the source tree while matching solver/contact representation."""

    import mujoco

    source_path = repo_root / "centipede_sim" / "model" / f"centipede_{n_segments}seg.xml"
    source_root = ET.parse(source_path).getroot()
    root = copy.deepcopy(source_root)
    root.set("model", f"centipede_{n_segments}seg_{REFERENCE_VARIANT}")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler", {"angle": "radian"})
        root.insert(0, compiler)
    compiler.set("meshdir", (repo_root / "centipede_sim" / "meshes").as_posix())
    existing_option = root.find("option")
    if existing_option is not None:
        root.remove(existing_option)
    option = ET.Element(
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
    root.insert(1, option)
    floor = root.find("worldbody/geom")
    if floor is None:
        raise KeyError("floor")
    floor.set("name", "floor")
    _name_source_tree_geometries(root)
    existing_contact = root.find("contact")
    if existing_contact is not None:
        root.remove(existing_contact)
    contact_manifest = _add_contacts(root, n_segments)
    ET.indent(root, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    source = mujoco.MjModel.from_xml_path(str(source_path))
    rebuilt = mujoco.MjModel.from_xml_path(str(output_path))
    return {
        "n_segments": n_segments,
        "source_model": str(source_path.resolve()),
        "model_path": str(output_path.resolve()),
        "source_mass_kg": float(source.body_subtreemass[1]),
        "rebuilt_mass_kg": float(rebuilt.body_subtreemass[1]),
        "mass_error_kg": float(rebuilt.body_subtreemass[1] - source.body_subtreemass[1]),
        "source_actuator_count": int(source.nu),
        "rebuilt_actuator_count": int(rebuilt.nu),
        **contact_manifest,
    }


def _copy_top_level(source_root: ET.Element, output_root: ET.Element, repo_root: Path) -> None:
    compiler = copy.deepcopy(source_root.find("compiler"))
    if compiler is None:
        compiler = ET.Element("compiler", {"angle": "radian"})
    compiler.set("meshdir", (repo_root / "centipede_sim" / "meshes").as_posix())
    output_root.append(compiler)
    ET.SubElement(
        output_root,
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
    for tag in ("visual", "asset", "default"):
        element = source_root.find(tag)
        if element is not None:
            output_root.append(copy.deepcopy(element))


def _copy_actuators_and_sensors(
    source_root: ET.Element,
    output_root: ET.Element,
    *,
    root_source_body_name: str,
) -> None:
    actuator = copy.deepcopy(source_root.find("actuator"))
    if actuator is not None:
        output_root.append(actuator)

    sensor = copy.deepcopy(source_root.find("sensor"))
    if sensor is not None:
        for element in sensor:
            if element.get("objname") == "robot_root":
                element.set("objname", "Chasis1")
            if element.get("objname") == root_source_body_name:
                element.set("objname", "robot_root")
        output_root.append(sensor)


def generate_model(
    repo_root: Path,
    output_path: Path,
    n_segments: int,
    *,
    variant: str = VARIANT,
    root_node_override: str | None = None,
) -> dict:
    import mujoco

    source_path = repo_root / "centipede_sim" / "model" / f"centipede_{n_segments}seg.xml"
    source_root = ET.parse(source_path).getroot()
    source_bodies = _source_body_elements(source_root)
    source_model = mujoco.MjModel.from_xml_path(str(source_path))
    source_data = mujoco.MjData(source_model)
    mujoco.mj_forward(source_model, source_data)

    adjacency, edges = _graph(n_segments)
    nodes = sorted(adjacency)
    poses = {
        node: _body_pose(
            source_model,
            source_data,
            _source_body_name(node),
            mujoco,
        )
        for node in nodes
    }

    output_root = ET.Element(
        "mujoco", {"model": f"centipede_{n_segments}seg_{variant}"}
    )
    _copy_top_level(source_root, output_root, repo_root)
    worldbody = ET.SubElement(output_root, "worldbody")
    source_worldbody = source_root.find("worldbody")
    if source_worldbody is None:
        raise KeyError("worldbody")
    for child in source_worldbody:
        if child.tag != "body":
            worldbody.append(copy.deepcopy(child))
    floor = worldbody.find("geom")
    if floor is None:
        raise KeyError("floor geom")
    floor.set("name", "floor")

    if root_node_override is not None:
        root_node = root_node_override
        root_kind = f"explicit_{root_node_override}"
    elif n_segments % 2:
        center_segment = (n_segments + 1) // 2
        root_node = f"C{center_segment}"
        root_kind = "middle_chassis"
    else:
        center_boundary = n_segments // 2
        root_node = f"Y{center_boundary}"
        root_kind = "middle_yaw_motor_body"

    source_center_pos, source_center_rot = poses[root_node]
    robot_root = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "robot_root",
            "pos": _fmt(np.asarray([0.0, 0.0, source_center_pos[2]], dtype=float)),
            "quat": _fmt(_quat_from_rotation(mujoco, source_center_rot)),
        },
    )
    ET.SubElement(robot_root, "freejoint", {"name": "root"})
    _populate_existing_root(
        robot_root,
        root_node=root_node,
        adjacency=adjacency,
        poses=poses,
        source_bodies=source_bodies,
        model=source_model,
        data=source_data,
        mujoco=mujoco,
    )

    _copy_actuators_and_sensors(
        source_root,
        output_root,
        root_source_body_name=_source_body_name(root_node),
    )
    contact_manifest = _add_contacts(output_root, n_segments)
    ET.indent(output_root, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(output_root).write(
        output_path, encoding="utf-8", xml_declaration=True
    )

    rebuilt = mujoco.MjModel.from_xml_path(str(output_path))
    source_mass = float(source_model.body_subtreemass[1])
    rebuilt_root_id = int(
        mujoco.mj_name2id(rebuilt, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
    )
    rebuilt_mass = float(rebuilt.body_subtreemass[rebuilt_root_id])
    return {
        "n_segments": n_segments,
        "source_model": str(source_path.resolve()),
        "model_path": str(output_path.resolve()),
        "root_kind": root_kind,
        "source_mass_kg": source_mass,
        "rebuilt_mass_kg": rebuilt_mass,
        "mass_error_kg": rebuilt_mass - source_mass,
        "source_actuator_count": int(source_model.nu),
        "rebuilt_actuator_count": int(rebuilt.nu),
        "source_geom_count": int(source_model.ngeom - 1),
        "rebuilt_geom_count": int(rebuilt.ngeom - 1),
        "source_body_count": int(source_model.nbody),
        "rebuilt_body_count": int(rebuilt.nbody),
        **contact_manifest,
    }


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    rows = []
    reference_rows = []
    tail_rows = []
    for n_segments in sorted(set(args.segments)):
        output_path = (
            output_dir
            / "models"
            / VARIANT
            / f"centipede_{n_segments}seg_{VARIANT}.xml"
        )
        rows.append(generate_model(repo_root, output_path, n_segments))
        reference_path = (
            output_dir
            / "models"
            / REFERENCE_VARIANT
            / f"centipede_{n_segments}seg_{REFERENCE_VARIANT}.xml"
        )
        reference_rows.append(
            generate_endpoint_reference(repo_root, reference_path, n_segments)
        )
        tail_path = (
            output_dir
            / "models"
            / TAIL_VARIANT
            / f"centipede_{n_segments}seg_{TAIL_VARIANT}.xml"
        )
        tail_rows.append(
            generate_model(
                repo_root,
                tail_path,
                n_segments,
                variant=TAIL_VARIANT,
                root_node_override=f"C{n_segments}",
            )
        )
        print(f"generated N={n_segments}: {output_path}", flush=True)
    manifest = {
        "variant": VARIANT,
        "preserves_source_stl_and_inertials": True,
        "does_not_project_fore_aft_geometry": True,
        "models": rows,
        "endpoint_optimized_reference_models": reference_rows,
        "tail_rooted_physical_reference_models": tail_rows,
    }
    (output_dir / "model_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )


def build_parser() -> argparse.ArgumentParser:
    repo_root = _repo_root_from_script()
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
    parser.add_argument("--segments", type=int, nargs="+", default=list(SEGMENTS))
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
