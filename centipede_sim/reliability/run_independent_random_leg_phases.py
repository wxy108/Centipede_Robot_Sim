"""Independent random swing/stance experiment for the modular centipede robot.

The body travelling wave is kept identical to the canonical clean gait.  The
original body-to-leg and leg-to-leg phase coupling is removed.  Each physical
leg owns an independent alternating renewal clock:

    stance -> swing -> stance -> ...

Every dwell duration is sampled independently from
``Uniform(0.5 * T_half, 1.5 * T_half)``, where ``T_half = pi / omega``.  Thus
the expected duty factor remains 0.5 and the expected full gait period remains
``2*pi/omega``, but neither left/right legs nor homologous legs share a phase.

The primary metric is signed whole-robot COM displacement along model +y over
a fixed post-transition evaluation horizon.  Planar displacement, lateral
drift, stability gates, state/contact statistics, trajectories, figures, and
representative MP4 demos are retained as supporting evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


OMEGA_BODY = 4.0
PHI_BODY = -math.pi
YAW_HALF_AMPLITUDE_DEG = 15.0
SHOULDER_HALF_AMPLITUDE_DEG = 30.0
LEG_UP_DEG = -40.0
LEG_DOWN_DEG = 5.0
LEG_OFFSET_DEG = -5.0
DUTY_FACTOR = 0.5
DWELL_LOW_FACTOR = 0.5
DWELL_HIGH_FACTOR = 1.5
SETTLE_TIME_S = 1.5
BRIDGE_TIME_S = 0.4
DEFAULT_SEGMENTS = (4, 6, 8)
DEFAULT_REPLAYS = 24
DEFAULT_EVALUATION_CYCLES = 12
DEFAULT_SAMPLE_HZ = 100.0
RANDOM_MODES = ("independent_random", "random_bilateral_complement")


def _mode_label(mode: str) -> str:
    return {
        "independent_random": "Independent random leg phases",
        "random_bilateral_complement": "Random bilateral-complement leg phases",
        "regular_coupled": "Regular coupled gait",
    }.get(mode, mode.replace("_", " ").title())


def _repo_root_from_script() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def _install_project_imports(repo_root: Path):
    scripts_dir = repo_root / "centipede_sim" / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    from test_models import build_actuator_indices, compute_ctrl

    return build_actuator_indices, compute_ctrl


@dataclass
class _LegClock:
    """One leg's independent alternating renewal state."""

    stance: bool
    duration_s: float
    elapsed_s: float
    transition_count: int = 0

    @property
    def progress(self) -> float:
        return min(1.0, max(0.0, self.elapsed_s / self.duration_s))


class IndependentRandomLegController:
    """Fixed body wave plus independent random swing/stance clocks."""

    def __init__(
        self,
        n_segments: int,
        n_ctrl: int,
        indices,
        *,
        seed: int,
        omega_body: float = OMEGA_BODY,
        phi_body: float = PHI_BODY,
        dwell_low_factor: float = DWELL_LOW_FACTOR,
        dwell_high_factor: float = DWELL_HIGH_FACTOR,
    ) -> None:
        if n_segments <= 0:
            raise ValueError("n_segments must be positive")
        if not 0.0 < dwell_low_factor <= dwell_high_factor:
            raise ValueError("invalid dwell-duration factors")
        self.n_segments = int(n_segments)
        self.n_ctrl = int(n_ctrl)
        self.indices = indices
        self.seed = int(seed)
        self.omega_body = float(omega_body)
        self.phi_body = float(phi_body)
        self.half_period_s = math.pi / self.omega_body
        self.dwell_low_s = dwell_low_factor * self.half_period_s
        self.dwell_high_s = dwell_high_factor * self.half_period_s
        self._rngs: list[np.random.Generator] = []
        self._clocks: list[_LegClock] = []

        # Counter-derived per-leg streams make schedules independent while
        # preserving homologous schedules when extra posterior segments are
        # added in N=6 and N=8 models.
        for segment_index in range(self.n_segments):
            for side_index in range(2):  # R=0, L=1
                rng = np.random.default_rng(
                    np.random.SeedSequence([self.seed, segment_index, side_index])
                )
                duration = self._sample_duration(rng)
                stance = bool(rng.integers(0, 2))
                elapsed = float(rng.uniform(0.0, duration))
                self._rngs.append(rng)
                self._clocks.append(_LegClock(stance, duration, elapsed))

    def _sample_duration(self, rng: np.random.Generator) -> float:
        return float(rng.uniform(self.dwell_low_s, self.dwell_high_s))

    @staticmethod
    def _flat_index(segment_index: int, side_index: int) -> int:
        return 2 * segment_index + side_index

    def advance(self, dt: float) -> None:
        if dt < 0.0:
            raise ValueError("dt must be nonnegative")
        for flat_index, (clock, rng) in enumerate(zip(self._clocks, self._rngs)):
            clock.elapsed_s += dt
            while clock.elapsed_s >= clock.duration_s:
                clock.elapsed_s -= clock.duration_s
                clock.stance = not clock.stance
                clock.duration_s = self._sample_duration(rng)
                clock.transition_count += 1

    def stance_flags(self) -> np.ndarray:
        return np.asarray(
            [clock.stance for clock in self._clocks], dtype=bool
        ).reshape(self.n_segments, 2)

    def state_progress(self) -> np.ndarray:
        return np.asarray(
            [clock.progress for clock in self._clocks], dtype=float
        ).reshape(self.n_segments, 2)

    def transition_counts(self) -> np.ndarray:
        return np.asarray(
            [clock.transition_count for clock in self._clocks], dtype=int
        ).reshape(self.n_segments, 2)

    def control(self, body_time_s: float) -> np.ndarray:
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = self.indices
        ctrl = np.zeros(self.n_ctrl, dtype=float)
        yaw_amplitude = math.radians(YAW_HALF_AMPLITUDE_DEG)
        shoulder_amplitude = math.radians(SHOULDER_HALF_AMPLITUDE_DEG)
        leg_up = math.radians(LEG_UP_DEG)
        leg_down = math.radians(LEG_DOWN_DEG)
        leg_offset = math.radians(LEG_OFFSET_DEG)

        # This is exactly the canonical body travelling wave.  No random
        # variable is allowed to enter these body-joint commands.
        for segment_index, actuator_index in enumerate(link_y):
            ctrl[actuator_index] = yaw_amplitude * math.sin(
                self.omega_body * body_time_s
                + segment_index * self.phi_body
            )
        for actuator_index in link_p:
            ctrl[actuator_index] = 0.0

        for segment_index in range(self.n_segments):
            for side_index, side in enumerate(("R", "L")):
                clock = self._clocks[self._flat_index(segment_index, side_index)]
                # Map each random dwell smoothly onto one half of a local leg
                # cycle.  The shoulder therefore performs a power stroke in
                # stance and a recovery stroke in swing, without a shared phase.
                theta = math.pi * clock.progress
                if not clock.stance:
                    theta += math.pi
                shoulder_base = leg_offset if side == "R" else -leg_offset
                shoulder_target = shoulder_base + shoulder_amplitude * math.cos(theta)
                leg_target = leg_down if clock.stance else leg_up
                if side == "R":
                    ctrl[shoulder_r[segment_index]] = shoulder_target
                    ctrl[leg_r[segment_index]] = leg_target
                else:
                    ctrl[shoulder_l[segment_index]] = shoulder_target
                    ctrl[leg_l[segment_index]] = leg_target
        return ctrl


class RandomBilateralComplementController:
    """One independent random side-selection clock per segment.

    At every instant exactly one leg in each segment pair is commanded to
    stance and the contralateral leg is commanded to swing.  A clock transition
    swaps the two sides.  Segment clocks and dwell durations are independent.
    """

    def __init__(
        self,
        n_segments: int,
        n_ctrl: int,
        indices,
        *,
        seed: int,
        omega_body: float = OMEGA_BODY,
        phi_body: float = PHI_BODY,
        dwell_low_factor: float = DWELL_LOW_FACTOR,
        dwell_high_factor: float = DWELL_HIGH_FACTOR,
    ) -> None:
        if n_segments <= 0:
            raise ValueError("n_segments must be positive")
        if not 0.0 < dwell_low_factor <= dwell_high_factor:
            raise ValueError("invalid dwell-duration factors")
        self.n_segments = int(n_segments)
        self.n_ctrl = int(n_ctrl)
        self.indices = indices
        self.seed = int(seed)
        self.omega_body = float(omega_body)
        self.phi_body = float(phi_body)
        self.half_period_s = math.pi / self.omega_body
        self.dwell_low_s = dwell_low_factor * self.half_period_s
        self.dwell_high_s = dwell_high_factor * self.half_period_s
        self._rngs: list[np.random.Generator] = []
        self._clocks: list[_LegClock] = []
        for segment_index in range(self.n_segments):
            rng = np.random.default_rng(
                np.random.SeedSequence([self.seed, segment_index, 7821])
            )
            duration = self._sample_duration(rng)
            right_stance = bool(rng.integers(0, 2))
            elapsed = float(rng.uniform(0.0, duration))
            self._rngs.append(rng)
            self._clocks.append(_LegClock(right_stance, duration, elapsed))

    def _sample_duration(self, rng: np.random.Generator) -> float:
        return float(rng.uniform(self.dwell_low_s, self.dwell_high_s))

    def advance(self, dt: float) -> None:
        if dt < 0.0:
            raise ValueError("dt must be nonnegative")
        for clock, rng in zip(self._clocks, self._rngs):
            clock.elapsed_s += dt
            while clock.elapsed_s >= clock.duration_s:
                clock.elapsed_s -= clock.duration_s
                clock.stance = not clock.stance
                clock.duration_s = self._sample_duration(rng)
                clock.transition_count += 1

    def stance_flags(self) -> np.ndarray:
        right = np.asarray([clock.stance for clock in self._clocks], dtype=bool)
        return np.column_stack((right, ~right))

    def state_progress(self) -> np.ndarray:
        progress = np.asarray([clock.progress for clock in self._clocks], dtype=float)
        return np.column_stack((progress, progress))

    def transition_counts(self) -> np.ndarray:
        counts = np.asarray(
            [clock.transition_count for clock in self._clocks], dtype=int
        )
        return np.column_stack((counts, counts))

    def control(self, body_time_s: float) -> np.ndarray:
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = self.indices
        ctrl = np.zeros(self.n_ctrl, dtype=float)
        yaw_amplitude = math.radians(YAW_HALF_AMPLITUDE_DEG)
        shoulder_amplitude = math.radians(SHOULDER_HALF_AMPLITUDE_DEG)
        leg_up = math.radians(LEG_UP_DEG)
        leg_down = math.radians(LEG_DOWN_DEG)
        leg_offset = math.radians(LEG_OFFSET_DEG)

        for segment_index, actuator_index in enumerate(link_y):
            ctrl[actuator_index] = yaw_amplitude * math.sin(
                self.omega_body * body_time_s
                + segment_index * self.phi_body
            )
        for actuator_index in link_p:
            ctrl[actuator_index] = 0.0

        flags = self.stance_flags()
        for segment_index, clock in enumerate(self._clocks):
            for side_index, side in enumerate(("R", "L")):
                is_stance = bool(flags[segment_index, side_index])
                theta = math.pi * clock.progress
                if not is_stance:
                    theta += math.pi
                shoulder_base = leg_offset if side == "R" else -leg_offset
                shoulder_target = shoulder_base + shoulder_amplitude * math.cos(theta)
                leg_target = leg_down if is_stance else leg_up
                if side == "R":
                    ctrl[shoulder_r[segment_index]] = shoulder_target
                    ctrl[leg_r[segment_index]] = leg_target
                else:
                    ctrl[shoulder_l[segment_index]] = shoulder_target
                    ctrl[leg_l[segment_index]] = leg_target
        return ctrl


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = list(rows[0])
    known = set(fieldnames)
    for row in rows[1:]:
        for field in row:
            if field not in known:
                fieldnames.append(field)
                known.add(field)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _root_roll_pitch_deg(qpos: np.ndarray) -> tuple[float, float]:
    w, x, y, z = (float(value) for value in qpos[3:7])
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def _root_up_z(qpos: np.ndarray) -> float:
    """World-z component of the root body's local up axis."""
    _w, x, y, _z = (float(value) for value in qpos[3:7])
    return 1.0 - 2.0 * (x * x + y * y)


def _leg_body_map(model, n_segments: int, mujoco) -> dict[int, tuple[int, int]]:
    body_map: dict[int, tuple[int, int]] = {}
    for segment_index in range(n_segments):
        for side_index, side in enumerate(("R", "L")):
            name = f"Leg{segment_index + 1}_{side}"
            body_id = int(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            )
            if body_id < 0:
                raise KeyError(f"required body is missing: {name}")
            body_map[body_id] = (segment_index, side_index)
    return body_map


def _actual_contact_flags(model, data, body_map) -> np.ndarray:
    flags = np.zeros((len(body_map) // 2, 2), dtype=bool)
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        body1 = int(model.geom_bodyid[int(contact.geom1)])
        body2 = int(model.geom_bodyid[int(contact.geom2)])
        limb = None
        if body1 == 0 and body2 in body_map:
            limb = body_map[body2]
        elif body2 == 0 and body1 in body_map:
            limb = body_map[body1]
        if limb is not None:
            flags[limb] = True
    return flags


def _binary_pairwise_correlation(states: np.ndarray) -> tuple[float, float]:
    """Mean and maximum absolute off-diagonal binary-state correlation."""
    if states.shape[0] < 2 or states.shape[1] < 2:
        return float("nan"), float("nan")
    keep = np.std(states, axis=0) > 0.0
    reduced = states[:, keep]
    if reduced.shape[1] < 2:
        return float("nan"), float("nan")
    corr = np.corrcoef(reduced, rowvar=False)
    values = np.abs(corr[np.triu_indices(corr.shape[0], k=1)])
    return float(np.mean(values)), float(np.max(values))


def _model_collision_check(model, mujoco) -> bool:
    floor_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor"))
    if floor_id < 0:
        return False
    floor_type = int(model.geom_contype[floor_id])
    floor_affinity = int(model.geom_conaffinity[floor_id])
    body_geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if geom_id != floor_id and int(model.geom_contype[geom_id]) == 2
    ]
    return bool(body_geom_ids) and all(
        (floor_type & int(model.geom_conaffinity[geom_id]))
        or (int(model.geom_contype[geom_id]) & floor_affinity)
        for geom_id in body_geom_ids
    )


def _run_trial(
    repo_root: Path,
    *,
    n_segments: int,
    seed: int,
    evaluation_cycles: int,
    sample_hz: float,
    mode: str,
    retain_trajectory: bool,
    trajectory_dir: Path,
) -> tuple[dict, dict[str, np.ndarray] | None]:
    import mujoco

    build_actuator_indices, canonical_compute_ctrl = _install_project_imports(repo_root)
    xml_path = repo_root / "centipede_sim" / "model" / f"centipede_{n_segments}seg.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    expected_ctrl = n_segments * 4 + (n_segments - 1) * 2
    if model.nu != expected_ctrl:
        raise RuntimeError(f"expected {expected_ctrl} actuators, found {model.nu}")
    indices = build_actuator_indices(n_segments)
    controller_type = (
        RandomBilateralComplementController
        if mode == "random_bilateral_complement"
        else IndependentRandomLegController
    )
    random_controller = controller_type(
        n_segments,
        model.nu,
        indices,
        seed=seed,
    )
    stochastic_mode = mode in RANDOM_MODES
    dt = float(model.opt.timestep)
    gait_period = 2.0 * math.pi / OMEGA_BODY
    evaluation_time_s = evaluation_cycles * gait_period
    evaluation_steps = int(math.ceil(evaluation_time_s / dt))
    evaluation_time_s = evaluation_steps * dt
    sample_stride = max(1, int(round(1.0 / (sample_hz * dt))))
    root_body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
    )
    if root_body_id < 0:
        raise KeyError("robot_root body is missing")
    body_map = _leg_body_map(model, n_segments, mujoco)

    # Deterministic settle to the canonical initial pose.
    initial_target = canonical_compute_ctrl(
        0.0,
        model.nu,
        n_segments,
        indices,
        omega_spine=OMEGA_BODY,
        phi_spine=PHI_BODY,
        dutyfactor=DUTY_FACTOR,
    )
    settle_steps = int(math.ceil(SETTLE_TIME_S / dt))
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial_target
        mujoco.mj_step(model, data)

    # Smoothly hand over to the independently timed controller.  This bridge
    # is deliberately excluded from the displacement evaluation.
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / dt))
    for step in range(bridge_steps):
        bridge_time = step * dt
        if stochastic_mode:
            candidate = random_controller.control(bridge_time)
            random_controller.advance(dt)
        elif mode == "regular_coupled":
            candidate = canonical_compute_ctrl(
                bridge_time,
                model.nu,
                n_segments,
                indices,
                omega_spine=OMEGA_BODY,
                phi_spine=PHI_BODY,
                dutyfactor=DUTY_FACTOR,
            )
        else:
            raise ValueError(f"unsupported mode: {mode}")
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial_target + alpha * candidate
        mujoco.mj_step(model, data)

    start_com = np.asarray(data.subtree_com[root_body_id]).copy()
    start_root_z = float(data.qpos[2])
    min_root_z = start_root_z
    max_abs_roll_deg = 0.0
    max_abs_pitch_deg = 0.0
    min_root_up_z = _root_up_z(np.asarray(data.qpos))
    max_abs_qacc = 0.0
    all_finite = True
    mechanical_absolute_work_j = 0.0
    mechanical_positive_work_j = 0.0

    times: list[float] = []
    coms: list[np.ndarray] = []
    roots: list[np.ndarray] = []
    planned_states: list[np.ndarray] = []
    actual_contacts: list[np.ndarray] = []

    for step in range(evaluation_steps):
        time_s = step * dt
        body_time_s = BRIDGE_TIME_S + time_s
        if stochastic_mode:
            ctrl = random_controller.control(body_time_s)
        else:
            ctrl = canonical_compute_ctrl(
                body_time_s,
                model.nu,
                n_segments,
                indices,
                omega_spine=OMEGA_BODY,
                phi_spine=PHI_BODY,
                dutyfactor=DUTY_FACTOR,
            )
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        if stochastic_mode:
            random_controller.advance(dt)

        power = np.asarray(data.actuator_force) * np.asarray(data.actuator_velocity)
        mechanical_absolute_work_j += float(np.sum(np.abs(power))) * dt
        mechanical_positive_work_j += float(np.sum(np.maximum(power, 0.0))) * dt
        all_finite &= bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and np.all(np.isfinite(data.qacc))
        )
        max_abs_qacc = max(max_abs_qacc, float(np.max(np.abs(data.qacc))))
        min_root_z = min(min_root_z, float(data.qpos[2]))
        roll_deg, pitch_deg = _root_roll_pitch_deg(np.asarray(data.qpos))
        max_abs_roll_deg = max(max_abs_roll_deg, abs(roll_deg))
        max_abs_pitch_deg = max(max_abs_pitch_deg, abs(pitch_deg))
        min_root_up_z = min(min_root_up_z, _root_up_z(np.asarray(data.qpos)))

        if step % sample_stride == 0 or step == evaluation_steps - 1:
            times.append((step + 1) * dt)
            coms.append(np.asarray(data.subtree_com[root_body_id]).copy())
            roots.append(np.asarray(data.qpos[:7]).copy())
            if stochastic_mode:
                planned_states.append(random_controller.stance_flags().copy())
            else:
                # Canonical state is logged only for shape-compatible output.
                phase = (
                    OMEGA_BODY * body_time_s
                    + np.arange(n_segments)[:, None] * PHI_BODY
                    + np.asarray([[0.0, math.pi]])
                )
                planned_states.append(np.cos(phase) >= 0.0)
            actual_contacts.append(_actual_contact_flags(model, data, body_map))

    final_com = np.asarray(data.subtree_com[root_body_id]).copy()
    displacement = final_com - start_com
    states = np.asarray(planned_states, dtype=bool)
    contacts = np.asarray(actual_contacts, dtype=bool)
    bilateral_complement_exact = bool(np.all(np.sum(states, axis=2) == 1))
    flat_states = states.reshape(states.shape[0], -1).astype(float)
    mean_abs_corr, max_abs_corr = _binary_pairwise_correlation(flat_states)
    planned_stance_fraction = float(np.mean(states))
    actual_contact_fraction = float(np.mean(contacts))
    stance_count = int(np.sum(states))
    swing_count = int(states.size - stance_count)
    contact_given_stance = (
        float(np.sum(contacts & states) / stance_count) if stance_count else float("nan")
    )
    contact_given_swing = (
        float(np.sum(contacts & ~states) / swing_count) if swing_count else float("nan")
    )
    warning_counts = {
        str(index): int(data.warning[index].number)
        for index in range(len(data.warning))
        if int(data.warning[index].number) > 0
    }
    warning_total = int(sum(warning_counts.values()))
    gates = {
        "model_loaded": True,
        "body_ground_contact_enabled": _model_collision_check(model, mujoco),
        "all_states_finite": all_finite,
        "no_mujoco_warnings": warning_total == 0,
        # Body-floor contact is intentionally enabled.  A low root height is
        # therefore a diagnostic, not a failure.  True rollover is gated by
        # the root body's up direction, matching the long-horizon pipeline.
        "never_inverted": min_root_up_z > 0.0,
        "trajectory_complete": len(times) >= 2,
    }
    if mode == "random_bilateral_complement":
        gates["bilateral_complement_exact"] = bilateral_complement_exact
    status = "PASS" if all(gates.values()) else "FAIL"
    result = {
        "schema_version": 1,
        "status": status,
        "mode": mode,
        "n_segments": n_segments,
        "n_legs": 2 * n_segments,
        "seed": seed,
        "evaluation_cycles": evaluation_cycles,
        "evaluation_time_s": evaluation_time_s,
        "yaw_half_amplitude_deg": YAW_HALF_AMPLITUDE_DEG,
        "omega_body_rad_s": OMEGA_BODY,
        "phi_body_rad": PHI_BODY,
        "forward_displacement_m": float(displacement[1]),
        "lateral_displacement_m": float(displacement[0]),
        "planar_displacement_m": float(np.linalg.norm(displacement[:2])),
        "vertical_displacement_m": float(displacement[2]),
        "mechanical_absolute_work_j": mechanical_absolute_work_j,
        "mechanical_positive_work_j": mechanical_positive_work_j,
        "planned_stance_fraction": planned_stance_fraction,
        "actual_contact_fraction": actual_contact_fraction,
        "contact_given_planned_stance": contact_given_stance,
        "contact_given_planned_swing": contact_given_swing,
        "mean_abs_pairwise_state_correlation": mean_abs_corr,
        "max_abs_pairwise_state_correlation": max_abs_corr,
        "bilateral_complement_exact": bilateral_complement_exact,
        "mean_transitions_per_leg": float(np.mean(random_controller.transition_counts()))
        if stochastic_mode
        else float("nan"),
        "min_root_z_m": min_root_z,
        "start_root_z_m": start_root_z,
        "max_abs_roll_deg": max_abs_roll_deg,
        "max_abs_pitch_deg": max_abs_pitch_deg,
        "minimum_root_up_z": min_root_up_z,
        "low_root_z_diagnostic": min_root_z < 0.5 * start_root_z,
        "max_abs_qacc": max_abs_qacc,
        "mujoco_warning_total": warning_total,
        "mujoco_warning_counts": json.dumps(warning_counts, sort_keys=True),
        "gates": gates,
    }
    trajectory = None
    if retain_trajectory:
        trajectory = {
            "time_s": np.asarray(times),
            "com_xyz_m": np.asarray(coms),
            "root_qpos": np.asarray(roots),
            "planned_stance": states,
            "actual_contact": contacts,
        }
        trajectory_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            trajectory_dir / f"N{n_segments}_{mode}_seed{seed:03d}.npz",
            **trajectory,
        )
    return result, trajectory


def _aggregate(trials: list[dict]) -> list[dict]:
    rows: list[dict] = []
    present_modes = [
        mode
        for mode in ("regular_coupled", *RANDOM_MODES)
        if any(row["mode"] == mode for row in trials)
    ]
    for n_segments in sorted({int(row["n_segments"]) for row in trials}):
        for mode in present_modes:
            selected_all = [
                row
                for row in trials
                if int(row["n_segments"]) == n_segments and row["mode"] == mode
            ]
            selected = [row for row in selected_all if row["status"] == "PASS"]
            values = np.asarray(
                [row["forward_displacement_m"] for row in selected], dtype=float
            )
            planar = np.asarray(
                [row["planar_displacement_m"] for row in selected], dtype=float
            )
            if values.size == 0:
                continue
            sd = float(np.std(values, ddof=1)) if values.size > 1 else 0.0
            ci95 = 1.96 * sd / math.sqrt(values.size) if values.size > 1 else 0.0
            rows.append(
                {
                    "n_segments": n_segments,
                    "n_legs": 2 * n_segments,
                    "mode": mode,
                    "n_trials": len(selected_all),
                    "n_stable": len(selected),
                    "stable_fraction": len(selected) / len(selected_all),
                    "mean_forward_displacement_m": float(np.mean(values)),
                    "sd_forward_displacement_m": sd,
                    "ci95_forward_displacement_m": ci95,
                    "median_forward_displacement_m": float(np.median(values)),
                    "min_forward_displacement_m": float(np.min(values)),
                    "max_forward_displacement_m": float(np.max(values)),
                    "mean_planar_displacement_m": float(np.mean(planar)),
                    "mean_planned_stance_fraction": float(
                        np.mean([row["planned_stance_fraction"] for row in selected])
                    ),
                    "mean_actual_contact_fraction": float(
                        np.mean([row["actual_contact_fraction"] for row in selected])
                    ),
                    "mean_contact_given_planned_stance": float(
                        np.mean(
                            [row["contact_given_planned_stance"] for row in selected]
                        )
                    ),
                    "mean_contact_given_planned_swing": float(
                        np.mean(
                            [row["contact_given_planned_swing"] for row in selected]
                        )
                    ),
                }
            )
    return rows


def _paired_comparisons(
    trials: list[dict], random_mode: str = "independent_random"
) -> list[dict]:
    passed = [
        row
        for row in trials
        if row["mode"] == random_mode and row["status"] == "PASS"
    ]
    lookup = {
        (int(row["n_segments"]), int(row["seed"])): float(
            row["forward_displacement_m"]
        )
        for row in passed
    }
    rows: list[dict] = []
    for lower, upper in ((4, 6), (6, 8), (4, 8)):
        common_seeds = sorted(
            seed
            for n_segments, seed in lookup
            if n_segments == lower and (upper, seed) in lookup
        )
        differences = np.asarray(
            [lookup[(upper, seed)] - lookup[(lower, seed)] for seed in common_seeds],
            dtype=float,
        )
        if differences.size == 0:
            continue
        sd = float(np.std(differences, ddof=1)) if differences.size > 1 else 0.0
        ci95 = 1.96 * sd / math.sqrt(differences.size) if differences.size > 1 else 0.0
        rows.append(
            {
                "lower_segments": lower,
                "upper_segments": upper,
                "paired_seed_count": int(differences.size),
                "mean_difference_upper_minus_lower_m": float(np.mean(differences)),
                "sd_difference_m": sd,
                "ci95_difference_m": ci95,
                "fraction_upper_greater_than_lower": float(np.mean(differences > 0.0)),
            }
        )
    return rows


def _plot_results(
    output_dir: Path,
    trials: list[dict],
    summary: list[dict],
    random_mode: str = "independent_random",
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    available_segments = sorted({int(row["n_segments"]) for row in trials})
    palette = ("#3366AA", "#DD8822", "#228855", "#AA4499")
    colors = {
        n_segments: palette[index % len(palette)]
        for index, n_segments in enumerate(available_segments)
    }

    # Primary figure: absolute signed forward displacement, never normalized.
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    rng = np.random.default_rng(20260820)
    for index, n_segments in enumerate(sorted(colors)):
        selected = [
            row
            for row in trials
            if row["mode"] == random_mode
            and row["status"] == "PASS"
            and int(row["n_segments"]) == n_segments
        ]
        values = np.asarray([row["forward_displacement_m"] for row in selected])
        jitter = rng.uniform(-0.08, 0.08, size=len(values))
        ax.scatter(
            index + jitter,
            values,
            s=27,
            alpha=0.55,
            color=colors[n_segments],
            edgecolors="none",
        )
        item = next(
            row
            for row in summary
            if row["mode"] == random_mode
            and int(row["n_segments"]) == n_segments
        )
        ax.errorbar(
            index,
            item["mean_forward_displacement_m"],
            yerr=item["ci95_forward_displacement_m"],
            fmt="o",
            markersize=8,
            capsize=5,
            color=colors[n_segments],
            markeredgecolor="black",
            markeredgewidth=0.6,
            zorder=5,
        )
        baseline = next(
            row
            for row in summary
            if row["mode"] == "regular_coupled"
            and int(row["n_segments"]) == n_segments
        )
        ax.plot(
            index,
            baseline["mean_forward_displacement_m"],
            marker="D",
            markersize=7,
            color="#555555",
            linestyle="none",
            zorder=6,
        )
    ax.axhline(0.0, color="#777777", linewidth=0.9)
    ax.set_xticks(range(len(available_segments)), [str(value) for value in available_segments])
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("Forward COM displacement (m)")
    ax.set_title(_mode_label(random_mode))
    ax.grid(axis="y", alpha=0.25)
    ax.plot([], [], "o", color="#555555", label="Random trials; mean ± 95% CI")
    ax.plot([], [], "D", color="#555555", label="Regular coupled gait")
    ax.legend(frameon=False, loc="best")
    fig.savefig(figure_dir / "forward_displacement_vs_segment_count.png", dpi=220)
    plt.close(fig)

    # Secondary figure: individual-seed paired comparison across morphologies.
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    random_rows = [
        row
        for row in trials
        if row["mode"] == random_mode and row["status"] == "PASS"
    ]
    seeds = sorted({int(row["seed"]) for row in random_rows})
    for seed in seeds:
        paired = {
            int(row["n_segments"]): float(row["forward_displacement_m"])
            for row in random_rows
            if int(row["seed"]) == seed
        }
        if all(n in paired for n in available_segments):
            ax.plot(
                available_segments,
                [paired[n] for n in available_segments],
                color="#888888",
                alpha=0.22,
                linewidth=0.8,
            )
    means = [
        next(
            row["mean_forward_displacement_m"]
            for row in summary
            if row["mode"] == random_mode
            and int(row["n_segments"]) == n
        )
        for n in available_segments
    ]
    cis = [
        next(
            row["ci95_forward_displacement_m"]
            for row in summary
            if row["mode"] == random_mode
            and int(row["n_segments"]) == n
        )
        for n in available_segments
    ]
    ax.errorbar(
        available_segments,
        means,
        yerr=cis,
        color="#222222",
        marker="o",
        markersize=7,
        linewidth=2.2,
        capsize=5,
        label="Mean ± 95% CI",
    )
    ax.axhline(0.0, color="#777777", linewidth=0.9)
    ax.set_xticks(available_segments)
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("Forward COM displacement (m)")
    ax.set_title("Paired random schedules across segment counts")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.savefig(figure_dir / "paired_seed_displacement.png", dpi=220)
    plt.close(fig)

    # Distribution views use the signed displacement in meters.  Common bins
    # and axes make changes in spread across segment counts directly visible.
    grouped_values = {
        n_segments: np.asarray(
            [
                float(row["forward_displacement_m"])
                for row in random_rows
                if int(row["n_segments"]) == n_segments
            ],
            dtype=float,
        )
        for n_segments in available_segments
    }
    all_values = np.concatenate(list(grouped_values.values()))
    lower = float(np.min(all_values))
    upper = float(np.max(all_values))
    padding = max(0.02, 0.05 * max(upper - lower, 0.02))
    bin_edges = np.linspace(lower - padding, upper + padding, 14)
    fig, axes = plt.subplots(
        1,
        len(available_segments),
        figsize=(10.8, 3.7),
        sharex=True,
        sharey=True,
    )
    if len(available_segments) == 1:
        axes = [axes]
    fig.subplots_adjust(left=0.08, bottom=0.17, right=0.99, top=0.82, wspace=0.12)
    for axis, n_segments in zip(axes, available_segments):
        values = grouped_values[n_segments]
        axis.hist(
            values,
            bins=bin_edges,
            color=colors[n_segments],
            alpha=0.82,
            edgecolor="white",
            linewidth=0.7,
        )
        axis.axvline(
            float(np.mean(values)),
            color="#222222",
            linewidth=1.6,
            label="mean",
        )
        axis.set_title(f"N={n_segments}  (n={len(values)})")
        axis.grid(axis="y", alpha=0.20)
    axes[0].set_ylabel("Trial count")
    for axis in axes:
        axis.set_xlabel("Forward COM displacement (m)")
    fig.suptitle(f"Displacement distributions: {_mode_label(random_mode)}", fontsize=14)
    axes[-1].legend(frameon=False, loc="upper right")
    fig.savefig(figure_dir / "forward_displacement_histograms.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    for n_segments in available_segments:
        values = np.sort(grouped_values[n_segments])
        probability = np.arange(1, len(values) + 1, dtype=float) / len(values)
        ax.step(
            values,
            probability,
            where="post",
            color=colors[n_segments],
            linewidth=2.0,
            label=f"N={n_segments}",
        )
    ax.axvline(0.0, color="#777777", linewidth=0.9)
    ax.set_xlabel("Forward COM displacement (m)")
    ax.set_ylabel("Empirical cumulative probability")
    ax.set_title("Empirical displacement distributions")
    ax.set_ylim(0.0, 1.02)
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.savefig(figure_dir / "forward_displacement_ecdf.png", dpi=220)
    plt.close(fig)


def _plot_state_schedule(
    output_dir: Path,
    *,
    n_segments: int,
    seed: int,
    mode: str = "independent_random",
) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch

    path = (
        output_dir
        / "trajectories"
        / f"N{n_segments}_{mode}_seed{seed:03d}.npz"
    )
    with np.load(path) as payload:
        time_s = np.asarray(payload["time_s"])
        stance = np.asarray(payload["planned_stance"], dtype=bool)
        contact = np.asarray(payload["actual_contact"], dtype=bool)
    stance_flat = stance.reshape(stance.shape[0], -1).T
    contact_flat = contact.reshape(contact.shape[0], -1).T
    labels = [
        f"{segment + 1}{side}"
        for segment in range(n_segments)
        for side in ("R", "L")
    ]
    extent = (float(time_s[0]), float(time_s[-1]), 0.5, len(labels) + 0.5)
    fig, axes = plt.subplots(
        2,
        1,
        figsize=(9.0, 6.2),
        sharex=True,
        gridspec_kw={"height_ratios": (1.0, 1.0), "hspace": 0.18},
    )
    fig.subplots_adjust(left=0.10, bottom=0.11, right=0.98, top=0.90)
    axes[0].imshow(
        stance_flat,
        aspect="auto",
        interpolation="nearest",
        origin="lower",
        extent=extent,
        cmap=ListedColormap(["#e6452b", "#2aa657"]),
        vmin=0,
        vmax=1,
    )
    axes[1].imshow(
        contact_flat,
        aspect="auto",
        interpolation="nearest",
        origin="lower",
        extent=extent,
        cmap=ListedColormap(["#eeeeee", "#222222"]),
        vmin=0,
        vmax=1,
    )
    for axis, title in zip(axes, ("Commanded leg state", "Measured ground contact")):
        axis.set_yticks(range(1, len(labels) + 1), labels)
        axis.set_ylabel("Physical leg")
        axis.set_title(title, loc="left", fontsize=11)
    axes[1].set_xlabel("Evaluation time (s)")
    fig.suptitle(
        f"{_mode_label(mode)} | N={n_segments}, seed={seed}",
        fontsize=15,
    )
    axes[0].legend(
        handles=(
            Patch(facecolor="#2aa657", label="stance"),
            Patch(facecolor="#e6452b", label="swing"),
        ),
        frameon=False,
        ncol=2,
        loc="upper right",
        bbox_to_anchor=(1.0, 1.17),
    )
    output_path = output_dir / "figures" / f"N{n_segments}_{mode}_state_schedule.png"
    fig.savefig(output_path, dpi=220)
    plt.close(fig)
    return output_path


def _font(size: int, *, bold: bool = False):
    from PIL import ImageFont

    candidates = (
        (Path("C:/Windows/Fonts/arialbd.ttf"), Path("C:/Windows/Fonts/calibrib.ttf"))
        if bold
        else (Path("C:/Windows/Fonts/arial.ttf"), Path("C:/Windows/Fonts/calibri.ttf"))
    )
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


def _set_leg_colors(model, body_map, stance: np.ndarray) -> None:
    stance_rgba = np.asarray([0.16, 0.66, 0.35, 1.0])
    swing_rgba = np.asarray([0.90, 0.27, 0.17, 1.0])
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        if body_id in body_map:
            model.geom_rgba[geom_id] = (
                stance_rgba if stance[body_map[body_id]] else swing_rgba
            )


def _compose_demo_frame(
    pixels: np.ndarray,
    *,
    n_segments: int,
    seed: int,
    time_s: float,
    displacement_m: float,
    stance: np.ndarray,
    actual_contact: np.ndarray,
    mode: str = "independent_random",
) -> np.ndarray:
    from PIL import Image, ImageDraw

    image = Image.fromarray(pixels)
    width, height = image.size
    header = 58
    footer = 32
    canvas = Image.new("RGB", (width, height + header + footer), "#f3f4f6")
    canvas.paste(image, (0, header))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (18, 10),
        f"{_mode_label(mode)} | N={n_segments} | seed={seed}",
        font=_font(23, bold=True),
        fill="#17202a",
    )
    draw.text(
        (18, height + header + 5),
        (
            f"t={time_s:5.2f} s   forward COM={displacement_m:+.3f} m   "
            f"stance/contact={int(np.sum(stance))}/{int(np.sum(actual_contact))} legs"
        ),
        font=_font(16),
        fill="#273746",
    )
    draw.ellipse((width - 265, 17, width - 249, 33), fill="#2aa657")
    draw.text((width - 242, 12), "stance", font=_font(16), fill="#17202a")
    draw.ellipse((width - 155, 17, width - 139, 33), fill="#e6452b")
    draw.text((width - 132, 12), "swing", font=_font(16), fill="#17202a")
    return np.asarray(canvas)


def _render_demo(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    seed: int,
    duration_s: float,
    fps: int,
    mode: str = "independent_random",
) -> dict:
    import imageio.v2 as imageio
    import mujoco

    build_actuator_indices, canonical_compute_ctrl = _install_project_imports(repo_root)
    xml_path = repo_root / "centipede_sim" / "model" / f"centipede_{n_segments}seg.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    indices = build_actuator_indices(n_segments)
    controller_type = (
        RandomBilateralComplementController
        if mode == "random_bilateral_complement"
        else IndependentRandomLegController
    )
    controller = controller_type(
        n_segments, model.nu, indices, seed=seed
    )
    dt = float(model.opt.timestep)
    root_body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
    )
    body_map = _leg_body_map(model, n_segments, mujoco)
    initial_target = canonical_compute_ctrl(
        0.0,
        model.nu,
        n_segments,
        indices,
        omega_spine=OMEGA_BODY,
        phi_spine=PHI_BODY,
        dutyfactor=DUTY_FACTOR,
    )
    settle_steps = int(math.ceil(SETTLE_TIME_S / dt))
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial_target
        mujoco.mj_step(model, data)
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / dt))
    for step in range(bridge_steps):
        time_s = step * dt
        candidate = controller.control(time_s)
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial_target + alpha * candidate
        mujoco.mj_step(model, data)
        controller.advance(dt)

    start_com = np.asarray(data.subtree_com[root_body_id]).copy()
    render_height = 630
    render_width = 1280
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, render_width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, render_height)
    renderer = mujoco.Renderer(model, height=render_height, width=render_width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = root_body_id
    camera.distance = max(1.0, n_segments * 0.18)
    camera.azimuth = 150
    camera.elevation = -48
    camera.lookat[:] = [0.0, 0.0, 0.0]
    option = mujoco.MjvOption()
    option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
    option.sitegroup[:] = [0, 0, 0, 0, 0, 0]

    demo_dir = output_dir / "demos"
    demo_dir.mkdir(parents=True, exist_ok=True)
    video_path = demo_dir / f"N{n_segments}_{mode}_seed{seed:03d}.mp4"
    preview_path = demo_dir / f"N{n_segments}_{mode}_seed{seed:03d}_preview.png"
    writer = imageio.get_writer(
        str(video_path),
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=2,
    )
    next_frame_time = 0.0
    preview_saved = False
    frame_count = 0
    steps = int(math.ceil(duration_s / dt))
    try:
        for step in range(steps):
            time_s = step * dt
            data.ctrl[:] = controller.control(BRIDGE_TIME_S + time_s)
            mujoco.mj_step(model, data)
            controller.advance(dt)
            time_end = (step + 1) * dt
            if time_end + 0.5 * dt >= next_frame_time:
                stance = controller.stance_flags()
                contact = _actual_contact_flags(model, data, body_map)
                _set_leg_colors(model, body_map, stance)
                renderer.update_scene(data, camera=camera, scene_option=option)
                frame = _compose_demo_frame(
                    renderer.render().copy(),
                    n_segments=n_segments,
                    seed=seed,
                    time_s=time_end,
                    displacement_m=float(
                        data.subtree_com[root_body_id][1] - start_com[1]
                    ),
                    stance=stance,
                    actual_contact=contact,
                    mode=mode,
                )
                writer.append_data(frame)
                frame_count += 1
                if not preview_saved and time_end >= 0.5 * duration_s:
                    from PIL import Image

                    Image.fromarray(frame).save(preview_path)
                    preview_saved = True
                next_frame_time += 1.0 / fps
    finally:
        writer.close()
        renderer.close()
    return {
        "n_segments": n_segments,
        "seed": seed,
        "duration_s": duration_s,
        "fps": fps,
        "frame_count": frame_count,
        "mode": mode,
        "video": str(video_path),
        "preview": str(preview_path),
    }


def _representative_seeds(
    trials: list[dict], random_mode: str = "independent_random"
) -> dict[int, int]:
    selected: dict[int, int] = {}
    for n_segments in (4, 6, 8):
        rows = [
            row
            for row in trials
            if row["mode"] == random_mode
            and row["status"] == "PASS"
            and int(row["n_segments"]) == n_segments
        ]
        median = float(np.median([row["forward_displacement_m"] for row in rows]))
        chosen = min(
            rows,
            key=lambda row: (
                abs(float(row["forward_displacement_m"]) - median),
                int(row["seed"]),
            ),
        )
        selected[n_segments] = int(chosen["seed"])
    return selected


def _write_readme(
    output_dir: Path,
    summary: list[dict],
    demo_manifest: list[dict],
    paired_comparisons: list[dict],
    random_mode: str = "independent_random",
) -> None:
    if random_mode == "random_bilateral_complement":
        title = "Random bilateral-complement stance experiment"
        process_description = (
            "Each segment pair has exactly one commanded stance leg and one "
            "commanded swing leg at every instant. Independent segment clocks "
            "randomly switch which side is in stance after dwell durations "
            "sampled from U(0.5 T_half, 1.5 T_half), T_half=pi/omega. "
            "Commanded stance and measured MuJoCo ground contact are recorded "
            "as distinct quantities."
        )
    else:
        title = "Independent random leg-phase experiment"
        process_description = (
            "Each physical leg independently alternates between stance and swing. "
            "Every dwell duration is sampled from U(0.5 T_half, 1.5 T_half), "
            "T_half=pi/omega. This preserves a 0.5 expected duty factor while "
            "making the contact opportunity schedule asynchronous and stochastic."
        )
    lines = [
        f"# {title}",
        "",
        "The body travelling wave is fixed at yaw half-amplitude 15 deg, "
        "omega=4 rad/s, and spatial phase increment -pi. The canonical "
        "body-to-leg and inter-leg phase coupling is disabled.",
        "",
        process_description,
        "",
        "The primary outcome is absolute signed whole-robot COM displacement "
        "along model +y. Failed stability trials are retained in trials.csv but "
        "excluded from mean displacement estimates.",
        "",
        "## Summary",
        "",
        "| N | control | stable / total | mean forward displacement (m) | SD (m) |",
        "|---:|---|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            f"| {row['n_segments']} | {row['mode']} | "
            f"{row['n_stable']} / {row['n_trials']} | "
            f"{row['mean_forward_displacement_m']:.4f} | "
            f"{row['sd_forward_displacement_m']:.4f} |"
        )
    lines += [
        "",
        "## Evidence",
        "",
        "- `trials.csv`: one row per simulation, including all stability gates.",
        "- `summary.csv`: stable-trial aggregates and uncertainty.",
        "- `trajectories/`: compressed sampled COM, planned state, and measured contact arrays.",
        "- `figures/forward_displacement_vs_segment_count.png`: primary result.",
        "- `figures/paired_seed_displacement.png`: common-seed morphology comparison.",
        "- `figures/forward_displacement_histograms.png`: trial-count histograms using common bins.",
        "- `figures/forward_displacement_ecdf.png`: empirical cumulative distributions.",
        f"- `figures/N8_{random_mode}_state_schedule.png`: commanded binary leg states and measured contacts.",
        "- `demos/`: median-displacement representative MP4 for N=4,6,8; green=stance, red=swing.",
        "",
        "## Paired morphology differences",
        "",
        "The same seed preserves the schedules of homologous anterior legs when posterior segments are added.",
        "",
        "| comparison | paired seeds | mean upper-lower (m) | 95% CI half-width (m) |",
        "|---|---:|---:|---:|",
    ]
    for row in paired_comparisons:
        lines.append(
            f"| N={row['upper_segments']} minus N={row['lower_segments']} | "
            f"{row['paired_seed_count']} | "
            f"{row['mean_difference_upper_minus_lower_m']:.4f} | "
            f"{row['ci95_difference_m']:.4f} |"
        )
    lines += [
        "",
        "## Demo files",
        "",
    ]
    lines.extend(f"- `{Path(item['video']).name}`" for item in demo_manifest)
    (output_dir / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_experiment(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory_dir = output_dir / "trajectories"
    trials: list[dict] = []

    # Deterministic same-horizon controls are retained as direct baselines.
    for n_segments in args.segments:
        result, _ = _run_trial(
            repo_root,
            n_segments=n_segments,
            seed=0,
            evaluation_cycles=args.evaluation_cycles,
            sample_hz=args.sample_hz,
            mode="regular_coupled",
            retain_trajectory=True,
            trajectory_dir=trajectory_dir,
        )
        trials.append(result)
        print(
            f"N={n_segments} regular: {result['status']} "
            f"D_y={result['forward_displacement_m']:+.4f} m",
            flush=True,
        )

    for n_segments in args.segments:
        for seed in range(args.seed_start, args.seed_start + args.replays):
            result, _ = _run_trial(
                repo_root,
                n_segments=n_segments,
                seed=seed,
                evaluation_cycles=args.evaluation_cycles,
                sample_hz=args.sample_hz,
                mode=args.random_mode,
                retain_trajectory=True,
                trajectory_dir=trajectory_dir,
            )
            trials.append(result)
            print(
                f"N={n_segments} seed={seed:03d}: {result['status']} "
                f"D_y={result['forward_displacement_m']:+.4f} m",
                flush=True,
            )

    # Flatten gates for a human-readable CSV while retaining nested JSON.
    csv_rows: list[dict] = []
    for row in trials:
        flat = {key: value for key, value in row.items() if key != "gates"}
        flat.update({f"gate_{key}": value for key, value in row["gates"].items()})
        csv_rows.append(flat)
    summary = _aggregate(trials)
    paired_comparisons = _paired_comparisons(trials, args.random_mode)
    _write_csv(output_dir / "trials.csv", csv_rows)
    _write_csv(output_dir / "summary.csv", summary)
    _write_csv(output_dir / "paired_comparisons.csv", paired_comparisons)
    _write_json(output_dir / "trials.json", trials)
    _write_json(
        output_dir / "run_configuration.json",
        {
            "schema_version": 1,
            "segments": list(args.segments),
            "replays": args.replays,
            "seed_start": args.seed_start,
            "evaluation_cycles": args.evaluation_cycles,
            "sample_hz": args.sample_hz,
            "random_mode": args.random_mode,
            "body_wave": {
                "yaw_half_amplitude_deg": YAW_HALF_AMPLITUDE_DEG,
                "omega_rad_s": OMEGA_BODY,
                "spatial_phase_increment_rad": PHI_BODY,
            },
            "leg_process": {
                "type": (
                    "independent_segment_bilateral_complement_renewal"
                    if args.random_mode == "random_bilateral_complement"
                    else "independent_leg_alternating_renewal"
                ),
                "states": ["stance", "swing"],
                "expected_duty_factor": DUTY_FACTOR,
                "nominal_half_period_s": math.pi / OMEGA_BODY,
                "dwell_distribution": "Uniform(0.5*T_half, 1.5*T_half)",
                "exactly_one_commanded_stance_leg_per_segment": (
                    args.random_mode == "random_bilateral_complement"
                ),
                "independent_clocks_between_segments": True,
                "independent_clocks_between_legs": (
                    args.random_mode == "independent_random"
                ),
                "homologous_prefix_schedules_shared_across_morphologies": True,
            },
            "primary_metric": "signed whole-robot COM displacement along model +y (m)",
            "failed_trial_policy": "retain in raw trials; exclude from aggregate displacement",
        },
    )
    _plot_results(output_dir, trials, summary, args.random_mode)

    demo_manifest: list[dict] = []
    if not args.no_demos:
        representatives = _representative_seeds(trials, args.random_mode)
        if representatives:
            largest = max(representatives)
            _plot_state_schedule(
                output_dir,
                n_segments=largest,
                seed=representatives[largest],
                mode=args.random_mode,
            )
        for n_segments in args.segments:
            if n_segments not in representatives:
                continue
            demo_manifest.append(
                _render_demo(
                    repo_root,
                    output_dir,
                    n_segments=n_segments,
                    seed=representatives[n_segments],
                    duration_s=args.demo_duration,
                    fps=args.demo_fps,
                    mode=args.random_mode,
                )
            )
        _write_json(output_dir / "demos" / "video_manifest.json", demo_manifest)
    _write_readme(
        output_dir,
        summary,
        demo_manifest,
        paired_comparisons,
        args.random_mode,
    )
    _write_json(
        output_dir / "summary.json",
        {
            "schema_version": 1,
            "status": "PASS"
            if all(row["stable_fraction"] > 0.0 for row in summary)
            else "FAIL",
            "summary": summary,
            "paired_comparisons": paired_comparisons,
            "demo_manifest": demo_manifest,
        },
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=_repo_root_from_script())
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=_repo_root_from_script()
        / "exp_data"
        / "reliability"
        / "step14_independent_random_leg_phases",
    )
    parser.add_argument("--segments", type=int, nargs="+", default=list(DEFAULT_SEGMENTS))
    parser.add_argument(
        "--random-mode",
        choices=RANDOM_MODES,
        default="independent_random",
    )
    parser.add_argument("--replays", type=int, default=DEFAULT_REPLAYS)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--evaluation-cycles", type=int, default=DEFAULT_EVALUATION_CYCLES)
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--demo-duration", type=float, default=8.0)
    parser.add_argument("--demo-fps", type=int, default=30)
    parser.add_argument("--no-demos", action="store_true")
    return parser


if __name__ == "__main__":
    run_experiment(build_parser().parse_args())
