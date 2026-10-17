"""Absolute-wave-number/fixed body-leg phase audit on the symmetric null.

The physical robot length is fixed while ``N_wave`` -- the number of complete
spatial waves distributed over the head-to-tail segment span -- is scanned.
This follows the live Centipede_IsaacLab_RL convention rather than the earlier
inverse-wavelength convention.  For ``N`` segment-phase locations,

    phi_spine = -2*pi*N_wave/(N-1).

For segment phase ``theta_i``, the leg-yaw waveform is shifted by ``pi/2``
from body yaw.  The continuous precursor of leg pitch is shifted by
``3*pi/2`` and converted to the robot's binary up/down target at its zero
crossings.  The left precursor is additionally shifted by ``pi``.  The only
intervention in the flipped condition is an exact left/right exchange of the
two binary leg-pitch commands; body yaw and leg yaw are retained.

This is intentionally deterministic.  Repeated rows are an ensemble of global
initial CPG phases, not IID stochastic trials, and ranges are reported as phase
sensitivity rather than confidence intervals.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import run_center_rooted_dynamic_null_iid as center  # noqa: E402
import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    BRIDGE_TIME_S,
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    LEG_UP_DEG,
    OMEGA_BODY,
    SETTLE_TIME_S,
    SHOULDER_HALF_AMPLITUDE_DEG,
    _actual_contact_flags,
    _leg_body_map,
    _root_roll_pitch_deg,
    _root_up_z,
)


OUTPUT_NAME = "step30_symmetric_null_fixed_body_leg_phase_coupling"
NULL_VARIANT = "center_rooted_dynamic_null"
MODE_CANONICAL = "canonical_coupled_contact"
MODE_FLIPPED = "lr_pitch_phase_flipped"
MODES = (MODE_CANONICAL, MODE_FLIPPED)
MODE_LABELS = {
    MODE_CANONICAL: "Canonical coupled contact",
    MODE_FLIPPED: "L/R pitch phase flipped",
}
MODE_COLORS = {MODE_CANONICAL: "#24796f", MODE_FLIPPED: "#c4513f"}
DEFAULT_N_SEGMENTS = 10
DEFAULT_N_WAVE = tuple(0.5 * value for value in range(1, 9))
DEFAULT_PHASE_OFFSETS_DEG = tuple(range(0, 360, 45))
DEFAULT_WARMUP_CYCLES = 4
DEFAULT_EVALUATION_CYCLES = 12
DEFAULT_SAMPLE_HZ = 100.0
BODY_WAVE_HALF_AMPLITUDE_RAD = 0.4
BODY_WAVE_HALF_AMPLITUDE_DEG = math.degrees(BODY_WAVE_HALF_AMPLITUDE_RAD)
LEG_YAW_BODY_PHASE_OFFSET_RAD = math.pi / 2.0
LEG_PITCH_BODY_PHASE_OFFSET_RAD = 3.0 * math.pi / 2.0
LEFT_RIGHT_PITCH_PHASE_OFFSET_RAD = math.pi
PITCH_ZERO_CROSSING_TOL = 1.0e-12


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
        / "step25_center_rooted_dynamic_null"
        / "models"
        / NULL_VARIANT
        / f"centipede_{n_segments}seg_{NULL_VARIANT}.xml"
    )


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


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def spatial_phase_increment(n_wave: float, n_segments: int) -> float:
    """Inter-segment lag for ``n_wave`` full waves across the body span.

    The RL controller uses ``2*pi*n_wave/(N_BODY_JOINTS-1)``.  The modular
    robot has one leg-phase location per physical segment, so its homologous
    head-to-tail span contains ``n_segments-1`` intervals.  The negative sign
    preserves this MuJoCo controller's established propagation direction.
    """
    if n_wave <= 0.0:
        raise ValueError("N_wave must be positive")
    if n_segments < 2:
        raise ValueError("at least two segments are required")
    return -2.0 * math.pi * float(n_wave) / float(n_segments - 1)


def _step_is_down(continuous_pitch_wave: float) -> bool:
    """Map the pitch precursor to the robot's binary target at zero.

    The established robot ``leg_angle`` convention maps the positive half-wave
    to pitch-up and the nonpositive half-wave to pitch-down. Values numerically
    indistinguishable from the zero crossing therefore enter pitch-down.
    """
    value = float(continuous_pitch_wave)
    if abs(value) <= PITCH_ZERO_CROSSING_TOL:
        value = 0.0
    return value <= 0.0


class CoupledContactPatternController:
    """Canonical coupled CPG with an optional pitch-only L/R exchange."""

    def __init__(
        self,
        n_segments: int,
        n_ctrl: int,
        indices,
        canonical_compute_ctrl,
        *,
        n_wave: float,
        global_phase_deg: float,
        flipped: bool,
    ) -> None:
        self.n_segments = int(n_segments)
        self.n_ctrl = int(n_ctrl)
        self.indices = indices
        # Retain the argument for compatibility with the earlier experiment
        # entry point, but do not use the legacy helper: it shares one
        # phi_bodyleg across yaw and pitch and cannot enforce this contract.
        self.canonical_compute_ctrl = canonical_compute_ctrl
        self.n_wave = float(n_wave)
        self.phi_body = spatial_phase_increment(self.n_wave, self.n_segments)
        self.phi_leg_yaw = LEG_YAW_BODY_PHASE_OFFSET_RAD
        self.phi_leg_pitch = LEG_PITCH_BODY_PHASE_OFFSET_RAD
        self.global_phase_rad = math.radians(float(global_phase_deg))
        self.flipped = bool(flipped)

    def _phase_time(self, body_time_s: float) -> float:
        return float(body_time_s) + self.global_phase_rad / OMEGA_BODY

    def control(self, body_time_s: float, _decision_time_s: float = 0.0) -> np.ndarray:
        ctrl = np.zeros(self.n_ctrl, dtype=float)
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = self.indices
        phase_time_s = self._phase_time(body_time_s)
        shoulder_amplitude = math.radians(SHOULDER_HALF_AMPLITUDE_DEG)
        leg_offset = math.radians(LEG_OFFSET_DEG)
        leg_up = math.radians(LEG_UP_DEG)
        leg_down = math.radians(LEG_DOWN_DEG)

        # Body-yaw samples the same segment-indexed travelling phase field.
        for body_index, actuator_index in enumerate(link_y):
            ctrl[actuator_index] = BODY_WAVE_HALF_AMPLITUDE_RAD * math.sin(
                OMEGA_BODY * phase_time_s + body_index * self.phi_body
            )
        for actuator_index in link_p:
            ctrl[actuator_index] = 0.0

        for segment_index in range(self.n_segments):
            theta = OMEGA_BODY * phase_time_s + segment_index * self.phi_body

            # Actual oscillatory leg-yaw target is +pi/2 from body yaw.
            leg_yaw_wave = math.sin(theta + self.phi_leg_yaw)
            ctrl[shoulder_r[segment_index]] = (
                leg_offset + shoulder_amplitude * leg_yaw_wave
            )
            ctrl[shoulder_l[segment_index]] = (
                -leg_offset + shoulder_amplitude * leg_yaw_wave
            )

            # Generate the continuous 3*pi/2-shifted pitch precursor first,
            # then quantize it at zero because this robot uses up/down steps.
            pitch_wave_r = math.sin(theta + self.phi_leg_pitch)
            pitch_wave_l = math.sin(
                theta
                + self.phi_leg_pitch
                + LEFT_RIGHT_PITCH_PHASE_OFFSET_RAD
            )
            ctrl[leg_r[segment_index]] = (
                leg_down if _step_is_down(pitch_wave_r) else leg_up
            )
            ctrl[leg_l[segment_index]] = (
                leg_down if _step_is_down(pitch_wave_l) else leg_up
            )

        if self.flipped:
            right = ctrl[leg_r].copy()
            left = ctrl[leg_l].copy()
            ctrl[leg_r] = left
            ctrl[leg_l] = right
        return ctrl

    def stance_flags(self, body_time_s: float) -> np.ndarray:
        ctrl = self.control(body_time_s)
        _shoulder_r, leg_r, _shoulder_l, leg_l, _link_y, _link_p = self.indices
        down = math.radians(LEG_DOWN_DEG)
        return np.column_stack(
            (
                np.isclose(ctrl[leg_r], down, atol=1e-10),
                np.isclose(ctrl[leg_l], down, atol=1e-10),
            )
        )


@dataclass
class Prepared:
    model: object
    data: object
    controller: CoupledContactPatternController
    indices: object
    root_body_id: int
    body_map: dict[int, tuple[int, int]]
    foot_geom_ids: np.ndarray
    dt: float
    warmup_time_s: float


def _foot_geom_ids(model, n_segments: int, mujoco) -> np.ndarray:
    ids = np.empty((n_segments, 2), dtype=int)
    for segment_index in range(n_segments):
        for side_index, side in enumerate(("R", "L")):
            name = f"foot_geom_{segment_index + 1}_{side}"
            geom_id = int(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            )
            if geom_id < 0:
                raise KeyError(name)
            ids[segment_index, side_index] = geom_id
    return ids


def prepare_simulation(
    repo_root: Path,
    *,
    n_segments: int,
    n_wave: int,
    phase_offset_deg: float,
    mode: str,
    warmup_cycles: int,
) -> Prepared:
    import mujoco

    build_indices, compute_ctrl = base._install_project_imports(repo_root)
    model_path = _model_path(repo_root, n_segments)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    indices = build_indices(n_segments)
    controller = CoupledContactPatternController(
        n_segments,
        model.nu,
        indices,
        compute_ctrl,
        n_wave=n_wave,
        global_phase_deg=phase_offset_deg,
        flipped=mode == MODE_FLIPPED,
    )
    root_body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
    )
    if root_body_id < 0:
        raise KeyError("robot_root")
    body_map = _leg_body_map(model, n_segments, mujoco)
    foot_ids = _foot_geom_ids(model, n_segments, mujoco)
    dt = float(model.opt.timestep)
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    warmup_steps = int(math.ceil(warmup_cycles * gait_period_s / dt))
    warmup_time_s = warmup_steps * dt

    initial_target = center._symmetric_initial_target(model.nu, indices)
    settle_steps = int(math.ceil(SETTLE_TIME_S / dt))
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial_target
        mujoco.mj_step(model, data)

    bridge_target = controller.control(0.0)
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / dt))
    for step in range(bridge_steps):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial_target + alpha * bridge_target
        mujoco.mj_step(model, data)

    for step in range(warmup_steps):
        body_time_s = step * dt
        data.ctrl[:] = controller.control(body_time_s)
        mujoco.mj_step(model, data)

    return Prepared(
        model=model,
        data=data,
        controller=controller,
        indices=indices,
        root_body_id=root_body_id,
        body_map=body_map,
        foot_geom_ids=foot_ids,
        dt=dt,
        warmup_time_s=warmup_time_s,
    )


def validate_control_contract(repo_root: Path, n_segments: int) -> dict:
    build_indices, compute_ctrl = base._install_project_imports(repo_root)
    indices = build_indices(n_segments)
    n_ctrl = n_segments * 4 + (n_segments - 1) * 2
    flip_checks = []
    definition_checks = []
    amplitude_checks = []
    coupling_checks = []
    step_checks = [
        _step_is_down(-1.0e-6),
        _step_is_down(0.0),
        not _step_is_down(1.0e-6),
    ]
    for n_wave in DEFAULT_N_WAVE:
        canonical = CoupledContactPatternController(
            n_segments,
            n_ctrl,
            indices,
            compute_ctrl,
            n_wave=n_wave,
            global_phase_deg=17.0,
            flipped=False,
        )
        flipped = CoupledContactPatternController(
            n_segments,
            n_ctrl,
            indices,
            compute_ctrl,
            n_wave=n_wave,
            global_phase_deg=17.0,
            flipped=True,
        )
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = indices
        definition_checks.append(
            bool(
                math.isclose(
                    canonical.phi_body * (n_segments - 1),
                    -2.0 * math.pi * n_wave,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                and math.isclose(
                    canonical.phi_leg_yaw,
                    math.pi / 2.0,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                and math.isclose(
                    canonical.phi_leg_pitch,
                    3.0 * math.pi / 2.0,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
            )
        )
        peak_time_s = (
            math.pi / 2.0 - canonical.global_phase_rad
        ) / OMEGA_BODY
        peak_control = canonical.control(peak_time_s)
        amplitude_checks.append(
            bool(
                link_y
                and math.isclose(
                    float(peak_control[link_y[0]]),
                    BODY_WAVE_HALF_AMPLITUDE_RAD,
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
            )
        )
        for time_s in np.linspace(
            0.0, 2.0 * math.pi / OMEGA_BODY, 73, endpoint=False
        ):
            source = canonical.control(float(time_s))
            target = flipped.control(float(time_s))
            unchanged = [*shoulder_r, *shoulder_l, *link_y, *link_p]
            flip_checks.append(
                bool(
                    np.allclose(target[unchanged], source[unchanged])
                    and np.allclose(target[leg_r], source[leg_l])
                    and np.allclose(target[leg_l], source[leg_r])
                    and np.array_equal(
                        flipped.stance_flags(float(time_s)),
                        canonical.stance_flags(float(time_s))[:, ::-1],
                    )
                )
            )

            phase_time_s = canonical._phase_time(float(time_s))
            shoulder_amplitude = math.radians(SHOULDER_HALF_AMPLITUDE_DEG)
            leg_offset = math.radians(LEG_OFFSET_DEG)
            up = math.radians(LEG_UP_DEG)
            down = math.radians(LEG_DOWN_DEG)
            for segment_index in range(n_segments):
                theta = (
                    OMEGA_BODY * phase_time_s
                    + segment_index * canonical.phi_body
                )
                yaw_wave = math.sin(theta + math.pi / 2.0)
                pitch_wave_r = math.sin(theta + 3.0 * math.pi / 2.0)
                pitch_wave_l = math.sin(
                    theta + 3.0 * math.pi / 2.0 + math.pi
                )
                expected_r = down if _step_is_down(pitch_wave_r) else up
                expected_l = down if _step_is_down(pitch_wave_l) else up
                body_ok = (
                    segment_index >= len(link_y)
                    or math.isclose(
                        float(source[link_y[segment_index]]),
                        BODY_WAVE_HALF_AMPLITUDE_RAD * math.sin(theta),
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                )
                coupling_checks.append(
                    bool(
                        body_ok
                        and math.isclose(
                            float(source[shoulder_r[segment_index]]),
                            leg_offset + shoulder_amplitude * yaw_wave,
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        )
                        and math.isclose(
                            float(source[shoulder_l[segment_index]]),
                            -leg_offset + shoulder_amplitude * yaw_wave,
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        )
                        and math.isclose(
                            float(source[leg_r[segment_index]]),
                            expected_r,
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        )
                        and math.isclose(
                            float(source[leg_l[segment_index]]),
                            expected_l,
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        )
                    )
                )
    return {
        "passed": bool(
            all(flip_checks)
            and all(definition_checks)
            and all(amplitude_checks)
            and all(coupling_checks)
            and all(step_checks)
        ),
        "flip_samples_checked": len(flip_checks),
        "coupling_samples_checked": len(coupling_checks),
        "body_yaw_unchanged": bool(all(flip_checks)),
        "leg_yaw_unchanged": bool(all(flip_checks)),
        "pitch_is_exact_lr_exchange": bool(all(flip_checks)),
        "absolute_wave_number_definition_verified": bool(all(definition_checks)),
        "body_wave_amplitude_0p4_rad_verified": bool(all(amplitude_checks)),
        "fixed_leg_yaw_phase_pi_over_2_verified": bool(all(coupling_checks)),
        "pitch_precursor_phase_3pi_over_2_verified": bool(all(coupling_checks)),
        "pitch_zero_crossing_step_verified": bool(
            all(coupling_checks) and all(step_checks)
        ),
        "left_right_pitch_antiphase_verified": bool(all(coupling_checks)),
    }


def wave_token(n_wave: float) -> str:
    return f"{float(n_wave):04.1f}".replace("-", "m").replace(".", "p")


def _trial_file_name(mode: str, n_wave: float, phase_offset_deg: float) -> str:
    phase_token = f"{phase_offset_deg:+07.2f}".replace("+", "p").replace("-", "m").replace(".", "p")
    return f"{mode}_Nwave{wave_token(n_wave)}_phase{phase_token}.json"


def run_trial(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    n_wave: float,
    phase_offset_deg: float,
    mode: str,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
    save_trajectory: bool,
    force: bool,
) -> dict:
    import mujoco

    record_path = output_dir / "trial_records" / _trial_file_name(
        mode, n_wave, phase_offset_deg
    )
    if record_path.is_file() and not force:
        return json.loads(record_path.read_text(encoding="utf-8"))

    prepared = prepare_simulation(
        repo_root,
        n_segments=n_segments,
        n_wave=n_wave,
        phase_offset_deg=phase_offset_deg,
        mode=mode,
        warmup_cycles=warmup_cycles,
    )
    model, data, controller = prepared.model, prepared.data, prepared.controller
    dt = prepared.dt
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    evaluation_steps = int(math.ceil(evaluation_cycles * gait_period_s / dt))
    evaluation_time_s = evaluation_steps * dt
    sample_stride = max(1, int(round(1.0 / (sample_hz * dt))))
    start_com = np.asarray(data.subtree_com[prepared.root_body_id]).copy()
    start_root_z = float(data.qpos[2])
    min_root_z = start_root_z
    min_root_up_z = _root_up_z(np.asarray(data.qpos))
    max_abs_roll_deg = 0.0
    max_abs_pitch_deg = 0.0
    max_abs_qacc = 0.0
    finite = True
    work_abs = 0.0
    work_positive = 0.0

    foot_positions = np.asarray(data.geom_xpos[prepared.foot_geom_ids]).copy()
    previous_contacts = _actual_contact_flags(model, data, prepared.body_map)
    slip_distance_m = 0.0
    longitudinal_slip_distance_m = 0.0
    lateral_slip_distance_m = 0.0
    persistent_contact_time_s = 0.0
    contact_count_sum = 0
    commanded_down_count = 0
    sample_count = 0
    contact_and_down_count = 0
    contact_and_up_count = 0
    down_count = 0
    up_count = 0

    sampled_time = []
    sampled_com = []
    sampled_stance = []
    sampled_contact = []
    sampled_slip_speed = []

    for step in range(evaluation_steps):
        evaluation_time_s_now = step * dt
        body_time_s = prepared.warmup_time_s + evaluation_time_s_now
        data.ctrl[:] = controller.control(body_time_s)
        mujoco.mj_step(model, data)

        stance = controller.stance_flags(body_time_s)
        contacts = _actual_contact_flags(model, data, prepared.body_map)
        current_positions = np.asarray(data.geom_xpos[prepared.foot_geom_ids]).copy()
        delta = current_positions - foot_positions
        persistent = contacts & previous_contacts
        tangential_speed = np.linalg.norm(delta[..., :2], axis=-1) / dt
        longitudinal_speed = np.abs(delta[..., 1]) / dt
        lateral_speed = np.abs(delta[..., 0]) / dt
        slip_distance_m += float(np.sum(tangential_speed[persistent])) * dt
        longitudinal_slip_distance_m += float(
            np.sum(longitudinal_speed[persistent])
        ) * dt
        lateral_slip_distance_m += float(np.sum(lateral_speed[persistent])) * dt
        persistent_contact_time_s += float(np.sum(persistent)) * dt
        foot_positions = current_positions
        previous_contacts = contacts

        contact_count_sum += int(np.sum(contacts))
        commanded_down_count += int(np.sum(stance))
        sample_count += contacts.size
        contact_and_down_count += int(np.sum(contacts & stance))
        contact_and_up_count += int(np.sum(contacts & ~stance))
        down_count += int(np.sum(stance))
        up_count += int(np.sum(~stance))

        power = np.asarray(data.actuator_force) * np.asarray(data.actuator_velocity)
        work_abs += float(np.sum(np.abs(power))) * dt
        work_positive += float(np.sum(np.maximum(power, 0.0))) * dt
        finite &= bool(
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

        if save_trajectory and (
            step % sample_stride == 0 or step == evaluation_steps - 1
        ):
            sampled_time.append((step + 1) * dt)
            sampled_com.append(
                np.asarray(data.subtree_com[prepared.root_body_id]).copy()
            )
            sampled_stance.append(stance.copy())
            sampled_contact.append(contacts.copy())
            sampled_slip_speed.append(tangential_speed.copy())

    final_com = np.asarray(data.subtree_com[prepared.root_body_id]).copy()
    displacement = final_com - start_com
    warnings = {
        str(index): int(data.warning[index].number)
        for index in range(len(data.warning))
        if int(data.warning[index].number) > 0
    }
    gates = {
        "model_loaded": True,
        "all_states_finite": finite,
        "no_mujoco_warnings": sum(warnings.values()) == 0,
        "never_inverted": min_root_up_z > 0.0,
        "trajectory_complete": evaluation_steps > 1,
    }
    result = {
        "schema_version": 1,
        "status": "PASS" if all(gates.values()) else "FAIL",
        "variant": NULL_VARIANT,
        "mode": mode,
        "mode_label": MODE_LABELS[mode],
        "n_segments": n_segments,
        "n_legs": 2 * n_segments,
        "N_wave": n_wave,
        "spatial_phase_increment_rad": controller.phi_body,
        "spatial_phase_increment_deg": math.degrees(controller.phi_body),
        "leg_yaw_body_phase_offset_rad": controller.phi_leg_yaw,
        "leg_yaw_body_phase_offset_deg": math.degrees(controller.phi_leg_yaw),
        "leg_pitch_body_phase_offset_rad": controller.phi_leg_pitch,
        "leg_pitch_body_phase_offset_deg": math.degrees(controller.phi_leg_pitch),
        "left_right_pitch_phase_offset_rad": LEFT_RIGHT_PITCH_PHASE_OFFSET_RAD,
        "pitch_step_rule": "up if sin(theta+3*pi/2)>0; down otherwise",
        "global_phase_offset_deg": phase_offset_deg,
        "pitch_lr_flipped": mode == MODE_FLIPPED,
        "body_yaw_unchanged_by_flip": True,
        "leg_yaw_unchanged_by_flip": True,
        "warmup_cycles": warmup_cycles,
        "evaluation_cycles": evaluation_cycles,
        "evaluation_time_s": evaluation_time_s,
        "body_wave_half_amplitude_rad": BODY_WAVE_HALF_AMPLITUDE_RAD,
        "body_wave_half_amplitude_deg": BODY_WAVE_HALF_AMPLITUDE_DEG,
        "forward_displacement_m": float(displacement[1]),
        "mean_forward_speed_m_s": float(displacement[1] / evaluation_time_s),
        "lateral_displacement_m": float(displacement[0]),
        "planar_displacement_m": float(np.linalg.norm(displacement[:2])),
        "mechanical_absolute_work_j": work_abs,
        "mechanical_positive_work_j": work_positive,
        "absolute_displacement_per_work_m_j": float(
            abs(displacement[1]) / work_abs if work_abs > 0 else math.nan
        ),
        "actual_contact_fraction": float(contact_count_sum / sample_count),
        "commanded_down_fraction": float(commanded_down_count / sample_count),
        "contact_given_down": float(contact_and_down_count / down_count),
        "contact_given_up": float(contact_and_up_count / up_count),
        "aggregate_contact_slip_distance_m": slip_distance_m,
        "mean_contact_slip_speed_m_s": float(
            slip_distance_m / persistent_contact_time_s
            if persistent_contact_time_s > 0
            else math.nan
        ),
        "mean_longitudinal_contact_slip_speed_m_s": float(
            longitudinal_slip_distance_m / persistent_contact_time_s
            if persistent_contact_time_s > 0
            else math.nan
        ),
        "mean_lateral_contact_slip_speed_m_s": float(
            lateral_slip_distance_m / persistent_contact_time_s
            if persistent_contact_time_s > 0
            else math.nan
        ),
        "slip_distance_per_leg_m": float(slip_distance_m / (2 * n_segments)),
        "persistent_contact_time_s": persistent_contact_time_s,
        "start_root_z_m": start_root_z,
        "min_root_z_m": min_root_z,
        "minimum_root_up_z": min_root_up_z,
        "max_abs_roll_deg": max_abs_roll_deg,
        "max_abs_pitch_deg": max_abs_pitch_deg,
        "max_abs_qacc": max_abs_qacc,
        "mujoco_warning_total": int(sum(warnings.values())),
        "mujoco_warning_counts": warnings,
        "gates": gates,
    }
    record_path.parent.mkdir(parents=True, exist_ok=True)
    record_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    if save_trajectory:
        trajectory_dir = output_dir / "trajectories"
        trajectory_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            trajectory_dir
            / _trial_file_name(mode, n_wave, phase_offset_deg).replace(".json", ".npz"),
            time_s=np.asarray(sampled_time),
            com_xyz_m=np.asarray(sampled_com),
            planned_stance=np.asarray(sampled_stance),
            actual_contact=np.asarray(sampled_contact),
            foot_slip_speed_m_s=np.asarray(sampled_slip_speed),
            start_com_xyz_m=start_com,
        )
    return result


def _worker(task: tuple) -> dict:
    (
        repo_root_text,
        output_dir_text,
        n_segments,
        n_wave,
        phase_offset_deg,
        mode,
        warmup_cycles,
        evaluation_cycles,
        sample_hz,
        save_trajectory,
        force,
    ) = task
    return run_trial(
        Path(repo_root_text),
        Path(output_dir_text),
        n_segments=n_segments,
        n_wave=n_wave,
        phase_offset_deg=phase_offset_deg,
        mode=mode,
        warmup_cycles=warmup_cycles,
        evaluation_cycles=evaluation_cycles,
        sample_hz=sample_hz,
        save_trajectory=save_trajectory,
        force=force,
    )


def _aggregate(rows: list[dict]) -> list[dict]:
    summary = []
    for mode in MODES:
        for n_wave in sorted({float(row["N_wave"]) for row in rows}):
            selected_all = [
                row
                for row in rows
                if row["mode"] == mode and float(row["N_wave"]) == n_wave
            ]
            selected = [row for row in selected_all if row["status"] == "PASS"]
            if not selected:
                continue
            values = np.asarray(
                [float(row["forward_displacement_m"]) for row in selected]
            )
            absolute_values = np.abs(values)
            slip = np.asarray(
                [float(row["mean_contact_slip_speed_m_s"]) for row in selected]
            )
            summary.append(
                {
                    "mode": mode,
                    "mode_label": MODE_LABELS[mode],
                    "N_wave": n_wave,
                    "spatial_phase_increment_rad": spatial_phase_increment(
                        n_wave, int(selected[0]["n_segments"])
                    ),
                    "spatial_phase_increment_deg": math.degrees(
                        spatial_phase_increment(n_wave, int(selected[0]["n_segments"]))
                    ),
                    "leg_yaw_body_phase_offset_deg": math.degrees(
                        LEG_YAW_BODY_PHASE_OFFSET_RAD
                    ),
                    "leg_pitch_body_phase_offset_deg": math.degrees(
                        LEG_PITCH_BODY_PHASE_OFFSET_RAD
                    ),
                    "n_phase_offsets": len(selected_all),
                    "n_pass": len(selected),
                    "mean_forward_displacement_m": float(np.mean(values)),
                    "sd_across_phase_offsets_m": float(
                        np.std(values, ddof=1) if values.size > 1 else 0.0
                    ),
                    "min_forward_displacement_m": float(np.min(values)),
                    "max_forward_displacement_m": float(np.max(values)),
                    "median_forward_displacement_m": float(np.median(values)),
                    "mean_absolute_forward_displacement_m": float(
                        np.mean(absolute_values)
                    ),
                    "min_absolute_forward_displacement_m": float(
                        np.min(absolute_values)
                    ),
                    "max_absolute_forward_displacement_m": float(
                        np.max(absolute_values)
                    ),
                    "mean_forward_speed_m_s": float(
                        np.mean([row["mean_forward_speed_m_s"] for row in selected])
                    ),
                    "mean_contact_slip_speed_m_s": float(np.mean(slip)),
                    "min_contact_slip_speed_m_s": float(np.min(slip)),
                    "max_contact_slip_speed_m_s": float(np.max(slip)),
                    "mean_slip_distance_per_leg_m": float(
                        np.mean([row["slip_distance_per_leg_m"] for row in selected])
                    ),
                    "mean_actual_contact_fraction": float(
                        np.mean([row["actual_contact_fraction"] for row in selected])
                    ),
                    "mean_contact_given_down": float(
                        np.mean([row["contact_given_down"] for row in selected])
                    ),
                    "mean_contact_given_up": float(
                        np.mean([row["contact_given_up"] for row in selected])
                    ),
                    "mean_mechanical_absolute_work_j": float(
                        np.mean([row["mechanical_absolute_work_j"] for row in selected])
                    ),
                    "positive_phase_fraction": float(np.mean(values > 0.0)),
                }
            )
    return summary


def _paired(rows: list[dict]) -> list[dict]:
    lookup = {
        (row["mode"], float(row["N_wave"]), float(row["global_phase_offset_deg"])): row
        for row in rows
        if row["status"] == "PASS"
    }
    paired = []
    for n_wave in sorted({float(row["N_wave"]) for row in rows}):
        phases = sorted(
            {
                float(row["global_phase_offset_deg"])
                for row in rows
                if float(row["N_wave"]) == n_wave
            }
        )
        for phase in phases:
            canonical = lookup.get((MODE_CANONICAL, n_wave, phase))
            flipped = lookup.get((MODE_FLIPPED, n_wave, phase))
            if canonical is None or flipped is None:
                continue
            paired.append(
                {
                    "N_wave": n_wave,
                    "global_phase_offset_deg": phase,
                    "canonical_displacement_m": canonical["forward_displacement_m"],
                    "flipped_displacement_m": flipped["forward_displacement_m"],
                    "canonical_minus_flipped_m": canonical["forward_displacement_m"]
                    - flipped["forward_displacement_m"],
                    "canonical_slip_speed_m_s": canonical["mean_contact_slip_speed_m_s"],
                    "flipped_slip_speed_m_s": flipped["mean_contact_slip_speed_m_s"],
                    "flipped_minus_canonical_slip_speed_m_s": flipped[
                        "mean_contact_slip_speed_m_s"
                    ]
                    - canonical["mean_contact_slip_speed_m_s"],
                }
            )
    return paired


def _directionality(summary: list[dict]) -> list[dict]:
    """Compare reversal magnitude and slip without assuming time-reversal symmetry."""
    lookup = {
        (row["mode"], float(row["N_wave"])): row
        for row in summary
    }
    comparison = []
    for n_wave in sorted(
        n_wave
        for mode, n_wave in lookup
        if mode == MODE_CANONICAL and (MODE_FLIPPED, n_wave) in lookup
    ):
        canonical = lookup[(MODE_CANONICAL, n_wave)]
        flipped = lookup[(MODE_FLIPPED, n_wave)]
        forward = float(canonical["mean_forward_displacement_m"])
        reverse = float(flipped["mean_forward_displacement_m"])
        canonical_slip = float(canonical["mean_contact_slip_speed_m_s"])
        flipped_slip = float(flipped["mean_contact_slip_speed_m_s"])
        comparison.append(
            {
                "N_wave": n_wave,
                "canonical_mean_displacement_m": forward,
                "flipped_mean_displacement_m": reverse,
                "pair_sum_m": forward + reverse,
                "reverse_to_forward_magnitude_ratio": (
                    abs(reverse) / abs(forward) if forward != 0.0 else math.nan
                ),
                "canonical_slip_speed_m_s": canonical_slip,
                "flipped_slip_speed_m_s": flipped_slip,
                "flipped_to_canonical_slip_speed_ratio": (
                    flipped_slip / canonical_slip
                    if canonical_slip > 0.0
                    else math.nan
                ),
                "canonical_actual_contact_fraction": canonical[
                    "mean_actual_contact_fraction"
                ],
                "flipped_actual_contact_fraction": flipped[
                    "mean_actual_contact_fraction"
                ],
            }
        )
    return comparison


def _plots(
    output_dir: Path,
    summary: list[dict],
    rows: list[dict],
    directionality: list[dict],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})

    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    for mode in MODES:
        selected = sorted(
            [row for row in summary if row["mode"] == mode],
            key=lambda row: float(row["N_wave"]),
        )
        x = np.asarray([row["N_wave"] for row in selected], dtype=float)
        mean = np.asarray(
            [row["mean_absolute_forward_displacement_m"] for row in selected]
        ) * 1000
        low = np.asarray(
            [row["min_absolute_forward_displacement_m"] for row in selected]
        ) * 1000
        high = np.asarray(
            [row["max_absolute_forward_displacement_m"] for row in selected]
        ) * 1000
        ax.errorbar(
            x,
            mean,
            yerr=np.vstack((mean - low, high - mean)),
            marker="o",
            capsize=3,
            linewidth=1.7,
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
        )
    ax.set_ylim(bottom=0.0)
    ax.set_xticks(sorted({float(row["N_wave"]) for row in summary}))
    ax.set_xlabel(r"Absolute spatial wave number $N_{wave}$")
    ax.set_ylabel(r"Forward-axis displacement magnitude $|D_y|$ (mm)")
    ax.set_title("Contact-pattern performance magnitude", weight="bold")
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(figure_dir / "absolute_displacement_vs_wave_number.png", dpi=260)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 4.4))
    for mode in MODES:
        selected = sorted(
            [row for row in summary if row["mode"] == mode],
            key=lambda row: float(row["N_wave"]),
        )
        ax.plot(
            [row["N_wave"] for row in selected],
            [1000 * row["mean_contact_slip_speed_m_s"] for row in selected],
            marker="o",
            linewidth=1.7,
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
        )
    ax.set_xticks(sorted({float(row["N_wave"]) for row in summary}))
    ax.set_xlabel(r"Absolute spatial wave number $N_{wave}$")
    ax.set_ylabel("Mean persistent-contact slip speed (mm/s)")
    ax.set_title("Pitch-phase flipping changes foot slip", weight="bold")
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(figure_dir / "contact_slip_speed_vs_N_wave.png", dpi=260)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 4.4))
    for mode in MODES:
        selected = sorted(
            [row for row in summary if row["mode"] == mode],
            key=lambda row: float(row["N_wave"]),
        )
        ax.plot(
            [row["N_wave"] for row in selected],
            [row["mean_actual_contact_fraction"] for row in selected],
            marker="o",
            linewidth=1.7,
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
        )
    ax.set_xticks(sorted({float(row["N_wave"]) for row in summary}))
    ax.set_xlabel(r"Absolute spatial wave number $N_{wave}$")
    ax.set_ylabel("Actual foot-contact fraction")
    ax.set_title("Measured contact occupancy", weight="bold")
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(figure_dir / "actual_contact_fraction_vs_N_wave.png", dpi=260)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(7.5, 6.2), sharex=True)
    x = [row["N_wave"] for row in directionality]
    displacement_ratio = [
        row["reverse_to_forward_magnitude_ratio"] for row in directionality
    ]
    slip_ratio = [
        row["flipped_to_canonical_slip_speed_ratio"] for row in directionality
    ]
    axes[0].plot(x, displacement_ratio, marker="o", color="#6d4c91", linewidth=1.7)
    axes[0].axhline(1.0, color="#42484c", linewidth=0.9, linestyle="--")
    axes[0].set_ylabel(r"$|D_{flip}|/D_{canonical}$")
    axes[0].set_title("Direction reversal is not a mechanical time reversal", weight="bold")
    axes[1].plot(x, slip_ratio, marker="o", color="#c4513f", linewidth=1.7)
    axes[1].axhline(1.0, color="#42484c", linewidth=0.9, linestyle="--")
    axes[1].set_ylabel("Flipped/canonical slip ratio")
    axes[1].set_xlabel(r"Absolute spatial wave number $N_{wave}$")
    axes[1].set_xticks(x)
    fig.tight_layout()
    fig.savefig(figure_dir / "directionality_asymmetry_vs_N_wave.png", dpi=260)
    plt.close(fig)

    n_wave_values = sorted({float(row["N_wave"]) for row in rows})
    phases = sorted({float(row["global_phase_offset_deg"]) for row in rows})
    fig, axes = plt.subplots(2, 1, figsize=(8.0, 5.8), sharex=True, sharey=True)
    finite_values = [1000 * float(row["forward_displacement_m"]) for row in rows if row["status"] == "PASS"]
    limit = max(abs(min(finite_values)), abs(max(finite_values)))
    images = []
    for ax, mode in zip(axes, MODES):
        lookup = {
            (float(row["N_wave"]), float(row["global_phase_offset_deg"])): 1000
            * float(row["forward_displacement_m"])
            for row in rows
            if row["mode"] == mode and row["status"] == "PASS"
        }
        matrix = np.asarray([[lookup[(n_wave, phase)] for n_wave in n_wave_values] for phase in phases])
        images.append(
            ax.imshow(
                matrix,
                aspect="auto",
                origin="lower",
                cmap="coolwarm",
                vmin=-limit,
                vmax=limit,
            )
        )
        ax.set_title(MODE_LABELS[mode])
        ax.set_yticks(range(len(phases)), [f"{phase:.0f}" for phase in phases])
        ax.set_ylabel("Global phase (deg)")
    axes[-1].set_xticks(
        range(len(n_wave_values)), [f"{value:g}" for value in n_wave_values]
    )
    axes[-1].set_xlabel(r"Absolute spatial wave number $N_{wave}$")
    fig.colorbar(images[-1], ax=axes, label="Signed displacement (mm)", shrink=0.88)
    fig.suptitle("Global-phase sensitivity", weight="bold")
    fig.subplots_adjust(left=0.12, right=0.88, top=0.88, bottom=0.10, hspace=0.28)
    fig.savefig(figure_dir / "phase_offset_displacement_map.png", dpi=260)
    plt.close(fig)

    # Controller-contract figure: continuous body/leg waves followed by the
    # robot-specific zero-crossing conversion of pitch into binary targets.
    theta = np.linspace(0.0, 2.0 * math.pi, 1201)
    body_wave = np.sin(theta)
    leg_yaw_wave = np.sin(theta + LEG_YAW_BODY_PHASE_OFFSET_RAD)
    pitch_wave_r = np.sin(theta + LEG_PITCH_BODY_PHASE_OFFSET_RAD)
    pitch_wave_l = np.sin(
        theta
        + LEG_PITCH_BODY_PHASE_OFFSET_RAD
        + LEFT_RIGHT_PITCH_PHASE_OFFSET_RAD
    )
    pitch_down_r = np.asarray(
        [_step_is_down(value) for value in pitch_wave_r], dtype=float
    )
    pitch_down_l = np.asarray(
        [_step_is_down(value) for value in pitch_wave_l], dtype=float
    )
    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.0), sharex=True)
    axes[0].plot(theta / math.pi, body_wave, linewidth=2.0, label="Body yaw")
    axes[0].plot(
        theta / math.pi,
        leg_yaw_wave,
        linewidth=2.0,
        label=r"Leg yaw ($+\pi/2$)",
    )
    axes[0].plot(
        theta / math.pi,
        pitch_wave_r,
        linewidth=2.0,
        label=r"Pitch precursor ($+3\pi/2$)",
    )
    axes[0].axhline(0.0, color="#555555", linewidth=0.9)
    axes[0].set_ylabel("Normalized waveform")
    axes[0].set_title("Fixed body-leg phase contract", weight="bold")
    axes[0].legend(loc="upper right", frameon=True, framealpha=0.9, ncol=1)
    axes[1].step(
        theta / math.pi,
        pitch_down_r,
        where="post",
        linewidth=2.0,
        label="Right pitch state",
    )
    axes[1].step(
        theta / math.pi,
        pitch_down_l,
        where="post",
        linewidth=2.0,
        label=r"Left pitch state ($+\pi$)",
    )
    axes[1].axhline(0.5, color="#777777", linewidth=0.9, linestyle="--")
    axes[1].set_yticks([0.0, 1.0], ["up", "down"])
    axes[1].set_xticks(
        [0.0, 0.5, 1.0, 1.5, 2.0],
        [r"$0$", r"$\pi/2$", r"$\pi$", r"$3\pi/2$", r"$2\pi$"],
    )
    axes[1].set_xlabel(r"Local body phase $\theta_i$")
    axes[1].set_ylabel("Binary pitch target")
    axes[1].legend(loc="center right", frameon=True, framealpha=0.9, ncol=1)
    fig.tight_layout()
    fig.savefig(figure_dir / "body_leg_phase_coupling_contract.png", dpi=260)
    plt.close(fig)


def _findings(
    output_dir: Path,
    summary: list[dict],
    paired: list[dict],
    directionality: list[dict],
    args,
) -> None:
    by_mode = {
        mode: sorted(
            [row for row in summary if row["mode"] == mode],
            key=lambda row: float(row["N_wave"]),
        )
        for mode in MODES
    }
    best_forward = max(
        by_mode[MODE_CANONICAL], key=lambda row: row["mean_forward_displacement_m"]
    )
    strongest_reverse = min(
        by_mode[MODE_FLIPPED], key=lambda row: row["mean_forward_displacement_m"]
    )
    canonical_positive = sum(
        row["mean_forward_displacement_m"] > 0 for row in by_mode[MODE_CANONICAL]
    )
    flipped_negative = sum(
        row["mean_forward_displacement_m"] < 0 for row in by_mode[MODE_FLIPPED]
    )
    slip_differences = np.asarray(
        [row["flipped_minus_canonical_slip_speed_m_s"] for row in paired]
    )
    weaker_reverse_with_more_slip = [
        row
        for row in directionality
        if row["reverse_to_forward_magnitude_ratio"] < 1.0
        and row["flipped_to_canonical_slip_speed_ratio"] > 1.0
    ]
    lines = [
        "# Step 30: fixed body-leg phase coupling and contact-phase flip",
        "",
        "## Definition",
        "",
        "The robot length is fixed. `N_wave` is the absolute number of complete ",
        "spatial waves over the head-to-tail segment span. This is migrated from ",
        "the live Centipede_IsaacLab_RL definition. For N segment-phase locations, ",
        "`phi_body=-2*pi*N_wave/(N-1)`. The negative sign retains the established ",
        "MuJoCo propagation direction. ",
        "For each segment phase `theta_i`, the leg-yaw waveform is ",
        "`sin(theta_i+pi/2)`. The leg-pitch precursor is ",
        "`sin(theta_i+3*pi/2)` and is converted to the robot's binary target ",
        "at zero using the robot convention: positive is up and nonpositive ",
        "is down. The left precursor ",
        "has an additional `pi` offset. In the flipped condition only the ",
        "left/right leg-pitch commands are exchanged; body and leg yaw are ",
        "bitwise unchanged.",
        "",
        "## Protocol",
        "",
        f"- Symmetric center-rooted null with N={args.n_segments} segments.",
        f"- N_wave values: {', '.join(str(v) for v in args.n_wave)}.",
        f"- Body-yaw half-amplitude: {BODY_WAVE_HALF_AMPLITUDE_RAD:.3f} rad "
        f"({BODY_WAVE_HALF_AMPLITUDE_DEG:.2f} deg), migrated from the live RL YAML.",
        "- Leg-yaw/body-yaw phase offset: pi/2 (fixed; independent of N_wave).",
        "- Leg-pitch precursor/body-yaw phase offset: 3*pi/2 (fixed), then zero-crossing quantization.",
        f"- Global CPG phases: {', '.join(f'{v:g}' for v in args.phase_offsets_deg)} deg.",
        f"- {args.warmup_cycles} unmeasured warm-up cycles and {args.evaluation_cycles} evaluated cycles.",
        "- Whiskers in the displacement figure are phase-ensemble minima/maxima, not IID confidence intervals.",
        "",
        "## Results",
        "",
        "| N_wave | Canonical mean (mm) | Canonical phase range (mm) | Flipped mean (mm) | Flipped phase range (mm) | Reverse/forward magnitude | Canonical slip (mm/s) | Flipped slip (mm/s) |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    canonical_lookup = {row["N_wave"]: row for row in by_mode[MODE_CANONICAL]}
    flipped_lookup = {row["N_wave"]: row for row in by_mode[MODE_FLIPPED]}
    directionality_lookup = {row["N_wave"]: row for row in directionality}
    for n_wave in sorted(canonical_lookup):
        c = canonical_lookup[n_wave]
        f = flipped_lookup[n_wave]
        d = directionality_lookup[n_wave]
        lines.append(
            f"| {n_wave} | {1000*c['mean_forward_displacement_m']:+.2f} | "
            f"[{1000*c['min_forward_displacement_m']:+.2f}, {1000*c['max_forward_displacement_m']:+.2f}] | "
            f"{1000*f['mean_forward_displacement_m']:+.2f} | "
            f"[{1000*f['min_forward_displacement_m']:+.2f}, {1000*f['max_forward_displacement_m']:+.2f}] | "
            f"{d['reverse_to_forward_magnitude_ratio']:.3f} | "
            f"{1000*c['mean_contact_slip_speed_m_s']:.2f} | {1000*f['mean_contact_slip_speed_m_s']:.2f} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            f"The canonical mean is positive at {canonical_positive}/{len(by_mode[MODE_CANONICAL])} tested absolute wave numbers. ",
            f"The flipped mean is negative at {flipped_negative}/{len(by_mode[MODE_FLIPPED])} wave numbers.",
            "",
            f"The largest canonical forward mean occurs at N_wave={best_forward['N_wave']} "
            f"({1000*best_forward['mean_forward_displacement_m']:+.2f} mm). The strongest flipped reverse mean occurs at "
            f"N_wave={strongest_reverse['N_wave']} ({1000*strongest_reverse['mean_forward_displacement_m']:+.2f} mm).",
            "",
            f"Across matched phase/wave-number pairs, flipping changes mean persistent-contact slip speed by "
            f"{1000*np.mean(slip_differences):+.2f} mm/s on average.",
            "",
            "A pitch-phase flip is not a full mechanical time reversal: shoulder yaw, body shape, impact history, ",
            "and frictional state are not reversed. Therefore unequal forward/reverse magnitudes are expected and ",
            "must be interpreted together with measured slip rather than as a violation of symmetry.",
            "",
            "Wave numbers where the flipped reverse magnitude is smaller and flipped slip is larger: "
            + (
                ", ".join(f"{row['N_wave']:g}" for row in weaker_reverse_with_more_slip)
                if weaker_reverse_with_more_slip
                else "none"
            )
            + ". This criterion is evaluated from the data rather than assumed.",
            "",
            "## Key artifacts",
            "",
            "- `wave_contact_flip_summary.csv`",
            "- `wave_contact_flip_trials.csv`",
            "- `paired_phase_comparison.csv`",
            "- `directionality_comparison.csv`",
            "- `control_contract.json`",
            "- `COUPLING_AND_WAVE_NUMBER_DEFINITION.md`",
            "- `figures/`",
            "- `demos/`",
        ]
    )
    (output_dir / "FINDINGS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> None:
    repo_root = _repo_root()
    output_dir = _output_dir(repo_root)
    if args.smoke:
        output_dir = output_dir / "smoke"
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = _model_path(repo_root, args.n_segments)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)

    contract = validate_control_contract(repo_root, args.n_segments)
    _write_json(output_dir / "control_contract.json", contract)
    if not contract["passed"]:
        raise RuntimeError("control-contract validation failed")

    tasks = []
    for n_wave in args.n_wave:
        for phase_offset_deg in args.phase_offsets_deg:
            for mode in MODES:
                tasks.append(
                    (
                        str(repo_root),
                        str(output_dir),
                        args.n_segments,
                        float(n_wave),
                        float(phase_offset_deg),
                        mode,
                        args.warmup_cycles,
                        args.evaluation_cycles,
                        args.sample_hz,
                        float(phase_offset_deg) == 0.0,
                        args.force,
                    )
                )
    rows = []
    if args.workers <= 1:
        for task in tasks:
            rows.append(_worker(task))
            print(
                f"{rows[-1]['mode']} N_wave={rows[-1]['N_wave']} phase={rows[-1]['global_phase_offset_deg']:+g}: "
                f"{1000*rows[-1]['forward_displacement_m']:+.2f} mm [{rows[-1]['status']}]",
                flush=True,
            )
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(_worker, task): task for task in tasks}
            for future in as_completed(futures):
                row = future.result()
                rows.append(row)
                print(
                    f"{row['mode']} N_wave={row['N_wave']} phase={row['global_phase_offset_deg']:+g}: "
                    f"{1000*row['forward_displacement_m']:+.2f} mm [{row['status']}]",
                    flush=True,
                )
    rows.sort(key=lambda row: (float(row["N_wave"]), float(row["global_phase_offset_deg"]), row["mode"]))
    summary = _aggregate(rows)
    paired = _paired(rows)
    directionality = _directionality(summary)
    _write_csv(output_dir / "wave_contact_flip_trials.csv", rows)
    _write_csv(output_dir / "wave_contact_flip_summary.csv", summary)
    _write_csv(output_dir / "paired_phase_comparison.csv", paired)
    _write_csv(output_dir / "directionality_comparison.csv", directionality)
    _plots(output_dir, summary, rows, directionality)
    _findings(output_dir, summary, paired, directionality, args)
    _write_json(
        output_dir / "run_configuration.json",
        {
            "n_segments": args.n_segments,
            "N_wave": list(args.n_wave),
            "N_wave_definition": "absolute full spatial waves over the head-to-tail segment span",
            "spatial_phase_increment_formula": "phi_body=-2*pi*N_wave/(N_segments-1)",
            "definition_source": (
                "Centipede_IsaacLab_RL/controllers/impedance_controller_torch.py "
                "and configs/farms_controller.yaml"
            ),
            "phase_offsets_deg": list(args.phase_offsets_deg),
            "modes": list(MODES),
            "warmup_cycles": args.warmup_cycles,
            "evaluation_cycles": args.evaluation_cycles,
            "sample_hz": args.sample_hz,
            "body_wave_half_amplitude_rad": BODY_WAVE_HALF_AMPLITUDE_RAD,
            "body_wave_half_amplitude_deg": BODY_WAVE_HALF_AMPLITUDE_DEG,
            "leg_yaw_body_phase_offset_rad": LEG_YAW_BODY_PHASE_OFFSET_RAD,
            "leg_yaw_body_phase_offset_deg": math.degrees(
                LEG_YAW_BODY_PHASE_OFFSET_RAD
            ),
            "leg_pitch_body_phase_offset_rad": LEG_PITCH_BODY_PHASE_OFFSET_RAD,
            "leg_pitch_body_phase_offset_deg": math.degrees(
                LEG_PITCH_BODY_PHASE_OFFSET_RAD
            ),
            "left_right_pitch_phase_offset_rad": LEFT_RIGHT_PITCH_PHASE_OFFSET_RAD,
            "pitch_step_rule": (
                "continuous precursor sin(theta+3*pi/2); up when >0, "
                "down when <=0"
            ),
            "flip_intervention": "exchange LEG_R and LEG_L pitch commands only",
            "model_path": str(model_path),
            "control_contract": contract,
        },
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n-segments", type=int, default=DEFAULT_N_SEGMENTS)
    parser.add_argument("--n-wave", type=float, nargs="+", default=DEFAULT_N_WAVE)
    parser.add_argument(
        "--phase-offsets-deg",
        type=float,
        nargs="+",
        default=DEFAULT_PHASE_OFFSETS_DEG,
    )
    parser.add_argument("--warmup-cycles", type=int, default=DEFAULT_WARMUP_CYCLES)
    parser.add_argument(
        "--evaluation-cycles", type=int, default=DEFAULT_EVALUATION_CYCLES
    )
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
