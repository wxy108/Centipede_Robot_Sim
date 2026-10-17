"""IID leg-pitch sweep over body-wave spatial wave number on the physical robot.

The measured-STL, center-rooted N=10 model is held fixed.  Leg-yaw joints are
locked at their neutral offsets.  The only time-varying commands are:

* a +/-15 degree body-yaw travelling wave; and
* IID Bernoulli(0.5) down/up commands for every physical leg in each T_g/2
  decision window.

The random stream includes the spatial wave number, so wave-number conditions
are independent rather than nested.  Actual contact is measured as an outcome;
the Bernoulli variable controls leg pitch, not a MuJoCo contact switch.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import numpy as np


if os.environ.get("CENTIPEDE_SKIP_OPTIONAL_MUJOCO_PLUGINS") == "1":
    # These models contain no MuJoCo plugin elements.  Some managed Windows
    # hosts block eager loading of optional bundled plugin DLLs; skip only
    # those DLLs while leaving all core MuJoCo libraries untouched.
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

import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    BRIDGE_TIME_S,
    DUTY_FACTOR,
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    LEG_UP_DEG,
    OMEGA_BODY,
    SETTLE_TIME_S,
    YAW_HALF_AMPLITUDE_DEG,
    _actual_contact_flags,
    _font,
    _leg_body_map,
    _root_roll_pitch_deg,
    _root_up_z,
    _set_leg_colors,
)


OUTPUT_NAME = "step33_physical_robot_pitch_body_yaw_iid_wave_sweep"
MODEL_VARIANT = "center_rooted_physical_robot"
N_SEGMENTS = 10
DEFAULT_N_WAVE = tuple(0.5 * value for value in range(1, 9))
PERIOD_SEED_OFFSET = 1_000_000
RNG_TAG = 33_010


def _repo_root() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def _model_path(repo_root: Path) -> Path:
    return (
        repo_root
        / "exp_data"
        / "reliability"
        / "step26_center_rooted_physical_robot"
        / "models"
        / MODEL_VARIANT
        / f"centipede_{N_SEGMENTS}seg_{MODEL_VARIANT}.xml"
    )


def _wave_tag(n_wave: float) -> str:
    return f"{float(n_wave):04.1f}".replace(".", "p")


def _wave_token(n_wave: float) -> int:
    token = int(round(1000.0 * float(n_wave)))
    if not math.isclose(token / 1000.0, float(n_wave), abs_tol=1.0e-12):
        raise ValueError("N_wave must be representable to 0.001")
    return token


def _spatial_increment(n_wave: float) -> float:
    """Joint-to-joint phase increment for N_wave waves over N-1 intervals."""

    return -2.0 * math.pi * float(n_wave) / float(N_SEGMENTS - 1)


def _build_actuator_indices(n_segments: int):
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = [], [], [], [], [], []
    actuator_index = 0
    for segment_index in range(n_segments):
        shoulder_r.append(actuator_index)
        leg_r.append(actuator_index + 1)
        shoulder_l.append(actuator_index + 2)
        leg_l.append(actuator_index + 3)
        actuator_index += 4
        if segment_index < n_segments - 1:
            link_y.append(actuator_index)
            link_p.append(actuator_index + 1)
            actuator_index += 2
    return tuple(np.asarray(values, dtype=int) for values in (
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p
    ))


def _body_only_compute_ctrl(
    t,
    n_ctrl,
    n_segments,
    indices,
    *,
    omega_spine=OMEGA_BODY,
    phi_spine=-math.pi,
    **_unused,
):
    """Minimal local equivalent of test_models.compute_ctrl for body yaw.

    Leg-yaw and leg-pitch entries are intentionally left at zero because the
    WaveNumberIIDController overwrites every one of them immediately.  Keeping
    this local avoids importing mujoco.viewer from the legacy demo module.
    """

    ctrl = np.zeros(int(n_ctrl), dtype=float)
    _shoulder_r, _leg_r, _shoulder_l, _leg_l, link_y, link_p = indices
    phase = omega_spine * float(t) + np.arange(n_segments - 1) * phi_spine
    ctrl[link_y] = math.radians(YAW_HALF_AMPLITUDE_DEG) * np.sin(phase)
    ctrl[link_p] = 0.0
    return ctrl


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


def _read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


class WaveNumberIIDController:
    """Body yaw at requested N_wave plus IID pitch; leg yaw is locked."""

    def __init__(self, n_ctrl: int, indices, compute_ctrl, *, seed: int, n_wave: float):
        self.n_segments = N_SEGMENTS
        self.n_ctrl = int(n_ctrl)
        self.indices = indices
        self.compute_ctrl = compute_ctrl
        self.seed = int(seed)
        self.n_wave = float(n_wave)
        self.wave_token = _wave_token(n_wave)
        self.phi_body = _spatial_increment(n_wave)
        self.gait_period_s = 2.0 * math.pi / OMEGA_BODY
        self.decision_window_s = 0.5 * self.gait_period_s
        self._state_cache: dict[int, np.ndarray] = {}

    def period_index(self, decision_time_s: float) -> int:
        return int(math.floor((float(decision_time_s) + 1.0e-12) / self.decision_window_s))

    def _draw_state(self, period_index: int) -> np.ndarray:
        encoded_period = int(period_index + PERIOD_SEED_OFFSET)
        if encoded_period < 0:
            raise ValueError("decision-time warmup exceeds seed offset")
        state = np.zeros((self.n_segments, 2), dtype=bool)
        for segment_index in range(self.n_segments):
            for side_index in range(2):
                rng = np.random.default_rng(
                    np.random.SeedSequence(
                        [
                            self.seed,
                            self.wave_token,
                            encoded_period,
                            segment_index,
                            side_index,
                            RNG_TAG,
                        ]
                    )
                )
                state[segment_index, side_index] = bool(rng.random() < 0.5)
        return state

    def stance_flags(self, decision_time_s: float) -> np.ndarray:
        period_index = self.period_index(decision_time_s)
        if period_index not in self._state_cache:
            self._state_cache[period_index] = self._draw_state(period_index)
        return self._state_cache[period_index].copy()

    def control(self, body_time_s: float, decision_time_s: float) -> np.ndarray:
        ctrl = self.compute_ctrl(
            body_time_s,
            self.n_ctrl,
            self.n_segments,
            self.indices,
            omega_spine=OMEGA_BODY,
            phi_spine=self.phi_body,
            dutyfactor=DUTY_FACTOR,
        )
        shoulder_r, leg_r, shoulder_l, leg_l, _link_y, link_p = self.indices
        neutral = math.radians(LEG_OFFSET_DEG)
        ctrl[shoulder_r] = neutral
        ctrl[shoulder_l] = -neutral
        ctrl[link_p] = 0.0
        stance = self.stance_flags(decision_time_s)
        down = math.radians(LEG_DOWN_DEG)
        up = math.radians(LEG_UP_DEG)
        ctrl[leg_r] = np.where(stance[:, 0], down, up)
        ctrl[leg_l] = np.where(stance[:, 1], down, up)
        return ctrl


@dataclass
class Assets:
    model: object
    indices: tuple
    compute_ctrl: object
    root_body_id: int
    stability_body_id: int
    body_map: dict[int, tuple[int, int]]
    dt: float


def _load_assets(repo_root: Path) -> Assets:
    import mujoco

    model_path = _model_path(repo_root)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    indices = _build_actuator_indices(N_SEGMENTS)
    expected_ctrl = 4 * N_SEGMENTS + 2 * (N_SEGMENTS - 1)
    if model.nu != expected_ctrl:
        raise RuntimeError(f"expected {expected_ctrl} actuators, found {model.nu}")
    root_body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root"))
    stability_body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Chasis1"))
    if root_body_id < 0 or stability_body_id < 0:
        raise KeyError("robot_root or Chasis1 missing")
    return Assets(
        model=model,
        indices=indices,
        compute_ctrl=_body_only_compute_ctrl,
        root_body_id=root_body_id,
        stability_body_id=stability_body_id,
        body_map=_leg_body_map(model, N_SEGMENTS, mujoco),
        dt=float(model.opt.timestep),
    )


def _initial_target(model, indices) -> np.ndarray:
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = indices
    ctrl = np.zeros(model.nu, dtype=float)
    neutral = math.radians(LEG_OFFSET_DEG)
    ctrl[shoulder_r] = neutral
    ctrl[shoulder_l] = -neutral
    ctrl[leg_r] = math.radians(LEG_DOWN_DEG)
    ctrl[leg_l] = math.radians(LEG_DOWN_DEG)
    ctrl[link_y] = 0.0
    ctrl[link_p] = 0.0
    return ctrl


def _prepare_data(
    assets: Assets, *, seed: int, n_wave: float, warmup_cycles: int
) -> tuple[object, WaveNumberIIDController, float]:
    import mujoco

    model = assets.model
    data = mujoco.MjData(model)
    controller = WaveNumberIIDController(
        model.nu, assets.indices, assets.compute_ctrl, seed=seed, n_wave=n_wave
    )
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    warmup_steps = int(math.ceil(warmup_cycles * gait_period_s / assets.dt))
    warmup_time_s = warmup_steps * assets.dt
    initial = _initial_target(model, assets.indices)

    settle_steps = int(math.ceil(SETTLE_TIME_S / assets.dt))
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial
        mujoco.mj_step(model, data)

    bridge_target = controller.control(0.0, -warmup_time_s)
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / assets.dt))
    for step in range(bridge_steps):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial + alpha * bridge_target
        mujoco.mj_step(model, data)

    for step in range(warmup_steps):
        body_time_s = step * assets.dt
        decision_time_s = body_time_s - warmup_time_s
        data.ctrl[:] = controller.control(body_time_s, decision_time_s)
        mujoco.mj_step(model, data)
    return data, controller, warmup_time_s


def _simulate_trial(
    assets: Assets,
    *,
    seed: int,
    n_wave: float,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
) -> dict:
    import mujoco

    data, controller, warmup_time_s = _prepare_data(
        assets, seed=seed, n_wave=n_wave, warmup_cycles=warmup_cycles
    )
    model = assets.model
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    evaluation_steps = int(math.ceil(evaluation_cycles * gait_period_s / assets.dt))
    evaluation_time_s = evaluation_steps * assets.dt
    sample_stride = max(1, int(round(1.0 / (sample_hz * assets.dt))))
    start_com = np.asarray(data.subtree_com[assets.root_body_id]).copy()
    start_root_z = float(data.xpos[assets.stability_body_id, 2])
    min_root_z = start_root_z
    min_root_up_z = float(data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2])
    max_abs_roll_deg = 0.0
    max_abs_pitch_deg = 0.0
    max_abs_qacc = 0.0
    all_finite = True
    mechanical_absolute_work_j = 0.0
    mechanical_positive_work_j = 0.0
    state_samples: list[np.ndarray] = []
    contact_samples: list[np.ndarray] = []
    window_indices: list[int] = []

    for step in range(evaluation_steps):
        evaluation_time = step * assets.dt
        body_time_s = warmup_time_s + evaluation_time
        data.ctrl[:] = controller.control(body_time_s, evaluation_time)
        mujoco.mj_step(model, data)
        power = np.asarray(data.actuator_force) * np.asarray(data.actuator_velocity)
        mechanical_absolute_work_j += float(np.sum(np.abs(power))) * assets.dt
        mechanical_positive_work_j += float(np.sum(np.maximum(power, 0.0))) * assets.dt
        all_finite &= bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and np.all(np.isfinite(data.qacc))
        )
        max_abs_qacc = max(max_abs_qacc, float(np.max(np.abs(data.qacc))))
        min_root_z = min(min_root_z, float(data.xpos[assets.stability_body_id, 2]))
        pose = np.zeros(7, dtype=float)
        pose[3:7] = np.asarray(data.xquat[assets.stability_body_id])
        roll_deg, pitch_deg = _root_roll_pitch_deg(pose)
        max_abs_roll_deg = max(max_abs_roll_deg, abs(roll_deg))
        max_abs_pitch_deg = max(max_abs_pitch_deg, abs(pitch_deg))
        min_root_up_z = min(
            min_root_up_z,
            float(data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2]),
        )
        if step % sample_stride == 0 or step == evaluation_steps - 1:
            state_samples.append(controller.stance_flags(evaluation_time))
            contact_samples.append(_actual_contact_flags(model, data, assets.body_map))
            window_indices.append(controller.period_index(evaluation_time))

    final_com = np.asarray(data.subtree_com[assets.root_body_id]).copy()
    displacement = final_com - start_com
    states = np.asarray(state_samples, dtype=bool)
    contacts = np.asarray(contact_samples, dtype=bool)
    flat_states = states.reshape(states.shape[0], -1).astype(float)
    mean_abs_corr, max_abs_corr = base._binary_pairwise_correlation(flat_states)
    down_count = int(np.sum(states))
    up_count = int(states.size - down_count)
    warning_counts = {
        str(index): int(data.warning[index].number)
        for index in range(len(data.warning))
        if int(data.warning[index].number) > 0
    }
    warning_total = int(sum(warning_counts.values()))
    window_state_constant = True
    indices_array = np.asarray(window_indices, dtype=int)
    for window_index in np.unique(indices_array):
        selected = states[indices_array == window_index]
        window_state_constant &= bool(np.all(selected == selected[0]))
    gates = {
        "model_loaded": True,
        "explicit_body_ground_contact_pairs": True,
        "all_states_finite": bool(all_finite),
        "no_mujoco_warnings": warning_total == 0,
        "never_inverted": min_root_up_z > 0.0,
        "window_state_constant": bool(window_state_constant),
    }
    return {
        "schema_version": 1,
        "status": "PASS" if all(gates.values()) else "FAIL",
        "model_variant": MODEL_VARIANT,
        "n_segments": N_SEGMENTS,
        "n_legs": 2 * N_SEGMENTS,
        "N_wave": float(n_wave),
        "spatial_phase_increment_rad": controller.phi_body,
        "seed": int(seed),
        "rng_wave_token": controller.wave_token,
        "rng_cross_wave_mode": "strict_independent",
        "leg_yaw_locked": True,
        "leg_yaw_right_target_deg": float(LEG_OFFSET_DEG),
        "leg_yaw_left_target_deg": float(-LEG_OFFSET_DEG),
        "warmup_cycles": int(warmup_cycles),
        "warmup_time_s": float(warmup_time_s),
        "evaluation_cycles": int(evaluation_cycles),
        "evaluation_time_s": float(evaluation_time_s),
        "decision_window_s": float(controller.decision_window_s),
        "decision_windows_per_gait_cycle": 2,
        "forward_displacement_m": float(displacement[1]),
        "absolute_forward_displacement_m": float(abs(displacement[1])),
        "lateral_displacement_m": float(displacement[0]),
        "planar_displacement_m": float(np.linalg.norm(displacement[:2])),
        "vertical_displacement_m": float(displacement[2]),
        "mechanical_absolute_work_j": mechanical_absolute_work_j,
        "mechanical_positive_work_j": mechanical_positive_work_j,
        "commanded_down_fraction": float(np.mean(states)),
        "both_down_pair_fraction": float(np.mean(np.all(states, axis=2))),
        "both_up_pair_fraction": float(np.mean(np.all(~states, axis=2))),
        "one_down_pair_fraction": float(np.mean(np.sum(states, axis=2) == 1)),
        "actual_contact_fraction": float(np.mean(contacts)),
        "mean_simultaneous_actual_contacts": float(np.mean(np.sum(contacts, axis=(1, 2)))),
        "contact_given_commanded_down": (
            float(np.sum(contacts & states) / down_count) if down_count else float("nan")
        ),
        "contact_given_commanded_up": (
            float(np.sum(contacts & ~states) / up_count) if up_count else float("nan")
        ),
        "mean_abs_pairwise_state_correlation": float(mean_abs_corr),
        "max_abs_pairwise_state_correlation": float(max_abs_corr),
        "start_root_z_m": start_root_z,
        "min_root_z_m": min_root_z,
        "minimum_root_up_z": min_root_up_z,
        "max_abs_roll_deg": max_abs_roll_deg,
        "max_abs_pitch_deg": max_abs_pitch_deg,
        "max_abs_qacc": max_abs_qacc,
        "mujoco_warning_total": warning_total,
        "mujoco_warning_counts": json.dumps(warning_counts, sort_keys=True),
        "gates": json.dumps(gates, sort_keys=True),
    }


def _worker_chunk(task: tuple) -> list[dict]:
    (
        repo_root_text,
        n_wave,
        seeds,
        warmup_cycles,
        evaluation_cycles,
        sample_hz,
    ) = task
    assets = _load_assets(Path(repo_root_text))
    return [
        _simulate_trial(
            assets,
            seed=int(seed),
            n_wave=float(n_wave),
            warmup_cycles=int(warmup_cycles),
            evaluation_cycles=int(evaluation_cycles),
            sample_hz=float(sample_hz),
        )
        for seed in seeds
    ]


def _build_open_loop_controls(
    assets: Assets,
    *,
    seeds: list[int],
    n_wave: float,
    warmup_cycles: int,
    evaluation_cycles: int,
) -> tuple[np.ndarray, int, float]:
    """Build controls matching the serial settle/bridge/warmup/evaluation law."""

    model = assets.model
    batch = len(seeds)
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    settle_steps = int(math.ceil(SETTLE_TIME_S / assets.dt))
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / assets.dt))
    warmup_steps = int(math.ceil(warmup_cycles * gait_period_s / assets.dt))
    evaluation_steps = int(math.ceil(evaluation_cycles * gait_period_s / assets.dt))
    warmup_time_s = warmup_steps * assets.dt
    evaluation_start = settle_steps + bridge_steps + warmup_steps
    total_steps = evaluation_start + evaluation_steps
    controls = np.zeros((batch, total_steps, model.nu), dtype=np.float64)
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = assets.indices
    del link_p
    neutral = math.radians(LEG_OFFSET_DEG)
    down = math.radians(LEG_DOWN_DEG)
    up = math.radians(LEG_UP_DEG)
    amplitude = math.radians(YAW_HALF_AMPLITUDE_DEG)
    phi = _spatial_increment(n_wave)

    # Straight, all-down symmetric settle.
    settle_alpha = (np.arange(settle_steps, dtype=float) + 1.0) / settle_steps
    batch_indices = np.arange(batch)
    settle_indices = np.arange(settle_steps)
    controls[np.ix_(batch_indices, settle_indices, shoulder_r)] = (
        neutral * settle_alpha[None, :, None]
    )
    controls[np.ix_(batch_indices, settle_indices, shoulder_l)] = (
        -neutral * settle_alpha[None, :, None]
    )
    controls[np.ix_(batch_indices, settle_indices, leg_r)] = (
        down * settle_alpha[None, :, None]
    )
    controls[np.ix_(batch_indices, settle_indices, leg_l)] = (
        down * settle_alpha[None, :, None]
    )

    # Body yaw is shared within one N_wave condition.
    bridge_phase = np.arange(N_SEGMENTS - 1, dtype=float) * phi
    bridge_body = amplitude * np.sin(bridge_phase)
    bridge_alpha = 0.5 - 0.5 * np.cos(
        math.pi * (np.arange(bridge_steps, dtype=float) + 1.0) / bridge_steps
    )
    bridge_slice = slice(settle_steps, settle_steps + bridge_steps)
    bridge_indices = np.arange(settle_steps, settle_steps + bridge_steps)
    controls[np.ix_(batch_indices, bridge_indices, shoulder_r)] = neutral
    controls[np.ix_(batch_indices, bridge_indices, shoulder_l)] = -neutral
    controls[np.ix_(batch_indices, bridge_indices, link_y)] = (
        bridge_alpha[None, :, None] * bridge_body[None, None, :]
    )

    dynamic_steps = warmup_steps + evaluation_steps
    warm_times = np.arange(warmup_steps, dtype=float) * assets.dt
    eval_times = warmup_time_s + np.arange(evaluation_steps, dtype=float) * assets.dt
    body_times = np.concatenate((warm_times, eval_times))
    dynamic_slice = slice(settle_steps + bridge_steps, total_steps)
    dynamic_indices_all = np.arange(settle_steps + bridge_steps, total_steps)
    controls[np.ix_(batch_indices, dynamic_indices_all, shoulder_r)] = neutral
    controls[np.ix_(batch_indices, dynamic_indices_all, shoulder_l)] = -neutral
    controls[np.ix_(batch_indices, dynamic_indices_all, link_y)] = amplitude * np.sin(
        OMEGA_BODY * body_times[None, :, None]
        + np.arange(N_SEGMENTS - 1, dtype=float)[None, None, :] * phi
    )

    decision_times = np.concatenate(
        (warm_times - warmup_time_s, np.arange(evaluation_steps, dtype=float) * assets.dt)
    )
    period_indices = np.floor(
        (decision_times + 1.0e-12) / (0.5 * gait_period_s)
    ).astype(int)
    unique_periods = np.unique(period_indices)
    for batch_index, seed in enumerate(seeds):
        controller = WaveNumberIIDController(
            model.nu,
            assets.indices,
            assets.compute_ctrl,
            seed=seed,
            n_wave=n_wave,
        )
        bridge_state = controller.stance_flags(-warmup_time_s)
        bridge_target_r = np.where(bridge_state[:, 0], down, up)
        bridge_target_l = np.where(bridge_state[:, 1], down, up)
        controls[batch_index][np.ix_(bridge_indices, leg_r)] = (
            (1.0 - bridge_alpha[:, None]) * down
            + bridge_alpha[:, None] * bridge_target_r[None, :]
        )
        controls[batch_index][np.ix_(bridge_indices, leg_l)] = (
            (1.0 - bridge_alpha[:, None]) * down
            + bridge_alpha[:, None] * bridge_target_l[None, :]
        )
        for period_index in unique_periods:
            state = controller._draw_state(int(period_index))
            mask = period_indices == period_index
            dynamic_indices = np.flatnonzero(mask) + settle_steps + bridge_steps
            controls[batch_index][np.ix_(dynamic_indices, leg_r)] = np.where(
                state[:, 0], down, up
            )[None, :]
            controls[batch_index][np.ix_(dynamic_indices, leg_l)] = np.where(
                state[:, 1], down, up
            )[None, :]
    return np.ascontiguousarray(controls), evaluation_start, warmup_time_s


def _simulate_rollout_batch(
    assets: Assets,
    rollout_engine,
    rollout_data: list,
    *,
    seeds: list[int],
    n_wave: float,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
) -> list[dict]:
    import mujoco

    controls, evaluation_start, warmup_time_s = _build_open_loop_controls(
        assets,
        seeds=seeds,
        n_wave=n_wave,
        warmup_cycles=warmup_cycles,
        evaluation_cycles=evaluation_cycles,
    )
    state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    nstate = mujoco.mj_stateSize(assets.model, state_spec)
    initial_data = mujoco.MjData(assets.model)
    initial = np.empty(nstate, dtype=float)
    mujoco.mj_getState(assets.model, initial_data, initial, state_spec)
    initial_states = np.repeat(initial[None, :], len(seeds), axis=0)
    states, _ = rollout_engine.rollout(
        assets.model,
        rollout_data,
        initial_states,
        controls,
        control_spec=mujoco.mjtState.mjSTATE_CTRL.value,
        chunk_size=1,
    )
    batch_warning_counts = {
        str(index): max(int(data.warning[index].number) for data in rollout_data)
        for index in range(len(rollout_data[0].warning))
        if max(int(data.warning[index].number) for data in rollout_data) > 0
    }
    batch_warning_total = int(sum(batch_warning_counts.values()))
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    evaluation_steps = states.shape[1] - evaluation_start
    evaluation_time_s = evaluation_steps * assets.dt
    sample_stride = max(1, int(round(1.0 / (sample_hz * assets.dt))))
    sample_indices = np.arange(evaluation_start, states.shape[1], sample_stride, dtype=int)
    if sample_indices[-1] != states.shape[1] - 1:
        sample_indices = np.append(sample_indices, states.shape[1] - 1)
    rows = []
    metric_data = mujoco.MjData(assets.model)
    for batch_index, seed in enumerate(seeds):
        controller = WaveNumberIIDController(
            assets.model.nu,
            assets.indices,
            assets.compute_ctrl,
            seed=seed,
            n_wave=n_wave,
        )

        def load(snapshot: np.ndarray) -> None:
            mujoco.mj_setState(assets.model, metric_data, snapshot, state_spec)
            mujoco.mj_forward(assets.model, metric_data)

        load(states[batch_index, evaluation_start - 1])
        start_com = np.asarray(metric_data.subtree_com[assets.root_body_id]).copy()
        start_root_z = float(metric_data.xpos[assets.stability_body_id, 2])
        min_root_z = start_root_z
        min_root_up_z = float(metric_data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2])
        max_abs_roll_deg = 0.0
        max_abs_pitch_deg = 0.0
        contacts = []
        planned = []
        for state_index in sample_indices:
            load(states[batch_index, state_index])
            eval_step = state_index - evaluation_start
            evaluation_time = eval_step * assets.dt
            planned.append(controller.stance_flags(evaluation_time))
            contacts.append(_actual_contact_flags(assets.model, metric_data, assets.body_map))
            min_root_z = min(min_root_z, float(metric_data.xpos[assets.stability_body_id, 2]))
            min_root_up_z = min(
                min_root_up_z,
                float(metric_data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2]),
            )
            pose = np.zeros(7, dtype=float)
            pose[3:7] = np.asarray(metric_data.xquat[assets.stability_body_id])
            roll_deg, pitch_deg = _root_roll_pitch_deg(pose)
            max_abs_roll_deg = max(max_abs_roll_deg, abs(roll_deg))
            max_abs_pitch_deg = max(max_abs_pitch_deg, abs(pitch_deg))
        final_com = np.asarray(metric_data.subtree_com[assets.root_body_id]).copy()
        displacement = final_com - start_com
        planned_array = np.asarray(planned, dtype=bool)
        contact_array = np.asarray(contacts, dtype=bool)
        down_count = int(np.sum(planned_array))
        up_count = int(planned_array.size - down_count)
        flat_states = planned_array.reshape(planned_array.shape[0], -1).astype(float)
        mean_abs_corr, max_abs_corr = base._binary_pairwise_correlation(flat_states)
        all_finite = bool(np.all(np.isfinite(states[batch_index])))
        gates = {
            "model_loaded": True,
            "explicit_body_ground_contact_pairs": True,
            "all_states_finite": all_finite,
            "no_mujoco_warnings": batch_warning_total == 0,
            "never_inverted_at_sampled_states": min_root_up_z > 0.0,
            "open_loop_rollout_complete": states.shape[1] == controls.shape[1],
        }
        rows.append(
            {
                "schema_version": 2,
                "simulation_engine": "mujoco.rollout exact open-loop integration",
                "status": "PASS" if all(gates.values()) else "FAIL",
                "model_variant": MODEL_VARIANT,
                "n_segments": N_SEGMENTS,
                "n_legs": 2 * N_SEGMENTS,
                "N_wave": float(n_wave),
                "spatial_phase_increment_rad": _spatial_increment(n_wave),
                "seed": int(seed),
                "rng_wave_token": _wave_token(n_wave),
                "rng_cross_wave_mode": "strict_independent",
                "leg_yaw_locked": True,
                "leg_yaw_right_target_deg": float(LEG_OFFSET_DEG),
                "leg_yaw_left_target_deg": float(-LEG_OFFSET_DEG),
                "warmup_cycles": int(warmup_cycles),
                "warmup_time_s": float(warmup_time_s),
                "evaluation_cycles": int(evaluation_cycles),
                "evaluation_time_s": float(evaluation_time_s),
                "decision_window_s": float(0.5 * gait_period_s),
                "decision_windows_per_gait_cycle": 2,
                "integration_timestep_s": float(assets.dt),
                "contact_audit_sample_hz": float(sample_hz),
                "forward_displacement_m": float(displacement[1]),
                "absolute_forward_displacement_m": float(abs(displacement[1])),
                "lateral_displacement_m": float(displacement[0]),
                "planar_displacement_m": float(np.linalg.norm(displacement[:2])),
                "vertical_displacement_m": float(displacement[2]),
                "commanded_down_fraction": float(np.mean(planned_array)),
                "both_down_pair_fraction": float(np.mean(np.all(planned_array, axis=2))),
                "both_up_pair_fraction": float(np.mean(np.all(~planned_array, axis=2))),
                "one_down_pair_fraction": float(np.mean(np.sum(planned_array, axis=2) == 1)),
                "actual_contact_fraction": float(np.mean(contact_array)),
                "mean_simultaneous_actual_contacts": float(
                    np.mean(np.sum(contact_array, axis=(1, 2)))
                ),
                "contact_given_commanded_down": (
                    float(np.sum(contact_array & planned_array) / down_count)
                    if down_count else float("nan")
                ),
                "contact_given_commanded_up": (
                    float(np.sum(contact_array & ~planned_array) / up_count)
                    if up_count else float("nan")
                ),
                "mean_abs_pairwise_state_correlation": float(mean_abs_corr),
                "max_abs_pairwise_state_correlation": float(max_abs_corr),
                "start_root_z_m": start_root_z,
                "min_root_z_m": min_root_z,
                "minimum_root_up_z": min_root_up_z,
                "max_abs_roll_deg": max_abs_roll_deg,
                "max_abs_pitch_deg": max_abs_pitch_deg,
                "mujoco_warning_total": batch_warning_total,
                "mujoco_warning_counts": json.dumps(batch_warning_counts, sort_keys=True),
                "gates": json.dumps(gates, sort_keys=True),
            }
        )
    return rows


def _bootstrap_ci(values: np.ndarray, seed: int) -> tuple[float, float]:
    random = np.random.default_rng(seed)
    means = np.mean(
        random.choice(values, size=(5000, values.size), replace=True), axis=1
    )
    return float(np.quantile(means, 0.025)), float(np.quantile(means, 0.975))


def _aggregate(trials: list[dict], waves: list[float]) -> list[dict]:
    rows = []
    for index, n_wave in enumerate(waves):
        selected_all = [row for row in trials if math.isclose(float(row["N_wave"]), n_wave)]
        selected = [row for row in selected_all if row["status"] == "PASS"]
        if not selected:
            continue
        values = np.asarray([float(row["forward_displacement_m"]) for row in selected])
        absolute = np.abs(values)
        low, high = _bootstrap_ci(values, 33_100 + index)
        abs_low, abs_high = _bootstrap_ci(absolute, 33_200 + index)
        rows.append(
            {
                "N_wave": n_wave,
                "spatial_phase_increment_rad": _spatial_increment(n_wave),
                "n_trials": len(selected_all),
                "n_pass": len(selected),
                "pass_fraction": len(selected) / len(selected_all),
                "mean_forward_displacement_m": float(np.mean(values)),
                "sd_forward_displacement_m": float(np.std(values, ddof=1)),
                "bootstrap_ci95_low_m": low,
                "bootstrap_ci95_high_m": high,
                "median_forward_displacement_m": float(np.median(values)),
                "q05_forward_displacement_m": float(np.quantile(values, 0.05)),
                "q95_forward_displacement_m": float(np.quantile(values, 0.95)),
                "mean_absolute_forward_displacement_m": float(np.mean(absolute)),
                "bootstrap_abs_ci95_low_m": abs_low,
                "bootstrap_abs_ci95_high_m": abs_high,
                "positive_fraction": float(np.mean(values > 0.0)),
                "mean_commanded_down_fraction": float(
                    np.mean([float(row["commanded_down_fraction"]) for row in selected])
                ),
                "mean_both_down_pair_fraction": float(
                    np.mean([float(row["both_down_pair_fraction"]) for row in selected])
                ),
                "mean_both_up_pair_fraction": float(
                    np.mean([float(row["both_up_pair_fraction"]) for row in selected])
                ),
                "mean_one_down_pair_fraction": float(
                    np.mean([float(row["one_down_pair_fraction"]) for row in selected])
                ),
                "mean_actual_contact_fraction": float(
                    np.mean([float(row["actual_contact_fraction"]) for row in selected])
                ),
                "mean_contact_given_down": float(
                    np.mean([float(row["contact_given_commanded_down"]) for row in selected])
                ),
                "mean_contact_given_up": float(
                    np.mean([float(row["contact_given_commanded_up"]) for row in selected])
                ),
            }
        )
    return rows


def _plot_results(output_dir: Path, trials: list[dict], summary: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    waves = np.asarray([float(row["N_wave"]) for row in summary])
    means = np.asarray([float(row["mean_forward_displacement_m"]) for row in summary])
    low = np.asarray([float(row["bootstrap_ci95_low_m"]) for row in summary])
    high = np.asarray([float(row["bootstrap_ci95_high_m"]) for row in summary])
    abs_mean = np.asarray([float(row["mean_absolute_forward_displacement_m"]) for row in summary])
    abs_low = np.asarray([float(row["bootstrap_abs_ci95_low_m"]) for row in summary])
    abs_high = np.asarray([float(row["bootstrap_abs_ci95_high_m"]) for row in summary])

    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    ax.errorbar(waves, means, yerr=np.vstack((means - low, high - means)), fmt="o-", capsize=4, color="#246b79")
    ax.axhline(0.0, color="#777777", linewidth=0.9)
    ax.set(xlabel=r"Spatial wave number, $N_{\mathrm{wave}}$", ylabel="Mean signed forward displacement (m)")
    ax.set_xticks(waves)
    ax.grid(alpha=0.22)
    fig.tight_layout()
    fig.savefig(figure_dir / "mean_signed_displacement_vs_Nwave.png", dpi=240)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    ax.errorbar(waves, abs_mean, yerr=np.vstack((abs_mean - abs_low, abs_high - abs_mean)), fmt="o-", capsize=4, color="#b45a36")
    ax.set(xlabel=r"Spatial wave number, $N_{\mathrm{wave}}$", ylabel="Mean absolute forward displacement (m)")
    ax.set_xticks(waves)
    ax.grid(alpha=0.22)
    fig.tight_layout()
    fig.savefig(figure_dir / "mean_absolute_displacement_vs_Nwave.png", dpi=240)
    plt.close(fig)

    passed = [row for row in trials if row["status"] == "PASS"]
    all_values = np.asarray([float(row["forward_displacement_m"]) for row in passed])
    bins = np.linspace(float(np.min(all_values)), float(np.max(all_values)), 35)
    fig, axes = plt.subplots(2, 4, figsize=(12.2, 6.0), sharex=True, sharey=True)
    for axis, n_wave in zip(axes.flat, waves):
        values = np.asarray([
            float(row["forward_displacement_m"])
            for row in passed
            if math.isclose(float(row["N_wave"]), n_wave)
        ])
        axis.hist(values, bins=bins, density=True, color="#6f9caf", alpha=0.82)
        axis.axvline(float(np.mean(values)), color="#202020", linewidth=1.5)
        axis.axvline(0.0, color="#777777", linewidth=0.8)
        axis.set_title(rf"$N_{{\mathrm{{wave}}}}={n_wave:g}$")
        axis.grid(alpha=0.15)
    fig.supxlabel("Signed forward displacement (m)")
    fig.supylabel("Empirical density")
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_displacement_histograms.png", dpi=240)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 4.6))
    colors = plt.cm.viridis(np.linspace(0.08, 0.92, len(waves)))
    for color, n_wave in zip(colors, waves):
        values = np.sort(np.asarray([
            float(row["forward_displacement_m"])
            for row in passed
            if math.isclose(float(row["N_wave"]), n_wave)
        ]))
        probability = np.arange(1, values.size + 1) / values.size
        # Empirical CDFs are step functions, not interpolated densities.
        ax.step(
            np.r_[values[0], values], np.r_[0.0, probability], where="post",
            label=rf"$N_{{\mathrm{{wave}}}}={n_wave:g}$", color=color,
        )
    ax.axvline(0.0, color="#777777", linewidth=0.8)
    ax.set(xlabel="Signed forward displacement (m)", ylabel="Empirical cumulative probability")
    ax.grid(alpha=0.18)
    ax.legend(ncol=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_displacement_ecdf.png", dpi=240)
    plt.close(fig)

    contacts = np.asarray([float(row["mean_actual_contact_fraction"]) for row in summary])
    down = np.asarray([float(row["mean_contact_given_down"]) for row in summary])
    up = np.asarray([float(row["mean_contact_given_up"]) for row in summary])
    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    ax.plot(waves, contacts, "o-", label="All legs", color="#555555")
    ax.plot(waves, down, "o-", label="Given pitch down", color="#27856f")
    ax.plot(waves, up, "o-", label="Given pitch up", color="#d15b41")
    ax.set(xlabel=r"Spatial wave number, $N_{\mathrm{wave}}$", ylabel="Measured contact fraction", ylim=(0.0, 1.0))
    ax.set_xticks(waves)
    ax.grid(alpha=0.2)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(figure_dir / "measured_contact_vs_Nwave.png", dpi=240)
    plt.close(fig)


def _semantic_validation(waves: list[float], replays: int, evaluation_cycles: int) -> dict:
    n_windows = 2 * int(evaluation_cycles)
    rows = []
    streams = {}
    dummy_indices = (
        np.arange(0, 4 * N_SEGMENTS, 4),
        np.arange(1, 4 * N_SEGMENTS, 4),
        np.arange(2, 4 * N_SEGMENTS, 4),
        np.arange(3, 4 * N_SEGMENTS, 4),
        np.arange(4 * N_SEGMENTS, 4 * N_SEGMENTS + 2 * (N_SEGMENTS - 1), 2),
        np.arange(4 * N_SEGMENTS + 1, 4 * N_SEGMENTS + 2 * (N_SEGMENTS - 1), 2),
    )

    def dummy_ctrl(_t, n_ctrl, *_args, **_kwargs):
        return np.zeros(n_ctrl)

    for n_wave in waves:
        samples = []
        for seed in range(replays):
            controller = WaveNumberIIDController(
                4 * N_SEGMENTS + 2 * (N_SEGMENTS - 1),
                dummy_indices,
                dummy_ctrl,
                seed=seed,
                n_wave=n_wave,
            )
            samples.extend(controller._draw_state(index).ravel() for index in range(n_windows))
        matrix = np.asarray(samples, dtype=float)
        correlation = np.corrcoef(matrix, rowvar=False)
        off_diagonal = correlation[~np.eye(correlation.shape[0], dtype=bool)]
        packed = np.packbits(matrix.astype(np.uint8), axis=None).tobytes()
        streams[n_wave] = packed
        probability_tolerance = max(0.01, 4.0 * math.sqrt(0.25 / matrix.size))
        rows.append(
            {
                "N_wave": n_wave,
                "N_segments": N_SEGMENTS,
                "N_legs": 2 * N_SEGMENTS,
                "trials": replays,
                "windows_per_trial": n_windows,
                "draw_count": int(matrix.size),
                "empirical_down_probability": float(np.mean(matrix)),
                "four_sigma_probability_tolerance": probability_tolerance,
                "max_abs_leg_pair_correlation": float(np.max(np.abs(off_diagonal))),
                "phi_matches_definition": math.isclose(
                    _spatial_increment(n_wave) * (N_SEGMENTS - 1),
                    -2.0 * math.pi * n_wave,
                    abs_tol=1.0e-12,
                ),
            }
        )
    cross_wave_streams_differ = all(
        streams[a] != streams[b]
        for i, a in enumerate(waves)
        for b in waves[i + 1 :]
    )
    passed = bool(
        cross_wave_streams_differ
        and all(row["phi_matches_definition"] for row in rows)
        and all(
            abs(row["empirical_down_probability"] - 0.5)
            < row["four_sigma_probability_tolerance"]
            for row in rows
        )
    )
    return {
        "passed": passed,
        "process": "Bernoulli(0.5) independently across wave, trial, window, segment, and side",
        "cross_wave_streams_differ": cross_wave_streams_differ,
        "rows": rows,
    }


def _control_contract_validation(repo_root: Path, waves: list[float]) -> dict:
    del repo_root
    n_ctrl = 4 * N_SEGMENTS + 2 * (N_SEGMENTS - 1)
    indices = _build_actuator_indices(N_SEGMENTS)
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = indices
    neutral = math.radians(LEG_OFFSET_DEG)
    up = math.radians(LEG_UP_DEG)
    down = math.radians(LEG_DOWN_DEG)
    amplitude = math.radians(YAW_HALF_AMPLITUDE_DEG)
    rows = []
    for n_wave in waves:
        controller = WaveNumberIIDController(
            n_ctrl, indices, _body_only_compute_ctrl, seed=0, n_wave=n_wave
        )
        yaw_locked = True
        pitch_binary = True
        pitch_matches_state = True
        body_yaw_matches = True
        body_pitch_locked = True
        window_constant = True
        for time_s in np.linspace(0.0, controller.gait_period_s, 65, endpoint=False):
            ctrl = controller.control(float(time_s), float(time_s))
            stance = controller.stance_flags(float(time_s))
            yaw_locked &= bool(
                np.allclose(ctrl[shoulder_r], neutral)
                and np.allclose(ctrl[shoulder_l], -neutral)
            )
            actual_pitch = np.column_stack((ctrl[leg_r], ctrl[leg_l]))
            pitch_binary &= bool(
                np.all(np.isclose(actual_pitch, up) | np.isclose(actual_pitch, down))
            )
            pitch_matches_state &= bool(
                np.allclose(actual_pitch, np.where(stance, down, up))
            )
            expected_body = amplitude * np.sin(
                OMEGA_BODY * float(time_s)
                + np.arange(N_SEGMENTS - 1) * controller.phi_body
            )
            body_yaw_matches &= bool(np.allclose(ctrl[link_y], expected_body))
            body_pitch_locked &= bool(np.allclose(ctrl[link_p], 0.0))
            epsilon = 0.1 * controller.decision_window_s
            first = controller.stance_flags(float(time_s))
            second = controller.stance_flags(
                controller.period_index(float(time_s)) * controller.decision_window_s
                + epsilon
            )
            window_constant &= bool(np.array_equal(first, second))
        passed = bool(
            yaw_locked
            and pitch_binary
            and pitch_matches_state
            and body_yaw_matches
            and body_pitch_locked
            and window_constant
        )
        rows.append(
            {
                "N_wave": n_wave,
                "leg_yaw_locked": yaw_locked,
                "leg_pitch_is_binary": pitch_binary,
                "leg_pitch_matches_IID_state": pitch_matches_state,
                "body_yaw_matches_requested_wave": body_yaw_matches,
                "body_pitch_locked": body_pitch_locked,
                "decision_state_constant_within_window": window_constant,
                "passed": passed,
            }
        )
    return {"passed": all(row["passed"] for row in rows), "rows": rows}


def _render_demos(
    repo_root: Path,
    output_dir: Path,
    trials: list[dict],
    waves: list[float],
    *,
    warmup_cycles: int,
    duration_s: float,
    fps: int,
) -> list[dict]:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    assets = _load_assets(repo_root)
    demo_dir = output_dir / "demos"
    demo_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for n_wave in waves:
        candidates = [
            row for row in trials
            if row["status"] == "PASS" and math.isclose(float(row["N_wave"]), n_wave)
        ]
        median = float(np.median([float(row["forward_displacement_m"]) for row in candidates]))
        chosen = min(candidates, key=lambda row: (abs(float(row["forward_displacement_m"]) - median), int(row["seed"])))
        seed = int(chosen["seed"])
        data, controller, warmup_time_s = _prepare_data(
            assets, seed=seed, n_wave=n_wave, warmup_cycles=warmup_cycles
        )
        width, height = 960, 600
        assets.model.vis.global_.offwidth = max(assets.model.vis.global_.offwidth, width)
        assets.model.vis.global_.offheight = max(assets.model.vis.global_.offheight, height)
        renderer = mujoco.Renderer(assets.model, height=height, width=width)
        camera, option = base._camera_for(assets.model, assets.root_body_id, N_SEGMENTS, mujoco)
        start = np.asarray(data.subtree_com[assets.root_body_id]).copy()
        tag = _wave_tag(n_wave)
        video_path = demo_dir / f"Nwave{tag}_body_yaw_iid_pitch_seed{seed:03d}.mp4"
        preview_path = demo_dir / f"Nwave{tag}_body_yaw_iid_pitch_seed{seed:03d}_preview.png"
        writer = imageio.get_writer(
            str(video_path), fps=fps, codec="libx264", quality=8,
            pixelformat="yuv420p", macro_block_size=2,
        )
        steps = int(math.ceil(duration_s / assets.dt))
        next_frame_s = 1.0 / fps
        frame_count = 0
        preview_written = False
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, _link_p = assets.indices
        yaw_actuators = np.r_[shoulder_r, shoulder_l]
        pitch_actuators = np.r_[leg_r, leg_l]
        yaw_qpos = assets.model.jnt_qposadr[assets.model.actuator_trnid[yaw_actuators, 0]]
        pitch_qpos = assets.model.jnt_qposadr[assets.model.actuator_trnid[pitch_actuators, 0]]
        body_qpos = assets.model.jnt_qposadr[assets.model.actuator_trnid[link_y, 0]]
        max_yaw_error_rad = 0.0
        pitch_squared_error = 0.0
        body_squared_error = 0.0
        try:
            for step in range(steps):
                evaluation_time_s = step * assets.dt
                body_time_s = warmup_time_s + evaluation_time_s
                data.ctrl[:] = controller.control(body_time_s, evaluation_time_s)
                mujoco.mj_step(assets.model, data)
                max_yaw_error_rad = max(max_yaw_error_rad, float(np.max(np.abs(
                    data.qpos[yaw_qpos] - data.ctrl[yaw_actuators]
                ))))
                pitch_squared_error += float(np.mean(np.square(
                    data.qpos[pitch_qpos] - data.ctrl[pitch_actuators]
                )))
                body_squared_error += float(np.mean(np.square(
                    data.qpos[body_qpos] - data.ctrl[link_y]
                )))
                time_end_s = (step + 1) * assets.dt
                if time_end_s + 0.5 * assets.dt < next_frame_s:
                    continue
                stance = controller.stance_flags(evaluation_time_s)
                contact = _actual_contact_flags(assets.model, data, assets.body_map)
                _set_leg_colors(assets.model, assets.body_map, stance)
                renderer.update_scene(data, camera=camera, scene_option=option)
                image = Image.fromarray(renderer.render().copy())
                canvas = Image.new("RGB", (width, height + 82), "#eef1f2")
                canvas.paste(image, (0, 44))
                draw = ImageDraw.Draw(canvas)
                draw.text(
                    (16, 9),
                    rf"N_wave={n_wave:g} | Body yaw + IID pitch | Leg yaw fixed",
                    font=_font(20, bold=True), fill="#17202a",
                )
                displacement = float(data.subtree_com[assets.root_body_id][1] - start[1])
                draw.text(
                    (16, height + 51),
                    f"t={time_end_s:4.2f} s   signed forward displacement={displacement:+.3f} m   down/contact={int(np.sum(stance))}/{int(np.sum(contact))}",
                    font=_font(16), fill="#273746",
                )
                draw.ellipse((width - 210, 14, width - 194, 30), fill="#2aa657")
                draw.text((width - 187, 9), "down", font=_font(15), fill="#17202a")
                draw.ellipse((width - 105, 14, width - 89, 30), fill="#e6452b")
                draw.text((width - 82, 9), "up", font=_font(15), fill="#17202a")
                writer.append_data(np.asarray(canvas))
                frame_count += 1
                if not preview_written and time_end_s >= 0.5 * duration_s:
                    canvas.save(preview_path)
                    preview_written = True
                next_frame_s += 1.0 / fps
        finally:
            writer.close()
            renderer.close()
        rows.append(
            {
                "N_wave": n_wave,
                "seed": seed,
                "selection": "nearest passing trial to within-wave median signed displacement",
                "trial_forward_displacement_m": float(chosen["forward_displacement_m"]),
                "duration_s": duration_s,
                "fps": fps,
                "frame_count": frame_count,
                "demo_max_leg_yaw_target_error_deg": math.degrees(max_yaw_error_rad),
                "demo_rms_leg_pitch_target_error_deg": math.degrees(math.sqrt(pitch_squared_error / steps)),
                "demo_rms_body_yaw_target_error_deg": math.degrees(math.sqrt(body_squared_error / steps)),
                "video": str(video_path),
                "preview": str(preview_path),
            }
        )
        print(f"rendered demo N_wave={n_wave:g}, seed={seed}", flush=True)
    _write_csv(output_dir / "demo_manifest.csv", rows)
    (output_dir / "demo_manifest.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    return rows


def _run_trials(args: argparse.Namespace, waves: list[float]) -> list[dict]:
    import mujoco
    import mujoco.rollout

    output_dir = args.output_dir.resolve()
    checkpoint_dir = output_dir / "checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    all_trials: list[dict] = []
    print("loading physical model for rollout", flush=True)
    assets = _load_assets(args.repo_root.resolve())
    print("physical model loaded; creating rollout pool", flush=True)
    rollout_data = [mujoco.MjData(assets.model) for _ in range(args.workers)]
    with mujoco.rollout.Rollout(nthread=args.workers) as rollout_engine:
        print(f"rollout pool ready ({args.workers} threads)", flush=True)
        for n_wave in waves:
            checkpoint = checkpoint_dir / f"Nwave{_wave_tag(n_wave)}.csv"
            cached: list[dict] = []
            if checkpoint.is_file() and not args.force:
                cached = _read_csv(checkpoint)
                cached_seeds = {int(row["seed"]) for row in cached}
                expected = set(range(args.replays))
                if cached_seeds == expected:
                    print(f"loaded checkpoint N_wave={n_wave:g} ({len(cached)} trials)", flush=True)
                    all_trials.extend(cached)
                    continue
                cached = [row for row in cached if int(row["seed"]) in expected]
                print(
                    f"resuming N_wave={n_wave:g} from {len(cached)}/{args.replays}",
                    flush=True,
                )
            cached_seed_set = {int(row["seed"]) for row in cached}
            seeds = [seed for seed in range(args.replays) if seed not in cached_seed_set]
            chunk_size = max(1, int(args.chunk_size))
            rows: list[dict] = list(cached)
            for start in range(0, len(seeds), chunk_size):
                batch_seeds = seeds[start : start + chunk_size]
                rows.extend(
                    _simulate_rollout_batch(
                        assets,
                        rollout_engine,
                        rollout_data,
                        seeds=batch_seeds,
                        n_wave=n_wave,
                        warmup_cycles=args.warmup_cycles,
                        evaluation_cycles=args.evaluation_cycles,
                        sample_hz=args.sample_hz,
                    )
                )
                rows.sort(key=lambda row: int(row["seed"]))
                _write_csv(checkpoint, rows)
                print(f"N_wave={n_wave:g}: {len(rows)}/{args.replays}", flush=True)
            rows.sort(key=lambda row: int(row["seed"]))
            _write_csv(checkpoint, rows)
            all_trials.extend(rows)
    all_trials.sort(key=lambda row: (float(row["N_wave"]), int(row["seed"])))
    return all_trials


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = _model_path(repo_root)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    waves = sorted(set(float(value) for value in args.n_wave))
    if any(value <= 0.0 for value in waves):
        raise ValueError("N_wave must be positive")
    configuration = {
        "experiment": "physical robot: body yaw + IID leg pitch; leg yaw locked",
        "model_path": str(model_path),
        "model_variant": MODEL_VARIANT,
        "n_segments": N_SEGMENTS,
        "N_wave": waves,
        "spatial_phase_increment": "-2*pi*N_wave/(N_segments-1)",
        "body_yaw_half_amplitude_deg": YAW_HALF_AMPLITUDE_DEG,
        "body_omega_rad_s": OMEGA_BODY,
        "leg_yaw_locked": True,
        "pitch_process": "IID Bernoulli(0.5) per physical leg per half-gait-cycle window",
        "random_independence": "wave number, trial, window, segment, side",
        "trials_per_N_wave": args.replays,
        "warmup_cycles": args.warmup_cycles,
        "evaluation_cycles": args.evaluation_cycles,
        "evaluation_time_s": args.evaluation_cycles * 2.0 * math.pi / OMEGA_BODY,
        "decision_window_s": math.pi / OMEGA_BODY,
        "sample_hz": args.sample_hz,
        "initialization": "straight all-down, body-wave-off settle; cosine bridge; IID warmup",
    }
    (output_dir / "run_configuration.json").write_text(
        json.dumps(configuration, indent=2), encoding="utf-8"
    )
    print("running RNG semantic validation", flush=True)
    semantic = _semantic_validation(waves, args.replays, args.evaluation_cycles)
    print("running actuator-level control validation", flush=True)
    control_contract = _control_contract_validation(repo_root, waves)
    semantic["control_contract"] = control_contract
    semantic["passed"] = bool(semantic["passed"] and control_contract["passed"])
    (output_dir / "semantic_validation.json").write_text(
        json.dumps(semantic, indent=2), encoding="utf-8"
    )
    if not semantic["passed"]:
        raise RuntimeError("semantic validation failed")

    print("starting dynamics trials", flush=True)
    trials = _run_trials(args, waves)
    summary = _aggregate(trials, waves)
    _write_csv(output_dir / "iid_trials.csv", trials)
    _write_csv(output_dir / "iid_summary.csv", summary)
    _plot_results(output_dir, trials, summary)
    demo_rows = []
    if not args.skip_demos:
        demo_rows = _render_demos(
            repo_root,
            output_dir,
            trials,
            waves,
            warmup_cycles=args.warmup_cycles,
            duration_s=args.demo_duration_s,
            fps=args.demo_fps,
        )
    expected_trials = args.replays * len(waves)
    seed_audit = {
        str(n_wave): {
            "count": sum(math.isclose(float(row["N_wave"]), n_wave) for row in trials),
            "unique_seed_count": len({int(row["seed"]) for row in trials if math.isclose(float(row["N_wave"]), n_wave)}),
        }
        for n_wave in waves
    }
    validation = {
        "passed": bool(
            len(trials) == expected_trials
            and all(row["status"] == "PASS" for row in trials)
            and semantic["passed"]
            and all(item["count"] == args.replays and item["unique_seed_count"] == args.replays for item in seed_audit.values())
            and (args.skip_demos or len(demo_rows) == len(waves))
        ),
        "trial_count": len(trials),
        "expected_trial_count": expected_trials,
        "all_trials_pass": all(row["status"] == "PASS" for row in trials),
        "maximum_mujoco_warning_count": max(int(row["mujoco_warning_total"]) for row in trials),
        "seed_audit": seed_audit,
        "strict_independent_rng_across_wave": semantic["cross_wave_streams_differ"],
        "demo_count": len(demo_rows),
        "expected_demo_count": 0 if args.skip_demos else len(waves),
    }
    (output_dir / "validation.json").write_text(json.dumps(validation, indent=2), encoding="utf-8")
    readme = f"""# Step 33: physical robot IID contact-command wave sweep

The center-rooted measured-STL robot has {N_SEGMENTS} segments. Leg-yaw joints
are locked at their neutral offsets. The body-yaw wave uses
`Delta_phi=-2*pi*N_wave/(N_segments-1)`. Every half gait cycle, each of the 20
leg-pitch joints independently draws down/stance-opportunity or up/swing with
probability 0.5. This randomizes commanded contact opportunity; measured ground
contact remains a dynamical output.

Each wave condition contains {args.replays} trials. Each trial has
{args.warmup_cycles} warm-up cycles and {args.evaluation_cycles} evaluated cycles
({configuration['evaluation_time_s']:.3f} s). Random streams are independent
across wave number, trial, time window, segment, and side.

Files:
- `iid_trials.csv`: one row per simulation.
- `iid_summary.csv`: absolute-unit ensemble statistics and bootstrap intervals.
- `figures/`: signed/absolute displacement and measured-contact plots.
- `demos/`: one median-displacement representative video per wave number.
- `semantic_validation.json` and `validation.json`: controller and run audits.
"""
    (output_dir / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps(validation, indent=2), flush=True)
    if not validation["passed"]:
        raise RuntimeError("Step 33 validation failed")


def build_parser() -> argparse.ArgumentParser:
    repo_root = _repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "exp_data" / "reliability" / OUTPUT_NAME,
    )
    parser.add_argument("--n-wave", type=float, nargs="+", default=list(DEFAULT_N_WAVE))
    parser.add_argument("--replays", type=int, default=128)
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--workers", type=int, default=max(1, min(6, (os.cpu_count() or 2) // 2)))
    parser.add_argument("--chunk-size", type=int, default=4)
    parser.add_argument("--demo-duration-s", type=float, default=6.0)
    parser.add_argument("--demo-fps", type=int, default=30)
    parser.add_argument("--skip-demos", action="store_true")
    parser.add_argument("--force", action="store_true")
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
