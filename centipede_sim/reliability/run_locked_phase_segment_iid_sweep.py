"""Step 37: IID leg-pitch sweep over locked spatial phase and robot length.

``N_wave_ref10`` is a label defined on the ten-segment reference robot.  For
each label ``nu`` the adjacent body-joint phase increment is

    delta_phi = -2*pi*nu/(10-1).

That increment is held fixed while the physical robot length changes.  Thus an
N-segment robot contains ``nu*(N-1)/9`` effective full body waves.  Leg-yaw is
fixed and every physical leg independently draws pitch-down or pitch-up once
per half gait cycle.  The Bernoulli draw is a contact opportunity command;
actual ground contact remains a measured dynamical outcome.

The source XML files are never edited.  A stronger body-yaw position actuator
is applied to the in-memory MuJoCo model and recorded explicitly.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import run_physical_robot_pitch_body_yaw_iid_wave_sweep as step33  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    BRIDGE_TIME_S,
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    LEG_UP_DEG,
    OMEGA_BODY,
    SETTLE_TIME_S,
    _actual_contact_flags,
    _font,
    _leg_body_map,
    _root_roll_pitch_deg,
    _set_leg_colors,
)


OUTPUT_NAME = "step37_locked_phase_segment_iid_sweep"
MODEL_VARIANT = "center_rooted_physical_robot"
REFERENCE_SEGMENTS = 10
SEGMENTS = tuple(range(5, 11))
REFERENCE_WAVES = tuple(0.5 * value for value in range(2, 9))
BODY_YAW_HALF_AMPLITUDE_DEG = 19.0
PERIOD_SEED_OFFSET = 1_000_000
RNG_TAG = 37_010
PD_CANDIDATES = {
    "baseline": dict(kp_scale=1.0, kv_scale=1.0, force_scale=1.0),
    "moderate": dict(kp_scale=2.0, kv_scale=math.sqrt(2.0), force_scale=2.0),
    "strong": dict(kp_scale=3.0, kv_scale=math.sqrt(3.0), force_scale=3.0),
}
FULL_PD_NAME = "moderate"


def repo_root() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def write_csv(path: Path, rows: list[dict]) -> None:
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


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")


def wave_token(value: float) -> int:
    token = int(round(1000.0 * float(value)))
    if not math.isclose(token / 1000.0, value, abs_tol=1.0e-12):
        raise ValueError("reference wave label must be representable to 0.001")
    return token


def condition_tag(nu: float, n_segments: int) -> str:
    wave = f"{nu:03.1f}".replace(".", "p")
    return f"Nwave10_{wave}_Nseg{n_segments:02d}"


def locked_phase_increment(reference_wave: float) -> float:
    return -2.0 * math.pi * float(reference_wave) / (REFERENCE_SEGMENTS - 1)


def effective_wave_count(reference_wave: float, n_segments: int) -> float:
    return float(reference_wave) * (int(n_segments) - 1) / (REFERENCE_SEGMENTS - 1)


def model_path(root: Path, n_segments: int) -> Path:
    return (
        root
        / "exp_data"
        / "reliability"
        / "step26_center_rooted_physical_robot"
        / "models"
        / MODEL_VARIANT
        / f"centipede_{n_segments}seg_{MODEL_VARIANT}.xml"
    )


@dataclass
class Assets:
    model: object
    indices: tuple
    root_body_id: int
    stability_body_id: int
    body_map: dict
    dt: float
    n_segments: int
    model_file: Path
    model_sha256: str
    pd_manifest: dict


def apply_body_yaw_pd(model, link_y: np.ndarray, pd_name: str) -> dict:
    if pd_name not in PD_CANDIDATES:
        raise KeyError(pd_name)
    cfg = PD_CANDIDATES[pd_name]
    original_kp = np.asarray(model.actuator_gainprm[link_y, 0], dtype=float).copy()
    original_kv = -np.asarray(model.actuator_biasprm[link_y, 2], dtype=float).copy()
    original_force = np.asarray(model.actuator_forcerange[link_y, 1], dtype=float).copy()
    model.actuator_gainprm[link_y, 0] = original_kp * cfg["kp_scale"]
    model.actuator_biasprm[link_y, 1] = -original_kp * cfg["kp_scale"]
    model.actuator_biasprm[link_y, 2] = -original_kv * cfg["kv_scale"]
    model.actuator_forcerange[link_y, 0] = -original_force * cfg["force_scale"]
    model.actuator_forcerange[link_y, 1] = original_force * cfg["force_scale"]
    return {
        "name": pd_name,
        **cfg,
        "original_kp": float(original_kp[0]),
        "original_kv": float(original_kv[0]),
        "original_force_limit_Nm": float(original_force[0]),
        "applied_kp": float(model.actuator_gainprm[link_y[0], 0]),
        "applied_kv": float(-model.actuator_biasprm[link_y[0], 2]),
        "applied_force_limit_Nm": float(model.actuator_forcerange[link_y[0], 1]),
        "uniform_across_body_yaw_actuators": bool(
            np.allclose(model.actuator_gainprm[link_y, 0], model.actuator_gainprm[link_y[0], 0])
            and np.allclose(model.actuator_biasprm[link_y, 2], model.actuator_biasprm[link_y[0], 2])
            and np.allclose(model.actuator_forcerange[link_y, 1], model.actuator_forcerange[link_y[0], 1])
        ),
    }


def load_assets(root: Path, n_segments: int, pd_name: str) -> Assets:
    import mujoco

    path = model_path(root, n_segments)
    if not path.is_file():
        raise FileNotFoundError(path)
    model = mujoco.MjModel.from_xml_path(str(path))
    indices = step33._build_actuator_indices(n_segments)
    expected = 4 * n_segments + 2 * (n_segments - 1)
    if model.nu != expected:
        raise RuntimeError(f"N={n_segments}: expected {expected} actuators, found {model.nu}")
    link_y = indices[4]
    pd_manifest = apply_body_yaw_pd(model, link_y, pd_name)
    root_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root"))
    stability_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Chasis1"))
    if root_id < 0 or stability_id < 0:
        raise KeyError("robot_root or Chasis1 missing")
    return Assets(
        model=model,
        indices=indices,
        root_body_id=root_id,
        stability_body_id=stability_id,
        body_map=_leg_body_map(model, n_segments, mujoco),
        dt=float(model.opt.timestep),
        n_segments=n_segments,
        model_file=path,
        model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        pd_manifest=pd_manifest,
    )


class LockedPhaseIIDController:
    def __init__(self, model, indices, *, seed: int, reference_wave: float, n_segments: int):
        self.n_ctrl = int(model.nu)
        self.indices = indices
        self.seed = int(seed)
        self.reference_wave = float(reference_wave)
        self.n_segments = int(n_segments)
        self.wave_token = wave_token(reference_wave)
        self.phi_body = locked_phase_increment(reference_wave)
        self.gait_period_s = 2.0 * math.pi / OMEGA_BODY
        self.decision_window_s = 0.5 * self.gait_period_s
        self._state_cache: dict[int, np.ndarray] = {}

    def period_index(self, time_s: float) -> int:
        return int(math.floor((float(time_s) + 1.0e-12) / self.decision_window_s))

    def draw_state(self, period_index: int) -> np.ndarray:
        encoded_period = int(period_index + PERIOD_SEED_OFFSET)
        if encoded_period < 0:
            raise ValueError("warmup exceeds period seed offset")
        state = np.zeros((self.n_segments, 2), dtype=bool)
        for segment_index in range(self.n_segments):
            for side_index in range(2):
                rng = np.random.default_rng(
                    np.random.SeedSequence(
                        [
                            self.seed,
                            self.wave_token,
                            self.n_segments,
                            encoded_period,
                            segment_index,
                            side_index,
                            RNG_TAG,
                        ]
                    )
                )
                state[segment_index, side_index] = bool(rng.random() < 0.5)
        return state

    def stance_flags(self, time_s: float) -> np.ndarray:
        index = self.period_index(time_s)
        if index not in self._state_cache:
            self._state_cache[index] = self.draw_state(index)
        return self._state_cache[index].copy()

    def control(self, body_time_s: float, decision_time_s: float) -> np.ndarray:
        ctrl = np.zeros(self.n_ctrl, dtype=float)
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = self.indices
        neutral = math.radians(LEG_OFFSET_DEG)
        ctrl[shoulder_r] = neutral
        ctrl[shoulder_l] = -neutral
        ctrl[link_y] = math.radians(BODY_YAW_HALF_AMPLITUDE_DEG) * np.sin(
            OMEGA_BODY * float(body_time_s)
            + np.arange(self.n_segments - 1, dtype=float) * self.phi_body
        )
        ctrl[link_p] = 0.0
        state = self.stance_flags(decision_time_s)
        down = math.radians(LEG_DOWN_DEG)
        up = math.radians(LEG_UP_DEG)
        ctrl[leg_r] = np.where(state[:, 0], down, up)
        ctrl[leg_l] = np.where(state[:, 1], down, up)
        return ctrl


def initial_target(model, indices) -> np.ndarray:
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = indices
    ctrl = np.zeros(model.nu, dtype=float)
    ctrl[shoulder_r] = math.radians(LEG_OFFSET_DEG)
    ctrl[shoulder_l] = -math.radians(LEG_OFFSET_DEG)
    ctrl[leg_r] = math.radians(LEG_DOWN_DEG)
    ctrl[leg_l] = math.radians(LEG_DOWN_DEG)
    ctrl[link_y] = 0.0
    ctrl[link_p] = 0.0
    return ctrl


def build_controls(
    assets: Assets,
    *,
    seeds: list[int],
    reference_wave: float,
    warmup_cycles: int,
    evaluation_cycles: int,
) -> tuple[np.ndarray, int, float]:
    model = assets.model
    batch = len(seeds)
    period = 2.0 * math.pi / OMEGA_BODY
    settle_steps = int(math.ceil(SETTLE_TIME_S / assets.dt))
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / assets.dt))
    warmup_steps = int(math.ceil(warmup_cycles * period / assets.dt))
    evaluation_steps = int(math.ceil(evaluation_cycles * period / assets.dt))
    warmup_time = warmup_steps * assets.dt
    evaluation_start = settle_steps + bridge_steps + warmup_steps
    total_steps = evaluation_start + evaluation_steps
    controls = np.zeros((batch, total_steps, model.nu), dtype=np.float64)
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, _link_p = assets.indices
    neutral = math.radians(LEG_OFFSET_DEG)
    down = math.radians(LEG_DOWN_DEG)
    up = math.radians(LEG_UP_DEG)
    amplitude = math.radians(BODY_YAW_HALF_AMPLITUDE_DEG)
    phi = locked_phase_increment(reference_wave)
    batch_indices = np.arange(batch)

    settle_indices = np.arange(settle_steps)
    alpha = (settle_indices + 1.0) / settle_steps
    controls[np.ix_(batch_indices, settle_indices, shoulder_r)] = neutral * alpha[None, :, None]
    controls[np.ix_(batch_indices, settle_indices, shoulder_l)] = -neutral * alpha[None, :, None]
    controls[np.ix_(batch_indices, settle_indices, leg_r)] = down * alpha[None, :, None]
    controls[np.ix_(batch_indices, settle_indices, leg_l)] = down * alpha[None, :, None]

    bridge_indices = np.arange(settle_steps, settle_steps + bridge_steps)
    bridge_alpha = 0.5 - 0.5 * np.cos(math.pi * (np.arange(bridge_steps) + 1.0) / bridge_steps)
    bridge_body = amplitude * np.sin(np.arange(assets.n_segments - 1) * phi)
    controls[np.ix_(batch_indices, bridge_indices, shoulder_r)] = neutral
    controls[np.ix_(batch_indices, bridge_indices, shoulder_l)] = -neutral
    controls[np.ix_(batch_indices, bridge_indices, link_y)] = bridge_alpha[None, :, None] * bridge_body[None, None, :]

    warm_times = np.arange(warmup_steps) * assets.dt
    eval_times = warmup_time + np.arange(evaluation_steps) * assets.dt
    body_times = np.concatenate((warm_times, eval_times))
    dynamic_indices = np.arange(settle_steps + bridge_steps, total_steps)
    controls[np.ix_(batch_indices, dynamic_indices, shoulder_r)] = neutral
    controls[np.ix_(batch_indices, dynamic_indices, shoulder_l)] = -neutral
    controls[np.ix_(batch_indices, dynamic_indices, link_y)] = amplitude * np.sin(
        OMEGA_BODY * body_times[None, :, None]
        + np.arange(assets.n_segments - 1)[None, None, :] * phi
    )

    decision_times = np.concatenate((warm_times - warmup_time, np.arange(evaluation_steps) * assets.dt))
    period_indices = np.floor((decision_times + 1.0e-12) / (0.5 * period)).astype(int)
    unique_periods = np.unique(period_indices)
    for batch_index, seed in enumerate(seeds):
        controller = LockedPhaseIIDController(
            model, assets.indices, seed=seed, reference_wave=reference_wave,
            n_segments=assets.n_segments,
        )
        bridge_state = controller.stance_flags(-warmup_time)
        target_r = np.where(bridge_state[:, 0], down, up)
        target_l = np.where(bridge_state[:, 1], down, up)
        controls[batch_index][np.ix_(bridge_indices, leg_r)] = (
            (1.0 - bridge_alpha[:, None]) * down + bridge_alpha[:, None] * target_r[None, :]
        )
        controls[batch_index][np.ix_(bridge_indices, leg_l)] = (
            (1.0 - bridge_alpha[:, None]) * down + bridge_alpha[:, None] * target_l[None, :]
        )
        for index in unique_periods:
            state = controller.draw_state(int(index))
            local = np.flatnonzero(period_indices == index) + settle_steps + bridge_steps
            controls[batch_index][np.ix_(local, leg_r)] = np.where(state[:, 0], down, up)[None, :]
            controls[batch_index][np.ix_(local, leg_l)] = np.where(state[:, 1], down, up)[None, :]
    return np.ascontiguousarray(controls), evaluation_start, warmup_time


def simulate_batch(
    assets: Assets,
    rollout_engine,
    rollout_data: list,
    *,
    seeds: list[int],
    reference_wave: float,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
) -> list[dict]:
    import mujoco

    controls, evaluation_start, warmup_time = build_controls(
        assets, seeds=seeds, reference_wave=reference_wave,
        warmup_cycles=warmup_cycles, evaluation_cycles=evaluation_cycles,
    )
    state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    nstate = mujoco.mj_stateSize(assets.model, state_spec)
    initial_data = mujoco.MjData(assets.model)
    initial = np.empty(nstate, dtype=float)
    mujoco.mj_getState(assets.model, initial_data, initial, state_spec)
    initial_states = np.repeat(initial[None, :], len(seeds), axis=0)
    states, _ = rollout_engine.rollout(
        assets.model, rollout_data, initial_states, controls,
        control_spec=mujoco.mjtState.mjSTATE_CTRL.value, chunk_size=1,
    )
    warning_counts = {
        str(index): max(int(data.warning[index].number) for data in rollout_data)
        for index in range(len(rollout_data[0].warning))
        if max(int(data.warning[index].number) for data in rollout_data) > 0
    }
    warning_total = int(sum(warning_counts.values()))
    evaluation_steps = states.shape[1] - evaluation_start
    sample_stride = max(1, int(round(1.0 / (sample_hz * assets.dt))))
    sample_indices = np.arange(evaluation_start, states.shape[1], sample_stride, dtype=int)
    if sample_indices[-1] != states.shape[1] - 1:
        sample_indices = np.append(sample_indices, states.shape[1] - 1)
    metric_data = mujoco.MjData(assets.model)
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, _link_p = assets.indices
    body_qpos = assets.model.jnt_qposadr[assets.model.actuator_trnid[link_y, 0]]
    rows = []
    for batch_index, seed in enumerate(seeds):
        controller = LockedPhaseIIDController(
            assets.model, assets.indices, seed=seed, reference_wave=reference_wave,
            n_segments=assets.n_segments,
        )

        def load(snapshot: np.ndarray) -> None:
            mujoco.mj_setState(assets.model, metric_data, snapshot, state_spec)
            mujoco.mj_forward(assets.model, metric_data)

        load(states[batch_index, evaluation_start - 1])
        start_com = np.asarray(metric_data.subtree_com[assets.root_body_id]).copy()
        start_z = float(metric_data.xpos[assets.stability_body_id, 2])
        min_z = start_z
        min_up_z = float(metric_data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2])
        max_roll = 0.0
        max_pitch = 0.0
        body_error_sq = 0.0
        body_error_count = 0
        contacts = []
        planned = []
        for state_index in sample_indices:
            load(states[batch_index, state_index])
            eval_step = state_index - evaluation_start
            eval_time = eval_step * assets.dt
            planned.append(controller.stance_flags(eval_time))
            contacts.append(_actual_contact_flags(assets.model, metric_data, assets.body_map))
            error = metric_data.qpos[body_qpos] - controls[batch_index, state_index, link_y]
            body_error_sq += float(np.sum(np.square(error)))
            body_error_count += int(error.size)
            min_z = min(min_z, float(metric_data.xpos[assets.stability_body_id, 2]))
            min_up_z = min(min_up_z, float(metric_data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2]))
            pose = np.zeros(7)
            pose[3:7] = metric_data.xquat[assets.stability_body_id]
            roll, pitch = _root_roll_pitch_deg(pose)
            max_roll = max(max_roll, abs(roll))
            max_pitch = max(max_pitch, abs(pitch))
        final_com = np.asarray(metric_data.subtree_com[assets.root_body_id]).copy()
        displacement = final_com - start_com
        planned_array = np.asarray(planned, dtype=bool)
        contact_array = np.asarray(contacts, dtype=bool)
        down_count = int(np.sum(planned_array))
        up_count = int(planned_array.size - down_count)
        flat = planned_array.reshape(planned_array.shape[0], -1).astype(float)
        mean_corr, max_corr = step33.base._binary_pairwise_correlation(flat)
        gates = {
            "all_states_finite": bool(np.all(np.isfinite(states[batch_index]))),
            "no_mujoco_warnings": warning_total == 0,
            "never_inverted_at_sampled_states": min_up_z > 0.0,
            "open_loop_rollout_complete": states.shape[1] == controls.shape[1],
        }
        rows.append({
            "schema_version": 1,
            "mujoco_version": mujoco.__version__,
            "status": "PASS" if all(gates.values()) else "FAIL",
            "model_variant": MODEL_VARIANT,
            "n_segments": assets.n_segments,
            "n_legs": 2 * assets.n_segments,
            "N_wave_ref10": float(reference_wave),
            "locked_spatial_phase_offset_rad": locked_phase_increment(reference_wave),
            "locked_spatial_phase_offset_deg": math.degrees(locked_phase_increment(reference_wave)),
            "effective_wave_count": effective_wave_count(reference_wave, assets.n_segments),
            "body_yaw_half_amplitude_deg": BODY_YAW_HALF_AMPLITUDE_DEG,
            "body_yaw_pd_name": assets.pd_manifest["name"],
            "body_yaw_kp": assets.pd_manifest["applied_kp"],
            "body_yaw_kv": assets.pd_manifest["applied_kv"],
            "body_yaw_force_limit_Nm": assets.pd_manifest["applied_force_limit_Nm"],
            "seed": int(seed),
            "rng_condition_mode": "strict independent across phase, N, trial, window, segment, side",
            "warmup_cycles": int(warmup_cycles),
            "evaluation_cycles": int(evaluation_cycles),
            "evaluation_time_s": evaluation_steps * assets.dt,
            "decision_window_s": math.pi / OMEGA_BODY,
            "forward_displacement_m": float(displacement[1]),
            "absolute_forward_displacement_m": float(abs(displacement[1])),
            "lateral_displacement_m": float(displacement[0]),
            "vertical_displacement_m": float(displacement[2]),
            "commanded_down_fraction": float(np.mean(planned_array)),
            "actual_contact_fraction": float(np.mean(contact_array)),
            "contact_given_commanded_down": float(np.sum(contact_array & planned_array) / down_count) if down_count else float("nan"),
            "contact_given_commanded_up": float(np.sum(contact_array & ~planned_array) / up_count) if up_count else float("nan"),
            "mean_abs_pairwise_state_correlation": float(mean_corr),
            "max_abs_pairwise_state_correlation": float(max_corr),
            "rms_body_yaw_tracking_error_deg": math.degrees(math.sqrt(body_error_sq / body_error_count)),
            "start_root_z_m": start_z,
            "min_root_z_m": min_z,
            "minimum_root_up_z": min_up_z,
            "max_abs_roll_deg": max_roll,
            "max_abs_pitch_deg": max_pitch,
            "mujoco_warning_total": warning_total,
            "mujoco_warning_counts": json.dumps(warning_counts, sort_keys=True),
            "gates": json.dumps(gates, sort_keys=True),
        })
    return rows


def bootstrap_mean(values: np.ndarray, rng: np.random.Generator, count: int = 3000) -> tuple[float, float]:
    draws = rng.choice(values, size=(count, values.size), replace=True)
    means = np.mean(draws, axis=1)
    return float(np.quantile(means, 0.025)), float(np.quantile(means, 0.975))


def summarize(trials: list[dict], waves: list[float], segments: list[int]) -> list[dict]:
    rows = []
    rng = np.random.default_rng(37_100)
    for wave in waves:
        for n_segments in segments:
            selected_all = [
                row for row in trials
                if math.isclose(float(row["N_wave_ref10"]), wave)
                and int(row["n_segments"]) == n_segments
            ]
            selected = [row for row in selected_all if row["status"] == "PASS"]
            if not selected:
                continue
            values = np.asarray([float(row["forward_displacement_m"]) for row in selected])
            low, high = bootstrap_mean(values, rng)
            rows.append({
                "N_wave_ref10": wave,
                "locked_spatial_phase_offset_rad": locked_phase_increment(wave),
                "locked_spatial_phase_offset_deg": math.degrees(locked_phase_increment(wave)),
                "n_segments": n_segments,
                "effective_wave_count": effective_wave_count(wave, n_segments),
                "n_trials": len(selected_all),
                "n_pass": len(selected),
                "mean_forward_displacement_m": float(np.mean(values)),
                "bootstrap_mean_ci95_low_m": low,
                "bootstrap_mean_ci95_high_m": high,
                "median_forward_displacement_m": float(np.median(values)),
                "sd_forward_displacement_m": float(np.std(values, ddof=1)),
                "iqr_forward_displacement_m": float(np.quantile(values, 0.75) - np.quantile(values, 0.25)),
                "p05_forward_displacement_m": float(np.quantile(values, 0.05)),
                "p95_forward_displacement_m": float(np.quantile(values, 0.95)),
                "p05_p95_width_m": float(np.quantile(values, 0.95) - np.quantile(values, 0.05)),
                "positive_displacement_fraction": float(np.mean(values > 0.0)),
                "mean_actual_contact_fraction": float(np.mean([float(row["actual_contact_fraction"]) for row in selected])),
                "mean_body_yaw_tracking_error_deg": float(np.mean([float(row["rms_body_yaw_tracking_error_deg"]) for row in selected])),
            })
    return rows


def dispersion_trends(
    trials: list[dict], waves: list[float], segments: list[int], bootstrap_count: int = 3000,
) -> list[dict]:
    rows = []
    rng = np.random.default_rng(37_200)
    x = np.asarray(segments, dtype=float)
    for wave in waves:
        groups = []
        for n_segments in segments:
            groups.append(np.asarray([
                float(row["forward_displacement_m"]) for row in trials
                if row["status"] == "PASS"
                and math.isclose(float(row["N_wave_ref10"]), wave)
                and int(row["n_segments"]) == n_segments
            ]))
        iqr = np.asarray([np.quantile(v, 0.75) - np.quantile(v, 0.25) for v in groups])
        sd = np.asarray([np.std(v, ddof=1) for v in groups])
        slope_iqr = float(np.polyfit(x, iqr, 1)[0])
        slope_sd = float(np.polyfit(x, sd, 1)[0])
        boot_iqr = np.empty(bootstrap_count)
        boot_sd = np.empty(bootstrap_count)
        for index in range(bootstrap_count):
            sampled = [rng.choice(v, size=v.size, replace=True) for v in groups]
            spread_iqr = [np.quantile(v, 0.75) - np.quantile(v, 0.25) for v in sampled]
            spread_sd = [np.std(v, ddof=1) for v in sampled]
            boot_iqr[index] = np.polyfit(x, spread_iqr, 1)[0]
            boot_sd[index] = np.polyfit(x, spread_sd, 1)[0]
        iqr_low, iqr_high = np.quantile(boot_iqr, [0.025, 0.975])
        sd_low, sd_high = np.quantile(boot_sd, [0.025, 0.975])
        if iqr_low > 0:
            direction = "concentrated_to_dispersed"
        elif iqr_high < 0:
            direction = "dispersed_to_concentrated"
        else:
            direction = "no_resolved_monotone_IQR_trend"
        rows.append({
            "N_wave_ref10": wave,
            "locked_spatial_phase_offset_rad": locked_phase_increment(wave),
            "locked_spatial_phase_offset_deg": math.degrees(locked_phase_increment(wave)),
            "iqr_slope_m_per_segment": slope_iqr,
            "iqr_slope_bootstrap_ci95_low": float(iqr_low),
            "iqr_slope_bootstrap_ci95_high": float(iqr_high),
            "sd_slope_m_per_segment": slope_sd,
            "sd_slope_bootstrap_ci95_low": float(sd_low),
            "sd_slope_bootstrap_ci95_high": float(sd_high),
            "predeclared_IQR_trend_class": direction,
        })
    return rows


def plot_results(output: Path, trials: list[dict], summary: list[dict], trends: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    waves = sorted({float(row["N_wave_ref10"]) for row in summary})
    segments = sorted({int(row["n_segments"]) for row in summary})
    colors = plt.cm.viridis(np.linspace(0.08, 0.92, len(waves)))

    fig, ax = plt.subplots(figsize=(7.3, 4.8))
    for color, wave in zip(colors, waves):
        selected = sorted((row for row in summary if math.isclose(float(row["N_wave_ref10"]), wave)), key=lambda r: int(r["n_segments"]))
        x = np.asarray([int(row["n_segments"]) for row in selected])
        mean = np.asarray([float(row["mean_forward_displacement_m"]) for row in selected])
        low = np.asarray([float(row["bootstrap_mean_ci95_low_m"]) for row in selected])
        high = np.asarray([float(row["bootstrap_mean_ci95_high_m"]) for row in selected])
        ax.errorbar(x, mean, yerr=np.vstack((mean-low, high-mean)), marker="o", capsize=2.5, color=color, label=rf"$N_{{wave}}^{{(10)}}={wave:g}$")
    ax.axhline(0, color="#777777", linewidth=0.8)
    ax.set(xlabel="Segment number, N", ylabel="Mean signed displacement (m)", xticks=segments)
    ax.grid(alpha=0.2)
    ax.legend(ncol=2, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_mean_displacement_vs_segment_all_phase_offsets.png", dpi=260)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.3, 4.8))
    for color, wave in zip(colors, waves):
        selected = sorted((row for row in summary if math.isclose(float(row["N_wave_ref10"]), wave)), key=lambda r: int(r["n_segments"]))
        ax.plot([int(row["n_segments"]) for row in selected], [float(row["iqr_forward_displacement_m"]) for row in selected], "o-", color=color, label=rf"$N_{{wave}}^{{(10)}}={wave:g}$")
    ax.set(xlabel="Segment number, N", ylabel="Displacement IQR (m)", xticks=segments)
    ax.grid(alpha=0.2)
    ax.legend(ncol=2, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_distribution_IQR_vs_segment_all_phase_offsets.png", dpi=260)
    plt.close(fig)

    sd_rows = []
    sd_rng = np.random.default_rng(37_301)
    for wave in waves:
        for n_segments in segments:
            values = np.asarray([
                float(row["forward_displacement_m"]) for row in trials
                if row["status"] == "PASS"
                and math.isclose(float(row["N_wave_ref10"]), wave)
                and int(row["n_segments"]) == n_segments
            ])
            boot = np.std(
                sd_rng.choice(values, size=(3000, values.size), replace=True),
                axis=1, ddof=1,
            )
            low_sd, high_sd = np.quantile(boot, [0.025, 0.975])
            sd_rows.append({
                "N_wave_ref10": wave,
                "locked_phase_offset_deg": math.degrees(locked_phase_increment(wave)),
                "n_segments": n_segments,
                "n_trials": int(values.size),
                "sample_SD_m": float(np.std(values, ddof=1)),
                "bootstrap_SD_ci95_low_m": float(low_sd),
                "bootstrap_SD_ci95_high_m": float(high_sd),
            })
    write_csv(output / "sd_by_phase_and_segment.csv", sd_rows)
    sd_ylim = max(row["bootstrap_SD_ci95_high_m"] for row in sd_rows) * 1.08
    fig, ax = plt.subplots(figsize=(7.3, 4.8))
    for color, wave in zip(colors, waves):
        selected = [row for row in sd_rows if math.isclose(row["N_wave_ref10"], wave)]
        ax.plot(
            [row["n_segments"] for row in selected],
            [row["sample_SD_m"] for row in selected], "o-", color=color,
            label=rf"$\Delta\phi={math.degrees(locked_phase_increment(wave)):.0f}^\circ$",
        )
    ax.set(xlabel="Segment number, N", ylabel="Displacement SD (m)", xticks=segments, ylim=(0, sd_ylim))
    ax.grid(alpha=0.2)
    ax.legend(ncol=2, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_distribution_SD_vs_segment_all_phase_offsets.png", dpi=260)
    plt.close(fig)
    for color, wave in zip(colors, waves):
        selected = [row for row in sd_rows if math.isclose(row["N_wave_ref10"], wave)]
        x = [row["n_segments"] for row in selected]
        y = [row["sample_SD_m"] for row in selected]
        low_sd = [row["bootstrap_SD_ci95_low_m"] for row in selected]
        high_sd = [row["bootstrap_SD_ci95_high_m"] for row in selected]
        phase_deg = math.degrees(locked_phase_increment(wave))
        fig, ax = plt.subplots(figsize=(5.6, 4.2))
        ax.fill_between(x, low_sd, high_sd, color=color, alpha=0.16)
        ax.plot(x, y, "o-", color=color)
        ax.set(
            xlabel="Segment number, N", ylabel="Displacement SD (m)",
            xticks=segments, ylim=(0, sd_ylim),
            title=rf"$\Delta\phi={phase_deg:.0f}^\circ$; $N_{{wave}}^{{(10)}}={wave:g}$",
        )
        ax.grid(alpha=0.2)
        fig.tight_layout()
        tag = f"{wave:03.1f}".replace(".", "p")
        fig.savefig(figure_dir / f"iid_SD_vs_segment_Nwave10_{tag}.png", dpi=260)
        plt.close(fig)
    sd_trend_rows = []
    for row in trends:
        low_sd = float(row["sd_slope_bootstrap_ci95_low"])
        high_sd = float(row["sd_slope_bootstrap_ci95_high"])
        sd_trend_rows.append({
            "N_wave_ref10": float(row["N_wave_ref10"]),
            "locked_phase_offset_deg": float(row["locked_spatial_phase_offset_deg"]),
            "SD_slope_m_per_segment": float(row["sd_slope_m_per_segment"]),
            "bootstrap_slope_ci95_low": low_sd,
            "bootstrap_slope_ci95_high": high_sd,
            "SD_trend_class": (
                "concentrated_to_dispersed" if low_sd > 0 else
                "dispersed_to_concentrated" if high_sd < 0 else "unresolved"
            ),
        })
    write_csv(output / "sd_dispersion_trends.csv", sd_trend_rows)
    sd_slopes = np.asarray([row["SD_slope_m_per_segment"] for row in sd_trend_rows])
    sd_low = np.asarray([row["bootstrap_slope_ci95_low"] for row in sd_trend_rows])
    sd_high = np.asarray([row["bootstrap_slope_ci95_high"] for row in sd_trend_rows])
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.errorbar(np.arange(len(sd_trend_rows)), sd_slopes,
                yerr=np.vstack((sd_slopes-sd_low, sd_high-sd_slopes)),
                fmt="o", capsize=4, color="#177e89")
    ax.axhline(0, color="#777777", linewidth=0.9)
    ax.set(
        xticks=np.arange(len(sd_trend_rows)),
        xticklabels=[f"{row['locked_phase_offset_deg']:.0f}" for row in sd_trend_rows],
        xlabel="Locked phase offset (deg)", ylabel="SD slope (m per added segment)",
    )
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_phase_offset_SD_trend.png", dpi=260)
    plt.close(fig)

    trend_x = np.arange(len(trends))
    slopes = np.asarray([float(row["iqr_slope_m_per_segment"]) for row in trends])
    low = np.asarray([float(row["iqr_slope_bootstrap_ci95_low"]) for row in trends])
    high = np.asarray([float(row["iqr_slope_bootstrap_ci95_high"]) for row in trends])
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    ax.errorbar(trend_x, slopes, yerr=np.vstack((slopes-low, high-slopes)), fmt="o", capsize=4, color="#177e89")
    ax.axhline(0, color="#777777", linewidth=0.9)
    ax.set(xticks=trend_x, xticklabels=[f"{float(row['N_wave_ref10']):g}" for row in trends], xlabel=r"Reference label, $N_{wave}^{(10)}$", ylabel="IQR slope (m per added segment)")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figure_dir / "iid_phase_offset_IQR_trend.png", dpi=260)
    plt.close(fig)

    passed = [row for row in trials if row["status"] == "PASS"]
    all_values = np.asarray([float(row["forward_displacement_m"]) for row in passed])
    bins = np.linspace(float(np.min(all_values)), float(np.max(all_values)), 34)
    for wave in waves:
        fig, axes = plt.subplots(2, 3, figsize=(10.2, 6.3), sharex=True, sharey=True)
        for axis, n_segments in zip(axes.flat, segments):
            values = np.asarray([
                float(row["forward_displacement_m"]) for row in passed
                if math.isclose(float(row["N_wave_ref10"]), wave)
                and int(row["n_segments"]) == n_segments
            ])
            axis.hist(values, bins=bins, density=True, color="#4f91a8", alpha=0.82)
            axis.axvline(float(np.mean(values)), color="#202020", linewidth=1.4)
            axis.axvline(0, color="#777777", linewidth=0.7)
            axis.set_title(f"N={n_segments}")
            axis.grid(alpha=0.12)
        fig.suptitle(rf"IID contact opportunity; locked $N_{{wave}}^{{(10)}}={wave:g}$ phase offset")
        fig.supxlabel("Signed forward displacement (m)")
        fig.supylabel("Empirical density")
        fig.tight_layout()
        tag = f"{wave:03.1f}".replace(".", "p")
        fig.savefig(figure_dir / f"iid_histograms_locked_phase_Nwave10_{tag}.png", dpi=260)
        plt.close(fig)


def run_condition(
    root: Path, output: Path, *, reference_wave: float, n_segments: int,
    replays: int, warmup_cycles: int, evaluation_cycles: int, sample_hz: float,
    workers: int, chunk_size: int, pd_name: str, force: bool,
) -> list[dict]:
    import mujoco
    import mujoco.rollout

    checkpoint = output / "checkpoints" / f"{condition_tag(reference_wave, n_segments)}.csv"
    cached = [] if force or not checkpoint.is_file() else read_csv(checkpoint)
    expected = set(range(replays))
    cached = [row for row in cached if int(row["seed"]) in expected]
    cached_seeds = {int(row["seed"]) for row in cached}
    if cached_seeds == expected:
        print(f"loaded {condition_tag(reference_wave, n_segments)} ({replays})", flush=True)
        return cached
    assets = load_assets(root, n_segments, pd_name)
    rollout_data = [mujoco.MjData(assets.model) for _ in range(workers)]
    rows = list(cached)
    pending = [seed for seed in range(replays) if seed not in cached_seeds]
    with mujoco.rollout.Rollout(nthread=workers) as engine:
        for start in range(0, len(pending), chunk_size):
            seeds = pending[start:start+chunk_size]
            rows.extend(simulate_batch(
                assets, engine, rollout_data, seeds=seeds, reference_wave=reference_wave,
                warmup_cycles=warmup_cycles, evaluation_cycles=evaluation_cycles,
                sample_hz=sample_hz,
            ))
            rows.sort(key=lambda row: int(row["seed"]))
            write_csv(checkpoint, rows)
            print(f"{condition_tag(reference_wave, n_segments)}: {len(rows)}/{replays}", flush=True)
    return rows


def semantic_validation(root: Path, waves: list[float], segments: list[int], pd_name: str) -> dict:
    streams = {}
    rows = []
    for wave in waves:
        for n_segments in segments:
            assets = load_assets(root, n_segments, pd_name)
            controller = LockedPhaseIIDController(
                assets.model, assets.indices, seed=0, reference_wave=wave,
                n_segments=n_segments,
            )
            flattened = np.concatenate([controller.draw_state(index).ravel() for index in range(12)])
            key = (wave, n_segments)
            streams[key] = tuple(bool(v) for v in flattened)
            rows.append({
                "N_wave_ref10": wave,
                "n_segments": n_segments,
                "locked_phase_rad": controller.phi_body,
                "effective_wave_count": effective_wave_count(wave, n_segments),
                "empirical_down_probability": float(np.mean(flattened)),
                "pd_manifest": assets.pd_manifest,
                "model_sha256": assets.model_sha256,
            })
    keys = list(streams)
    independent = all(streams[a] != streams[b] for index, a in enumerate(keys) for b in keys[index+1:])
    phase_correct = all(math.isclose(float(row["locked_phase_rad"]), locked_phase_increment(float(row["N_wave_ref10"])), abs_tol=1e-12) for row in rows)
    pd_uniform = all(bool(row["pd_manifest"]["uniform_across_body_yaw_actuators"]) for row in rows)
    return {
        "passed": bool(independent and phase_correct and pd_uniform),
        "strict_independent_condition_streams": independent,
        "locked_phase_definition_passed": phase_correct,
        "body_yaw_pd_uniform": pd_uniform,
        "rows": rows,
    }


def run_pd_pilot(args: argparse.Namespace) -> None:
    import mujoco
    import mujoco.rollout

    root = args.repo_root.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for pd_name in PD_CANDIDATES:
        for n_segments in (5, 10):
            assets = load_assets(root, n_segments, pd_name)
            rollout_data = [mujoco.MjData(assets.model) for _ in range(args.workers)]
            with mujoco.rollout.Rollout(nthread=args.workers) as engine:
                for wave in (1.0, 2.5, 4.0):
                    trial_rows = simulate_batch(
                        assets, engine, rollout_data, seeds=list(range(4)), reference_wave=wave,
                        warmup_cycles=2, evaluation_cycles=4, sample_hz=args.sample_hz,
                    )
                    for row in trial_rows:
                        rows.append(row)
    write_csv(output / "pd_pilot_trials.csv", rows)
    summary = []
    for name in PD_CANDIDATES:
        selected = [row for row in rows if row["body_yaw_pd_name"] == name]
        summary.append({
            "pd_name": name,
            **PD_CANDIDATES[name],
            "n_trials": len(selected),
            "pass_fraction": float(np.mean([row["status"] == "PASS" for row in selected])),
            "mean_rms_body_yaw_tracking_error_deg": float(np.mean([row["rms_body_yaw_tracking_error_deg"] for row in selected])),
            "maximum_roll_deg": float(np.max([row["max_abs_roll_deg"] for row in selected])),
            "maximum_pitch_deg": float(np.max([row["max_abs_pitch_deg"] for row in selected])),
        })
    baseline = next(row for row in summary if row["pd_name"] == "baseline")
    for row in summary:
        row["tracking_error_reduction_vs_baseline_percent"] = 100.0 * (baseline["mean_rms_body_yaw_tracking_error_deg"] - row["mean_rms_body_yaw_tracking_error_deg"]) / baseline["mean_rms_body_yaw_tracking_error_deg"]
    write_csv(output / "pd_pilot_summary.csv", summary)
    selected = next(row for row in summary if row["pd_name"] == FULL_PD_NAME)
    validation = {
        "passed": bool(selected["pass_fraction"] == 1.0 and selected["tracking_error_reduction_vs_baseline_percent"] > 0.0),
        "selection": FULL_PD_NAME,
        "rule": "preselected moderate 2x Kp/force, sqrt(2)x Kv; require all pilot trials pass and mean tracking error improve over baseline",
        "summary": summary,
    }
    write_json(output / "pd_pilot_validation.json", validation)
    print(json.dumps(validation, indent=2), flush=True)
    if not validation["passed"]:
        raise RuntimeError("PD pilot gate failed")


def run_full(args: argparse.Namespace) -> None:
    root = args.repo_root.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    waves = sorted(set(float(value) for value in args.reference_waves))
    segments = sorted(set(int(value) for value in args.segments))
    if min(segments) < 2 or any(value <= 0 for value in waves):
        raise ValueError("invalid waves or segments")
    pilot_path = output / "pd_pilot_validation.json"
    if not pilot_path.is_file() or not json.loads(pilot_path.read_text(encoding="utf-8"))["passed"]:
        raise RuntimeError("run --mode pilot before the full sweep")
    semantic = semantic_validation(root, waves, segments, FULL_PD_NAME)
    write_json(output / "semantic_validation.json", semantic)
    if not semantic["passed"]:
        raise RuntimeError("semantic validation failed")
    configuration = {
        "experiment": "locked reference-10 phase offset x physical segment count under IID leg pitch",
        "reference_segment_count": REFERENCE_SEGMENTS,
        "segments": segments,
        "N_wave_ref10": waves,
        "locked_phase_formula": "Delta_phi=-2*pi*N_wave_ref10/(10-1)",
        "effective_wave_formula": "N_wave_eff=N_wave_ref10*(N_segments-1)/9",
        "body_yaw_half_amplitude_deg": BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_omega_rad_s": OMEGA_BODY,
        "body_yaw_pd": FULL_PD_NAME,
        "body_yaw_pd_scales": PD_CANDIDATES[FULL_PD_NAME],
        "leg_yaw_locked": True,
        "pitch_process": "IID Bernoulli(0.5) per physical leg per half-gait-cycle window",
        "random_independence": "reference phase, N, trial, window, segment, side",
        "trials_per_condition": args.replays,
        "warmup_cycles": args.warmup_cycles,
        "evaluation_cycles": args.evaluation_cycles,
        "evaluation_time_s": args.evaluation_cycles * 2.0 * math.pi / OMEGA_BODY,
        "contact_audit_sample_hz": args.sample_hz,
        "primary_distribution_width": "IQR of signed forward displacement",
        "predeclared_trend": "linear IQR slope versus N with within-N bootstrap 95% CI",
    }
    write_json(output / "run_configuration.json", configuration)
    all_rows = []
    for wave in waves:
        for n_segments in segments:
            all_rows.extend(run_condition(
                root, output, reference_wave=wave, n_segments=n_segments,
                replays=args.replays, warmup_cycles=args.warmup_cycles,
                evaluation_cycles=args.evaluation_cycles, sample_hz=args.sample_hz,
                workers=args.workers, chunk_size=args.chunk_size,
                pd_name=FULL_PD_NAME, force=args.force,
            ))
    all_rows.sort(key=lambda row: (float(row["N_wave_ref10"]), int(row["n_segments"]), int(row["seed"])))
    write_csv(output / "iid_trials.csv", all_rows)
    summary = summarize(all_rows, waves, segments)
    trends = dispersion_trends(all_rows, waves, segments)
    write_csv(output / "iid_summary.csv", summary)
    write_csv(output / "dispersion_trends.csv", trends)
    endpoint_rows = []
    for wave in waves:
        start = next(row for row in summary if math.isclose(float(row["N_wave_ref10"]), wave) and int(row["n_segments"]) == min(segments))
        end = next(row for row in summary if math.isclose(float(row["N_wave_ref10"]), wave) and int(row["n_segments"]) == max(segments))
        endpoint_rows.append({
            "N_wave_ref10": wave,
            "locked_spatial_phase_offset_deg": math.degrees(locked_phase_increment(wave)),
            "start_n_segments": min(segments),
            "end_n_segments": max(segments),
            "start_effective_wave_count": start["effective_wave_count"],
            "end_effective_wave_count": end["effective_wave_count"],
            "mean_displacement_start_m": start["mean_forward_displacement_m"],
            "mean_displacement_end_m": end["mean_forward_displacement_m"],
            "mean_displacement_change_m": end["mean_forward_displacement_m"] - start["mean_forward_displacement_m"],
            "IQR_start_m": start["iqr_forward_displacement_m"],
            "IQR_end_m": end["iqr_forward_displacement_m"],
            "IQR_change_m": end["iqr_forward_displacement_m"] - start["iqr_forward_displacement_m"],
        })
    write_csv(output / "phase_endpoint_comparison.csv", endpoint_rows)
    plot_results(output, all_rows, summary, trends)
    expected = len(waves) * len(segments) * args.replays
    identities = {(float(row["N_wave_ref10"]), int(row["n_segments"]), int(row["seed"])) for row in all_rows}
    validation = {
        "passed": bool(len(all_rows) == expected and len(identities) == expected and all(row["status"] == "PASS" for row in all_rows)),
        "trial_count": len(all_rows),
        "expected_trial_count": expected,
        "unique_condition_trial_keys": len(identities),
        "all_trials_pass": all(row["status"] == "PASS" for row in all_rows),
        "maximum_warning_count": max(int(row["mujoco_warning_total"]) for row in all_rows),
        "semantic_validation_passed": semantic["passed"],
    }
    write_json(output / "validation.json", validation)
    largest_slope = max(trends, key=lambda row: float(row["iqr_slope_m_per_segment"]))
    most_negative_slope = min(trends, key=lambda row: float(row["iqr_slope_m_per_segment"]))
    findings = {
        "largest_observed_IQR_slope": largest_slope,
        "most_negative_observed_IQR_slope": most_negative_slope,
        "all_observed_IQR_slopes_are_nonpositive": all(float(row["iqr_slope_m_per_segment"]) <= 0.0 for row in trends),
        "resolved_positive_trends": [row for row in trends if row["predeclared_IQR_trend_class"] == "concentrated_to_dispersed"],
        "resolved_negative_trends": [row for row in trends if row["predeclared_IQR_trend_class"] == "dispersed_to_concentrated"],
        "interpretation_limit": "descriptive phase-specific physical-model evidence; not an information-theoretic scaling law by itself",
    }
    write_json(output / "findings.json", findings)
    lines = [
        "# Step 37: locked phase-offset × segment-count IID sweep", "",
        "`N_wave_ref10` labels a phase increment defined on the 10-segment reference robot:",
        "`Delta_phi=-2*pi*N_wave_ref10/9`. The same adjacent-joint phase increment is",
        "held fixed for N=5..10, so `N_wave_eff=N_wave_ref10*(N-1)/9`.", "",
        f"Each of the {len(waves)*len(segments)} conditions contains {args.replays} independent trials.",
        f"The shared body-yaw half-amplitude is {BODY_YAW_HALF_AMPLITUDE_DEG:g} deg. Leg yaw is fixed.",
        "Body-yaw PD uses the audited `moderate` runtime setting; source XML files are unchanged.", "",
        "IID means independent Bernoulli(0.5) pitch-down/up commands per physical leg and",
        "half-cycle. Actual contact is measured and is not directly switched off.", "",
        "Primary outputs:",
        "- `iid_trials.csv`: raw trial outcomes.",
        "- `iid_summary.csv`: center and spread per phase/segment condition.",
        "- `dispersion_trends.csv`: predeclared IQR/SD trend tests.",
        "- `phase_endpoint_comparison.csv`: direct N=5 versus N=10 comparison.",
        "- `figures/iid_histograms_locked_phase_*.png`: one histogram panel per locked phase.",
        "- `validation.json`: completion gates.",
    ]
    (output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"validation": validation, "findings": findings}, indent=2), flush=True)
    if not validation["passed"]:
        raise RuntimeError("full sweep validation failed")


def build_parser() -> argparse.ArgumentParser:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("pilot", "full"), required=True)
    parser.add_argument("--repo-root", type=Path, default=root)
    parser.add_argument("--output-dir", type=Path, default=root / "exp_data" / "reliability" / OUTPUT_NAME)
    parser.add_argument("--reference-waves", type=float, nargs="+", default=list(REFERENCE_WAVES))
    parser.add_argument("--segments", type=int, nargs="+", default=list(SEGMENTS))
    parser.add_argument("--replays", type=int, default=128)
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--workers", type=int, default=max(1, min(6, (os.cpu_count() or 2) // 2)))
    parser.add_argument("--chunk-size", type=int, default=4)
    parser.add_argument("--force", action="store_true")
    return parser


if __name__ == "__main__":
    parsed = build_parser().parse_args()
    if parsed.mode == "pilot":
        run_pd_pilot(parsed)
    else:
        run_full(parsed)
