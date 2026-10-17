"""Fixed-window IID leg-state experiment and locked-pitch locomotion ablations.

The canonical body wave is retained in every condition.  In the stochastic
condition, each physical leg independently draws a commanded down/up state at
the start of every fixed half-gait-period decision window.  The canonical
leg-yaw (Shoulder*_R/L) oscillator is not randomized.

The deterministic ablations lock all leg-pitch commands either fully up or
fully down and compare body-wave-only against body-wave-plus-leg-yaw control.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from run_independent_random_leg_phases import (  # noqa: E402
    BRIDGE_TIME_S,
    DUTY_FACTOR,
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    LEG_UP_DEG,
    OMEGA_BODY,
    PHI_BODY,
    SETTLE_TIME_S,
    YAW_HALF_AMPLITUDE_DEG,
    _actual_contact_flags,
    _binary_pairwise_correlation,
    _font,
    _install_project_imports,
    _leg_body_map,
    _model_collision_check,
    _root_roll_pitch_deg,
    _root_up_z,
    _set_leg_colors,
    _write_csv,
    _write_json,
)


DEFAULT_SEGMENTS = (4, 6, 8)
DEFAULT_REPLAYS = 64
DEFAULT_EVALUATION_CYCLES = 12
DEFAULT_WARMUP_CYCLES = 4
DEFAULT_SAMPLE_HZ = 100.0
RANDOM_MODE = "fixed_window_iid_leg_states"
REGULAR_MODE = "regular_coupled"
ABLATION_MODES = (
    "body_wave_pitch_locked_up",
    "body_wave_pitch_locked_down",
    "body_wave_leg_yaw_pitch_locked_up",
    "body_wave_leg_yaw_pitch_locked_down",
)
ALL_MODES = (RANDOM_MODE, REGULAR_MODE, *ABLATION_MODES)
PERIOD_SEED_OFFSET = 100_000


def _repo_root_from_script() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def _mode_label(mode: str) -> str:
    return {
        RANDOM_MODE: "Fixed-window IID leg states",
        REGULAR_MODE: "Regular coupled gait",
        "body_wave_pitch_locked_up": "Body wave; pitch locked up",
        "body_wave_pitch_locked_down": "Body wave; pitch locked down",
        "body_wave_leg_yaw_pitch_locked_up": (
            "Body wave + leg yaw; pitch locked up"
        ),
        "body_wave_leg_yaw_pitch_locked_down": (
            "Body wave + leg yaw; pitch locked down"
        ),
    }[mode]


class FixedWindowIIDLegStateController:
    """Canonical body/leg-yaw wave plus IID pitch-state draws per window."""

    def __init__(
        self,
        n_segments: int,
        n_ctrl: int,
        indices,
        canonical_compute_ctrl,
        *,
        seed: int,
        stance_probability: float = 0.5,
        rng_cross_n_mode: str = "legacy_nested",
    ) -> None:
        if n_segments <= 0:
            raise ValueError("n_segments must be positive")
        if not 0.0 <= stance_probability <= 1.0:
            raise ValueError("stance_probability must be in [0, 1]")
        if rng_cross_n_mode not in ("legacy_nested", "strict_independent"):
            raise ValueError(
                "rng_cross_n_mode must be 'legacy_nested' or 'strict_independent'"
            )
        self.n_segments = int(n_segments)
        self.n_ctrl = int(n_ctrl)
        self.indices = indices
        self.canonical_compute_ctrl = canonical_compute_ctrl
        self.seed = int(seed)
        self.stance_probability = float(stance_probability)
        self.rng_cross_n_mode = rng_cross_n_mode
        self.gait_period_s = 2.0 * math.pi / OMEGA_BODY
        self.decision_window_s = 0.5 * self.gait_period_s
        self._state_cache: dict[int, np.ndarray] = {}

    def period_index(self, decision_time_s: float) -> int:
        return int(math.floor((decision_time_s + 1.0e-12) / self.decision_window_s))

    def _draw_state(self, period_index: int) -> np.ndarray:
        state = np.zeros((self.n_segments, 2), dtype=bool)
        encoded_period = int(period_index + PERIOD_SEED_OFFSET)
        if encoded_period < 0:
            raise ValueError("decision-time warmup exceeds period seed offset")
        for segment_index in range(self.n_segments):
            for side_index in range(2):
                entropy = [self.seed]
                if self.rng_cross_n_mode == "strict_independent":
                    entropy.append(self.n_segments)
                entropy.extend(
                    [encoded_period, segment_index, side_index, 16_023]
                )
                rng = np.random.default_rng(
                    np.random.SeedSequence(entropy)
                )
                state[segment_index, side_index] = bool(
                    rng.random() < self.stance_probability
                )
        return state

    def stance_flags(self, decision_time_s: float) -> np.ndarray:
        period_index = self.period_index(decision_time_s)
        if period_index not in self._state_cache:
            self._state_cache[period_index] = self._draw_state(period_index)
        return self._state_cache[period_index].copy()

    def control(self, body_time_s: float, decision_time_s: float) -> np.ndarray:
        ctrl = self.canonical_compute_ctrl(
            body_time_s,
            self.n_ctrl,
            self.n_segments,
            self.indices,
            omega_spine=OMEGA_BODY,
            phi_spine=PHI_BODY,
            dutyfactor=DUTY_FACTOR,
        )
        shoulder_r, leg_r, shoulder_l, leg_l, _link_y, _link_p = self.indices
        del shoulder_r, shoulder_l
        stance = self.stance_flags(decision_time_s)
        up = math.radians(LEG_UP_DEG)
        down = math.radians(LEG_DOWN_DEG)
        for segment_index in range(self.n_segments):
            ctrl[leg_r[segment_index]] = (
                down if stance[segment_index, 0] else up
            )
            ctrl[leg_l[segment_index]] = (
                down if stance[segment_index, 1] else up
            )
        return ctrl


class DeterministicAblationController:
    """Canonical gait or a locked-pitch body-wave ablation."""

    def __init__(
        self,
        n_segments: int,
        n_ctrl: int,
        indices,
        canonical_compute_ctrl,
        *,
        mode: str,
    ) -> None:
        if mode not in (REGULAR_MODE, *ABLATION_MODES):
            raise ValueError(f"unsupported deterministic mode: {mode}")
        self.n_segments = int(n_segments)
        self.n_ctrl = int(n_ctrl)
        self.indices = indices
        self.canonical_compute_ctrl = canonical_compute_ctrl
        self.mode = mode

    @property
    def pitch_locked_up(self) -> bool:
        return self.mode.endswith("pitch_locked_up")

    @property
    def pitch_locked_down(self) -> bool:
        return self.mode.endswith("pitch_locked_down")

    @property
    def leg_yaw_enabled(self) -> bool:
        return self.mode == REGULAR_MODE or "leg_yaw" in self.mode

    def stance_flags(self, body_time_s: float) -> np.ndarray:
        if self.pitch_locked_up:
            return np.zeros((self.n_segments, 2), dtype=bool)
        if self.pitch_locked_down:
            return np.ones((self.n_segments, 2), dtype=bool)
        phase = (
            OMEGA_BODY * body_time_s
            + np.arange(self.n_segments)[:, None] * PHI_BODY
            + np.asarray([[0.0, math.pi]])
        )
        return np.cos(phase) >= 0.0

    def control(self, body_time_s: float, _decision_time_s: float) -> np.ndarray:
        ctrl = self.canonical_compute_ctrl(
            body_time_s,
            self.n_ctrl,
            self.n_segments,
            self.indices,
            omega_spine=OMEGA_BODY,
            phi_spine=PHI_BODY,
            dutyfactor=DUTY_FACTOR,
        )
        shoulder_r, leg_r, shoulder_l, leg_l, _link_y, _link_p = self.indices
        if not self.leg_yaw_enabled:
            neutral_r = math.radians(LEG_OFFSET_DEG)
            neutral_l = -neutral_r
            ctrl[shoulder_r] = neutral_r
            ctrl[shoulder_l] = neutral_l
        if self.pitch_locked_up or self.pitch_locked_down:
            pitch_target = math.radians(
                LEG_UP_DEG if self.pitch_locked_up else LEG_DOWN_DEG
            )
            ctrl[leg_r] = pitch_target
            ctrl[leg_l] = pitch_target
        return ctrl


def _make_controller(
    mode: str,
    n_segments: int,
    n_ctrl: int,
    indices,
    canonical_compute_ctrl,
    seed: int,
):
    if mode == RANDOM_MODE:
        return FixedWindowIIDLegStateController(
            n_segments,
            n_ctrl,
            indices,
            canonical_compute_ctrl,
            seed=seed,
        )
    return DeterministicAblationController(
        n_segments,
        n_ctrl,
        indices,
        canonical_compute_ctrl,
        mode=mode,
    )


@dataclass
class PreparedSimulation:
    model: object
    data: object
    controller: object
    root_body_id: int
    body_map: dict[int, tuple[int, int]]
    dt: float
    warmup_time_s: float
    # Optional physical body whose orientation defines "upright".  This is
    # distinct from root_body_id for center-rooted representations, where the
    # freejoint frame may be a motor/CAD frame with a non-upright local z axis.
    stability_body_id: int | None = None


def _prepare_simulation(
    repo_root: Path,
    *,
    n_segments: int,
    seed: int,
    mode: str,
    warmup_cycles: int,
) -> PreparedSimulation:
    import mujoco

    build_actuator_indices, canonical_compute_ctrl = _install_project_imports(repo_root)
    xml_path = repo_root / "centipede_sim" / "model" / f"centipede_{n_segments}seg.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    expected_ctrl = n_segments * 4 + (n_segments - 1) * 2
    if model.nu != expected_ctrl:
        raise RuntimeError(f"expected {expected_ctrl} actuators, found {model.nu}")
    indices = build_actuator_indices(n_segments)
    controller = _make_controller(
        mode,
        n_segments,
        model.nu,
        indices,
        canonical_compute_ctrl,
        seed,
    )
    root_body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_root")
    )
    if root_body_id < 0:
        raise KeyError("robot_root body is missing")
    body_map = _leg_body_map(model, n_segments, mujoco)
    dt = float(model.opt.timestep)
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    warmup_steps = int(math.ceil(warmup_cycles * gait_period_s / dt))
    warmup_time_s = warmup_steps * dt

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

    bridge_target = controller.control(0.0, -warmup_time_s)
    bridge_steps = int(math.ceil(BRIDGE_TIME_S / dt))
    for step in range(bridge_steps):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1.0 - alpha) * initial_target + alpha * bridge_target
        mujoco.mj_step(model, data)

    for step in range(warmup_steps):
        body_time_s = step * dt
        decision_time_s = body_time_s - warmup_time_s
        data.ctrl[:] = controller.control(body_time_s, decision_time_s)
        mujoco.mj_step(model, data)

    return PreparedSimulation(
        model=model,
        data=data,
        controller=controller,
        root_body_id=root_body_id,
        body_map=body_map,
        dt=dt,
        warmup_time_s=warmup_time_s,
    )


def _run_trial(
    repo_root: Path,
    *,
    n_segments: int,
    seed: int,
    mode: str,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
    trajectory_dir: Path,
) -> dict:
    import mujoco

    prepared = _prepare_simulation(
        repo_root,
        n_segments=n_segments,
        seed=seed,
        mode=mode,
        warmup_cycles=warmup_cycles,
    )
    model = prepared.model
    data = prepared.data
    controller = prepared.controller
    dt = prepared.dt
    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    evaluation_steps = int(math.ceil(evaluation_cycles * gait_period_s / dt))
    evaluation_time_s = evaluation_steps * dt
    sample_stride = max(1, int(round(1.0 / (sample_hz * dt))))

    stability_body_id = prepared.stability_body_id

    def stability_qpos() -> np.ndarray:
        if stability_body_id is None:
            return np.asarray(data.qpos)
        value = np.zeros(7, dtype=float)
        value[3:7] = np.asarray(data.xquat[stability_body_id])
        return value

    def stability_z() -> float:
        if stability_body_id is None:
            return float(data.qpos[2])
        return float(data.xpos[stability_body_id][2])

    start_com = np.asarray(data.subtree_com[prepared.root_body_id]).copy()
    start_root_z = stability_z()
    min_root_z = start_root_z
    min_root_up_z = _root_up_z(stability_qpos())
    max_abs_roll_deg = 0.0
    max_abs_pitch_deg = 0.0
    max_abs_qacc = 0.0
    all_finite = True
    mechanical_absolute_work_j = 0.0
    mechanical_positive_work_j = 0.0

    times: list[float] = []
    coms: list[np.ndarray] = []
    roots: list[np.ndarray] = []
    planned_states: list[np.ndarray] = []
    actual_contacts: list[np.ndarray] = []
    decision_indices: list[int] = []

    for step in range(evaluation_steps):
        evaluation_time = step * dt
        body_time_s = prepared.warmup_time_s + evaluation_time
        data.ctrl[:] = controller.control(body_time_s, evaluation_time)
        mujoco.mj_step(model, data)

        power = np.asarray(data.actuator_force) * np.asarray(data.actuator_velocity)
        mechanical_absolute_work_j += float(np.sum(np.abs(power))) * dt
        mechanical_positive_work_j += float(np.sum(np.maximum(power, 0.0))) * dt
        all_finite &= bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and np.all(np.isfinite(data.qacc))
        )
        max_abs_qacc = max(max_abs_qacc, float(np.max(np.abs(data.qacc))))
        min_root_z = min(min_root_z, stability_z())
        roll_deg, pitch_deg = _root_roll_pitch_deg(stability_qpos())
        max_abs_roll_deg = max(max_abs_roll_deg, abs(roll_deg))
        max_abs_pitch_deg = max(max_abs_pitch_deg, abs(pitch_deg))
        min_root_up_z = min(min_root_up_z, _root_up_z(stability_qpos()))

        if step % sample_stride == 0 or step == evaluation_steps - 1:
            times.append((step + 1) * dt)
            coms.append(np.asarray(data.subtree_com[prepared.root_body_id]).copy())
            roots.append(np.asarray(data.qpos[:7]).copy())
            if mode == RANDOM_MODE:
                stance = controller.stance_flags(evaluation_time)
                decision_index = controller.period_index(evaluation_time)
            else:
                stance = controller.stance_flags(body_time_s)
                decision_index = -1
            planned_states.append(stance)
            actual_contacts.append(
                _actual_contact_flags(model, data, prepared.body_map)
            )
            decision_indices.append(decision_index)

    final_com = np.asarray(data.subtree_com[prepared.root_body_id]).copy()
    displacement = final_com - start_com
    states = np.asarray(planned_states, dtype=bool)
    contacts = np.asarray(actual_contacts, dtype=bool)
    decision_index_array = np.asarray(decision_indices, dtype=int)
    flat_states = states.reshape(states.shape[0], -1).astype(float)
    mean_abs_corr, max_abs_corr = _binary_pairwise_correlation(flat_states)
    down_count = int(np.sum(states))
    up_count = int(states.size - down_count)
    contact_given_down = (
        float(np.sum(contacts & states) / down_count) if down_count else float("nan")
    )
    contact_given_up = (
        float(np.sum(contacts & ~states) / up_count) if up_count else float("nan")
    )
    both_down_fraction = float(np.mean(np.all(states, axis=2)))
    both_up_fraction = float(np.mean(np.all(~states, axis=2)))
    one_down_fraction = float(np.mean(np.sum(states, axis=2) == 1))

    window_state_constant = True
    if mode == RANDOM_MODE:
        for period_index in np.unique(decision_index_array):
            selected = states[decision_index_array == period_index]
            window_state_constant &= bool(np.all(selected == selected[0]))

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
        "never_inverted": min_root_up_z > 0.0,
        "trajectory_complete": len(times) >= 2,
        "window_state_constant": window_state_constant,
    }
    status = "PASS" if all(gates.values()) else "FAIL"
    result = {
        "schema_version": 1,
        "status": status,
        "mode": mode,
        "experiment_family": (
            "fixed_window_random" if mode == RANDOM_MODE else "pitch_ablation"
        ),
        "n_segments": n_segments,
        "n_legs": 2 * n_segments,
        "seed": seed,
        "warmup_cycles": warmup_cycles,
        "warmup_time_s": prepared.warmup_time_s,
        "evaluation_cycles": evaluation_cycles,
        "evaluation_time_s": evaluation_time_s,
        "decision_window_s": 0.5 * gait_period_s if mode == RANDOM_MODE else float("nan"),
        "decision_windows_per_gait_cycle": 2 if mode == RANDOM_MODE else 0,
        "yaw_half_amplitude_deg": YAW_HALF_AMPLITUDE_DEG,
        "omega_body_rad_s": OMEGA_BODY,
        "phi_body_rad": PHI_BODY,
        "forward_displacement_m": float(displacement[1]),
        "lateral_displacement_m": float(displacement[0]),
        "planar_displacement_m": float(np.linalg.norm(displacement[:2])),
        "vertical_displacement_m": float(displacement[2]),
        "mechanical_absolute_work_j": mechanical_absolute_work_j,
        "mechanical_positive_work_j": mechanical_positive_work_j,
        "commanded_down_fraction": float(np.mean(states)),
        "both_down_pair_fraction": both_down_fraction,
        "both_up_pair_fraction": both_up_fraction,
        "one_down_pair_fraction": one_down_fraction,
        "actual_contact_fraction": float(np.mean(contacts)),
        "mean_simultaneous_actual_contacts": float(
            np.mean(np.sum(contacts, axis=(1, 2)))
        ),
        "contact_given_commanded_down": contact_given_down,
        "contact_given_commanded_up": contact_given_up,
        "mean_abs_pairwise_state_correlation": mean_abs_corr,
        "max_abs_pairwise_state_correlation": max_abs_corr,
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

    trajectory_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        trajectory_dir / f"N{n_segments}_{mode}_seed{seed:03d}.npz",
        time_s=np.asarray(times),
        com_xyz_m=np.asarray(coms),
        root_qpos=np.asarray(roots),
        planned_stance=states,
        actual_contact=contacts,
        decision_window_index=decision_index_array,
        start_com_xyz_m=start_com,
    )
    return result


def _aggregate(trials: list[dict]) -> list[dict]:
    rows = []
    for mode in ALL_MODES:
        for n_segments in sorted({int(row["n_segments"]) for row in trials}):
            selected_all = [
                row
                for row in trials
                if row["mode"] == mode and int(row["n_segments"]) == n_segments
            ]
            if not selected_all:
                continue
            selected = [row for row in selected_all if row["status"] == "PASS"]
            if not selected:
                continue
            values = np.asarray(
                [row["forward_displacement_m"] for row in selected], dtype=float
            )
            sd = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
            ci95 = 1.96 * sd / math.sqrt(len(values)) if len(values) > 1 else 0.0
            rows.append(
                {
                    "mode": mode,
                    "n_segments": n_segments,
                    "n_legs": 2 * n_segments,
                    "n_trials": len(selected_all),
                    "n_stable": len(selected),
                    "stable_fraction": len(selected) / len(selected_all),
                    "mean_forward_displacement_m": float(np.mean(values)),
                    "sd_forward_displacement_m": sd,
                    "ci95_forward_displacement_m": ci95,
                    "median_forward_displacement_m": float(np.median(values)),
                    "min_forward_displacement_m": float(np.min(values)),
                    "max_forward_displacement_m": float(np.max(values)),
                    "mean_planar_displacement_m": float(
                        np.mean([row["planar_displacement_m"] for row in selected])
                    ),
                    "mean_mechanical_absolute_work_j": float(
                        np.mean(
                            [row["mechanical_absolute_work_j"] for row in selected]
                        )
                    ),
                    "mean_commanded_down_fraction": float(
                        np.mean([row["commanded_down_fraction"] for row in selected])
                    ),
                    "mean_both_down_pair_fraction": float(
                        np.mean([row["both_down_pair_fraction"] for row in selected])
                    ),
                    "mean_both_up_pair_fraction": float(
                        np.mean([row["both_up_pair_fraction"] for row in selected])
                    ),
                    "mean_one_down_pair_fraction": float(
                        np.mean([row["one_down_pair_fraction"] for row in selected])
                    ),
                    "mean_actual_contact_fraction": float(
                        np.mean([row["actual_contact_fraction"] for row in selected])
                    ),
                    "mean_simultaneous_actual_contacts": float(
                        np.mean(
                            [
                                row["mean_simultaneous_actual_contacts"]
                                for row in selected
                            ]
                        )
                    ),
                }
            )
    return rows


def _flatten_trials(trials: list[dict]) -> list[dict]:
    rows = []
    for row in trials:
        flat = {key: value for key, value in row.items() if key != "gates"}
        flat.update({f"gate_{key}": value for key, value in row["gates"].items()})
        rows.append(flat)
    return rows


def _plot_results(output_dir: Path, trials: list[dict], summary: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    segments = (4, 6, 8)
    colors = {4: "#3366AA", 6: "#DD8822", 8: "#228855"}

    random_rows = [
        row for row in trials if row["mode"] == RANDOM_MODE and row["status"] == "PASS"
    ]
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    rng = np.random.default_rng(20260823)
    for index, n_segments in enumerate(segments):
        values = np.asarray(
            [
                row["forward_displacement_m"]
                for row in random_rows
                if int(row["n_segments"]) == n_segments
            ]
        )
        ax.scatter(
            index + rng.uniform(-0.08, 0.08, len(values)),
            values,
            s=25,
            alpha=0.50,
            color=colors[n_segments],
            edgecolors="none",
        )
        item = next(
            row
            for row in summary
            if row["mode"] == RANDOM_MODE and row["n_segments"] == n_segments
        )
        ax.errorbar(
            index,
            item["mean_forward_displacement_m"],
            yerr=item["ci95_forward_displacement_m"],
            fmt="o",
            color=colors[n_segments],
            markeredgecolor="black",
            markeredgewidth=0.6,
            markersize=8,
            capsize=5,
            zorder=5,
        )
    ax.axhline(0.0, color="#777777", linewidth=0.9)
    ax.set_xticks(range(3), [str(value) for value in segments])
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("Forward COM displacement (m)")
    ax.set_title("Fixed-window IID leg states")
    ax.grid(axis="y", alpha=0.25)
    fig.savefig(figure_dir / "random_forward_displacement_vs_segments.png", dpi=220)
    plt.close(fig)

    grouped = {
        n_segments: np.asarray(
            [
                row["forward_displacement_m"]
                for row in random_rows
                if int(row["n_segments"]) == n_segments
            ]
        )
        for n_segments in segments
    }
    all_values = np.concatenate(list(grouped.values()))
    lower = float(np.min(all_values))
    upper = float(np.max(all_values))
    padding = max(0.02, 0.05 * max(upper - lower, 0.02))
    edges = np.linspace(lower - padding, upper + padding, 14)
    fig, axes = plt.subplots(1, 3, figsize=(10.8, 3.7), sharex=True, sharey=True)
    fig.subplots_adjust(left=0.08, bottom=0.17, right=0.99, top=0.82, wspace=0.12)
    for axis, n_segments in zip(axes, segments):
        values = grouped[n_segments]
        axis.hist(
            values,
            bins=edges,
            color=colors[n_segments],
            alpha=0.82,
            edgecolor="white",
        )
        axis.axvline(float(np.mean(values)), color="#222222", linewidth=1.6)
        axis.set_title(f"N={n_segments}  (n={len(values)})")
        axis.set_xlabel("Forward COM displacement (m)")
        axis.grid(axis="y", alpha=0.20)
    axes[0].set_ylabel("Trial count")
    fig.suptitle("Fixed-window IID displacement distributions", fontsize=14)
    fig.savefig(figure_dir / "random_forward_displacement_histograms.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    for n_segments in segments:
        values = np.sort(grouped[n_segments])
        probability = np.arange(1, len(values) + 1) / len(values)
        ax.step(
            values,
            probability,
            where="post",
            linewidth=2.0,
            color=colors[n_segments],
            label=f"N={n_segments}",
        )
    ax.axvline(0.0, color="#777777", linewidth=0.9)
    ax.set_xlabel("Forward COM displacement (m)")
    ax.set_ylabel("Empirical cumulative probability")
    ax.set_ylim(0.0, 1.02)
    ax.set_title("Fixed-window IID displacement distributions")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.savefig(figure_dir / "random_forward_displacement_ecdf.png", dpi=220)
    plt.close(fig)

    palette = {
        REGULAR_MODE: "#222222",
        "body_wave_pitch_locked_up": "#CC6677",
        "body_wave_pitch_locked_down": "#4477AA",
        "body_wave_leg_yaw_pitch_locked_up": "#DDCC77",
        "body_wave_leg_yaw_pitch_locked_down": "#228833",
    }
    markers = {
        REGULAR_MODE: "D",
        "body_wave_pitch_locked_up": "o",
        "body_wave_pitch_locked_down": "s",
        "body_wave_leg_yaw_pitch_locked_up": "^",
        "body_wave_leg_yaw_pitch_locked_down": "v",
    }
    fig, ax = plt.subplots(figsize=(8.2, 5.1))
    fig.subplots_adjust(left=0.13, bottom=0.14, right=0.98, top=0.90)
    for mode in (REGULAR_MODE, *ABLATION_MODES):
        values = [
            next(
                row["mean_forward_displacement_m"]
                for row in summary
                if row["mode"] == mode and row["n_segments"] == n_segments
            )
            for n_segments in segments
        ]
        ax.plot(
            segments,
            values,
            marker=markers[mode],
            color=palette[mode],
            linewidth=2.0,
            markersize=7,
            label=_mode_label(mode),
        )
    ax.axhline(0.0, color="#777777", linewidth=0.9)
    ax.set_xticks(segments)
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("Forward COM displacement (m)")
    ax.set_title("Locked-pitch locomotion ablations")
    ax.grid(alpha=0.25)
    ax.legend(frameon=False, fontsize=9)
    fig.savefig(figure_dir / "ablation_forward_displacement_vs_segments.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(11.2, 3.8), sharey=True)
    fig.subplots_adjust(left=0.08, bottom=0.17, right=0.99, top=0.82, wspace=0.14)
    for axis, n_segments in zip(axes, segments):
        for mode in (REGULAR_MODE, *ABLATION_MODES):
            path = (
                output_dir
                / "trajectories"
                / f"N{n_segments}_{mode}_seed000.npz"
            )
            with np.load(path) as payload:
                time_s = np.asarray(payload["time_s"])
                com = np.asarray(payload["com_xyz_m"])
                start_com = np.asarray(payload["start_com_xyz_m"])
            axis.plot(
                time_s,
                com[:, 1] - start_com[1],
                color=palette[mode],
                linewidth=1.4,
                label=_mode_label(mode),
            )
        axis.axhline(0.0, color="#777777", linewidth=0.8)
        axis.set_title(f"N={n_segments}")
        axis.set_xlabel("Evaluation time (s)")
        axis.grid(alpha=0.22)
    axes[0].set_ylabel("Forward COM displacement (m)")
    axes[-1].legend(frameon=False, fontsize=7, loc="upper left")
    fig.suptitle("Locked-pitch displacement trajectories", fontsize=14)
    fig.savefig(figure_dir / "ablation_forward_displacement_trajectories.png", dpi=220)
    plt.close(fig)


def _representative_seeds(trials: list[dict]) -> dict[int, int]:
    selected = {}
    for n_segments in (4, 6, 8):
        rows = [
            row
            for row in trials
            if row["mode"] == RANDOM_MODE
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


def _plot_random_schedule(
    output_dir: Path, *, n_segments: int, seed: int
) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch

    path = (
        output_dir
        / "trajectories"
        / f"N{n_segments}_{RANDOM_MODE}_seed{seed:03d}.npz"
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
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 6.2), sharex=True)
    fig.subplots_adjust(left=0.10, bottom=0.11, right=0.98, top=0.90, hspace=0.18)
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
    for axis, title in zip(axes, ("Commanded pitch state", "Measured ground contact")):
        axis.set_yticks(range(1, len(labels) + 1), labels)
        axis.set_ylabel("Physical leg")
        axis.set_title(title, loc="left", fontsize=11)
    axes[1].set_xlabel("Evaluation time (s)")
    fig.suptitle(f"Fixed-window IID decisions | N={n_segments}, seed={seed}")
    axes[0].legend(
        handles=(
            Patch(facecolor="#2aa657", label="down / stance command"),
            Patch(facecolor="#e6452b", label="up / swing command"),
        ),
        frameon=False,
        ncol=2,
        loc="upper right",
        bbox_to_anchor=(1.0, 1.17),
    )
    output_path = figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    output_path = figure_dir / f"N{n_segments}_fixed_window_state_schedule.png"
    fig.savefig(output_path, dpi=220)
    plt.close(fig)
    return output_path


def _compose_single_frame(
    pixels: np.ndarray,
    *,
    n_segments: int,
    seed: int,
    time_s: float,
    displacement_m: float,
    stance: np.ndarray,
    actual_contact: np.ndarray,
    period_index: int,
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
        f"Fixed-window IID leg states | N={n_segments} | seed={seed}",
        font=_font(23, bold=True),
        fill="#17202a",
    )
    draw.text(
        (18, height + header + 5),
        (
            f"t={time_s:5.2f} s   window={period_index:02d}   "
            f"forward COM={displacement_m:+.3f} m   "
            f"down/contact={int(np.sum(stance))}/{int(np.sum(actual_contact))}"
        ),
        font=_font(16),
        fill="#273746",
    )
    draw.ellipse((width - 300, 17, width - 284, 33), fill="#2aa657")
    draw.text((width - 277, 12), "down", font=_font(16), fill="#17202a")
    draw.ellipse((width - 180, 17, width - 164, 33), fill="#e6452b")
    draw.text((width - 157, 12), "up", font=_font(16), fill="#17202a")
    return np.asarray(canvas)


def _camera_for(model, root_body_id: int, n_segments: int, mujoco):
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
    return camera, option


def _render_random_demo(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    seed: int,
    warmup_cycles: int,
    duration_s: float,
    fps: int,
) -> dict:
    import imageio.v2 as imageio
    import mujoco

    prepared = _prepare_simulation(
        repo_root,
        n_segments=n_segments,
        seed=seed,
        mode=RANDOM_MODE,
        warmup_cycles=warmup_cycles,
    )
    model, data, controller = prepared.model, prepared.data, prepared.controller
    start_com = np.asarray(data.subtree_com[prepared.root_body_id]).copy()
    render_height, render_width = 630, 1280
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, render_width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, render_height)
    renderer = mujoco.Renderer(model, height=render_height, width=render_width)
    camera, option = _camera_for(model, prepared.root_body_id, n_segments, mujoco)
    demo_dir = output_dir / "demos" / "random"
    demo_dir.mkdir(parents=True, exist_ok=True)
    video_path = demo_dir / f"N{n_segments}_{RANDOM_MODE}_seed{seed:03d}.mp4"
    preview_path = demo_dir / f"N{n_segments}_{RANDOM_MODE}_seed{seed:03d}_preview.png"
    writer = imageio.get_writer(
        str(video_path),
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=2,
    )
    steps = int(math.ceil(duration_s / prepared.dt))
    next_frame_time = 0.0
    frame_count = 0
    preview_saved = False
    try:
        for step in range(steps):
            time_s = step * prepared.dt
            body_time_s = prepared.warmup_time_s + time_s
            data.ctrl[:] = controller.control(body_time_s, time_s)
            mujoco.mj_step(model, data)
            time_end = (step + 1) * prepared.dt
            if time_end + 0.5 * prepared.dt >= next_frame_time:
                stance = controller.stance_flags(time_s)
                contact = _actual_contact_flags(model, data, prepared.body_map)
                _set_leg_colors(model, prepared.body_map, stance)
                renderer.update_scene(data, camera=camera, scene_option=option)
                frame = _compose_single_frame(
                    renderer.render().copy(),
                    n_segments=n_segments,
                    seed=seed,
                    time_s=time_end,
                    displacement_m=float(
                        data.subtree_com[prepared.root_body_id][1] - start_com[1]
                    ),
                    stance=stance,
                    actual_contact=contact,
                    period_index=controller.period_index(time_s),
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
        "family": "fixed_window_random",
        "n_segments": n_segments,
        "seed": seed,
        "duration_s": duration_s,
        "fps": fps,
        "frame_count": frame_count,
        "video": str(video_path),
        "preview": str(preview_path),
    }


def _render_ablation_grid_demo(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    warmup_cycles: int,
    duration_s: float,
    fps: int,
) -> dict:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    modes = ABLATION_MODES
    prepared_list = [
        _prepare_simulation(
            repo_root,
            n_segments=n_segments,
            seed=0,
            mode=mode,
            warmup_cycles=warmup_cycles,
        )
        for mode in modes
    ]
    panel_width, render_height = 640, 294
    renderers = []
    cameras = []
    options = []
    starts = []
    for prepared in prepared_list:
        prepared.model.vis.global_.offwidth = max(
            prepared.model.vis.global_.offwidth, panel_width
        )
        prepared.model.vis.global_.offheight = max(
            prepared.model.vis.global_.offheight, render_height
        )
        renderers.append(
            mujoco.Renderer(
                prepared.model, height=render_height, width=panel_width
            )
        )
        camera, option = _camera_for(
            prepared.model, prepared.root_body_id, n_segments, mujoco
        )
        cameras.append(camera)
        options.append(option)
        starts.append(
            np.asarray(prepared.data.subtree_com[prepared.root_body_id]).copy()
        )

    demo_dir = output_dir / "demos" / "ablations"
    demo_dir.mkdir(parents=True, exist_ok=True)
    video_path = demo_dir / f"N{n_segments}_locked_pitch_ablation_grid.mp4"
    preview_path = demo_dir / f"N{n_segments}_locked_pitch_ablation_grid_preview.png"
    writer = imageio.get_writer(
        str(video_path),
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=2,
    )
    dt = prepared_list[0].dt
    steps = int(math.ceil(duration_s / dt))
    next_frame_time = 0.0
    frame_count = 0
    preview_saved = False
    try:
        for step in range(steps):
            time_s = step * dt
            for prepared in prepared_list:
                body_time_s = prepared.warmup_time_s + time_s
                prepared.data.ctrl[:] = prepared.controller.control(
                    body_time_s, time_s
                )
                mujoco.mj_step(prepared.model, prepared.data)
            time_end = (step + 1) * dt
            if time_end + 0.5 * dt >= next_frame_time:
                panels = []
                for mode, prepared, renderer, camera, option, start_com in zip(
                    modes,
                    prepared_list,
                    renderers,
                    cameras,
                    options,
                    starts,
                ):
                    body_time_s = prepared.warmup_time_s + time_s
                    stance = prepared.controller.stance_flags(body_time_s)
                    _set_leg_colors(prepared.model, prepared.body_map, stance)
                    renderer.update_scene(
                        prepared.data, camera=camera, scene_option=option
                    )
                    image = Image.fromarray(renderer.render().copy())
                    panel = Image.new(
                        "RGB", (panel_width, render_height + 58), "#f3f4f6"
                    )
                    panel.paste(image, (0, 34))
                    draw = ImageDraw.Draw(panel)
                    draw.text(
                        (12, 5),
                        _mode_label(mode),
                        font=_font(18, bold=True),
                        fill="#17202a",
                    )
                    displacement = float(
                        prepared.data.subtree_com[prepared.root_body_id][1]
                        - start_com[1]
                    )
                    draw.text(
                        (12, render_height + 38),
                        f"forward COM={displacement:+.3f} m",
                        font=_font(15),
                        fill="#273746",
                    )
                    panels.append(panel)
                header = 42
                canvas = Image.new(
                    "RGB",
                    (2 * panel_width, header + 2 * (render_height + 58)),
                    "#e9ecef",
                )
                draw = ImageDraw.Draw(canvas)
                draw.text(
                    (16, 8),
                    f"Locked-pitch ablations | N={n_segments} | t={time_end:5.2f} s",
                    font=_font(22, bold=True),
                    fill="#17202a",
                )
                canvas.paste(panels[0], (0, header))
                canvas.paste(panels[1], (panel_width, header))
                canvas.paste(panels[2], (0, header + render_height + 58))
                canvas.paste(
                    panels[3],
                    (panel_width, header + render_height + 58),
                )
                frame = np.asarray(canvas)
                writer.append_data(frame)
                frame_count += 1
                if not preview_saved and time_end >= 0.5 * duration_s:
                    canvas.save(preview_path)
                    preview_saved = True
                next_frame_time += 1.0 / fps
    finally:
        writer.close()
        for renderer in renderers:
            renderer.close()
    return {
        "family": "locked_pitch_ablation_grid",
        "n_segments": n_segments,
        "duration_s": duration_s,
        "fps": fps,
        "frame_count": frame_count,
        "video": str(video_path),
        "preview": str(preview_path),
    }


def _write_readme(
    output_dir: Path,
    summary: list[dict],
    demo_manifest: list[dict],
) -> None:
    lines = [
        "# Fixed-window IID leg states and locked-pitch ablations",
        "",
        "This directory is independent of Step 15. The body wave is locked at "
        "yaw half-amplitude 15 deg, omega=4 rad/s, and spatial phase increment -pi.",
        "",
        "## Fixed-window random process",
        "",
        "One gait cycle is divided into two decision windows. Each window has "
        "duration T_g/2=pi/omega. At every window, every physical leg independently "
        "draws down/stance with probability 0.5; all decisions are independent "
        "across segment, side, and window. The canonical leg-yaw oscillator remains "
        "deterministic. Pair states SS, SW, WS, and WW therefore each have probability 0.25.",
        "",
        "## Locked-pitch ablations",
        "",
        "Leg-pitch position targets are held fixed for the whole trial at either "
        "-40 deg (up) or +5 deg (down). The pitch joint remains physically present "
        "and actuated; there is no pitch switching. Pure body-wave modes hold leg-yaw "
        "at the neutral offsets, while body-wave-plus-leg-yaw modes retain the "
        "canonical Shoulder*_R/L oscillator.",
        "",
        "All reported displacement is signed whole-robot COM displacement along +y "
        "after a four-cycle warmup. No normalized displacement is used.",
        "",
        "## Summary",
        "",
        "| mode | N | stable / total | mean forward displacement (m) | SD (m) |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            f"| {row['mode']} | {row['n_segments']} | "
            f"{row['n_stable']} / {row['n_trials']} | "
            f"{row['mean_forward_displacement_m']:.4f} | "
            f"{row['sd_forward_displacement_m']:.4f} |"
        )
    lines += [
        "",
        "## Evidence",
        "",
        "- `trials.csv`: all stability gates and scalar trial results.",
        "- `summary.csv`: mode-by-morphology aggregates.",
        "- `trajectories/`: sampled COM, commanded pitch state, measured contact, and decision-window index.",
        "- `figures/`: absolute displacement distributions and locked-pitch comparisons.",
        "- `demos/random/`: one median-displacement IID-state demo for each morphology.",
        "- `demos/ablations/`: synchronized four-condition ablation grid for each morphology.",
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

    for n_segments in args.segments:
        for mode in (REGULAR_MODE, *ABLATION_MODES):
            result = _run_trial(
                repo_root,
                n_segments=n_segments,
                seed=0,
                mode=mode,
                warmup_cycles=args.warmup_cycles,
                evaluation_cycles=args.evaluation_cycles,
                sample_hz=args.sample_hz,
                trajectory_dir=trajectory_dir,
            )
            trials.append(result)
            print(
                f"N={n_segments} {mode}: {result['status']} "
                f"D_y={result['forward_displacement_m']:+.4f} m",
                flush=True,
            )

    for n_segments in args.segments:
        for seed in range(args.seed_start, args.seed_start + args.replays):
            result = _run_trial(
                repo_root,
                n_segments=n_segments,
                seed=seed,
                mode=RANDOM_MODE,
                warmup_cycles=args.warmup_cycles,
                evaluation_cycles=args.evaluation_cycles,
                sample_hz=args.sample_hz,
                trajectory_dir=trajectory_dir,
            )
            trials.append(result)
            print(
                f"N={n_segments} random seed={seed:03d}: {result['status']} "
                f"D_y={result['forward_displacement_m']:+.4f} m",
                flush=True,
            )

    summary = _aggregate(trials)
    _write_csv(output_dir / "trials.csv", _flatten_trials(trials))
    _write_csv(output_dir / "summary.csv", summary)
    _write_json(output_dir / "trials.json", trials)
    _write_json(
        output_dir / "run_configuration.json",
        {
            "schema_version": 1,
            "segments": list(args.segments),
            "random_replays_per_morphology": args.replays,
            "seed_start": args.seed_start,
            "warmup_cycles": args.warmup_cycles,
            "evaluation_cycles": args.evaluation_cycles,
            "sample_hz": args.sample_hz,
            "body_wave": {
                "yaw_half_amplitude_deg": YAW_HALF_AMPLITUDE_DEG,
                "omega_rad_s": OMEGA_BODY,
                "spatial_phase_increment_rad": PHI_BODY,
            },
            "fixed_window_random_process": {
                "decision_window_s": math.pi / OMEGA_BODY,
                "windows_per_gait_cycle": 2,
                "stance_probability_per_leg_per_window": 0.5,
                "independent_across_segments": True,
                "independent_across_sides": True,
                "independent_across_windows": True,
                "leg_yaw_oscillator": "canonical deterministic",
            },
            "locked_pitch_targets_deg": {
                "up": LEG_UP_DEG,
                "down": LEG_DOWN_DEG,
            },
            "primary_metric": "signed whole-robot COM displacement along model +y (m)",
        },
    )
    _plot_results(output_dir, trials, summary)

    demo_manifest = []
    if not args.no_demos:
        representatives = _representative_seeds(trials)
        _plot_random_schedule(
            output_dir,
            n_segments=max(representatives),
            seed=representatives[max(representatives)],
        )
        for n_segments in args.segments:
            demo_manifest.append(
                _render_random_demo(
                    repo_root,
                    output_dir,
                    n_segments=n_segments,
                    seed=representatives[n_segments],
                    warmup_cycles=args.warmup_cycles,
                    duration_s=args.demo_duration,
                    fps=args.demo_fps,
                )
            )
            demo_manifest.append(
                _render_ablation_grid_demo(
                    repo_root,
                    output_dir,
                    n_segments=n_segments,
                    warmup_cycles=args.warmup_cycles,
                    duration_s=args.demo_duration,
                    fps=args.demo_fps,
                )
            )
        _write_json(output_dir / "demos" / "video_manifest.json", demo_manifest)

    _write_readme(output_dir, summary, demo_manifest)
    _write_json(
        output_dir / "summary.json",
        {
            "schema_version": 1,
            "status": "PASS" if all(row["status"] == "PASS" for row in trials) else "FAIL",
            "trial_count": len(trials),
            "summary": summary,
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
        / "step16_fixed_window_iid_and_pitch_ablation",
    )
    parser.add_argument("--segments", type=int, nargs="+", default=list(DEFAULT_SEGMENTS))
    parser.add_argument("--replays", type=int, default=DEFAULT_REPLAYS)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--warmup-cycles", type=int, default=DEFAULT_WARMUP_CYCLES)
    parser.add_argument("--evaluation-cycles", type=int, default=DEFAULT_EVALUATION_CYCLES)
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--demo-duration", type=float, default=8.0)
    parser.add_argument("--demo-fps", type=int, default=30)
    parser.add_argument("--no-demos", action="store_true")
    return parser


if __name__ == "__main__":
    run_experiment(build_parser().parse_args())
