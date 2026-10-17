"""Body-wave-only pitch-pattern and IID leg-pitch experiments.

The lateral body travelling wave is retained at the canonical +/-15 degree
amplitude.  Every Shoulder*_R/L actuator (leg yaw) is locked at its neutral
offset.  Three leg-pitch command processes are compared:

1. the canonical left/right alternating pitch pattern;
2. the same pattern with right and left pitch commands exchanged; and
3. fixed-window IID down/up pitch commands for every physical leg.

The stochastic process randomizes commanded pitch/contact opportunity, not
measured ground contact.  Actual contact is recorded as a dynamical outcome.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    DUTY_FACTOR,
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    LEG_UP_DEG,
    OMEGA_BODY,
    PHI_BODY,
    YAW_HALF_AMPLITUDE_DEG,
    _actual_contact_flags,
    _font,
    _set_leg_colors,
    _write_csv,
    _write_json,
)


ALL_SEGMENTS = tuple(range(4, 11))
MODE_CANONICAL = "body_wave_canonical_pitch_yaw_locked"
MODE_FLIPPED = "body_wave_lr_flipped_pitch_yaw_locked"
MODE_RANDOM = "body_wave_fixed_window_iid_pitch_yaw_locked"
MODES = (MODE_CANONICAL, MODE_FLIPPED, MODE_RANDOM)
MODE_LABELS = {
    MODE_CANONICAL: "Canonical pitch; leg yaw locked",
    MODE_FLIPPED: "L/R-flipped pitch; leg yaw locked",
    MODE_RANDOM: "IID pitch; leg yaw locked",
}
MODE_COLORS = {
    MODE_CANONICAL: "#242424",
    MODE_FLIPPED: "#c95a3d",
    MODE_RANDOM: "#2f78a4",
}
DEFAULT_REPLAYS = 128
DEFAULT_WARMUP_CYCLES = 4
DEFAULT_EVALUATION_CYCLES = 12
DEFAULT_SAMPLE_HZ = 100.0
ACTIVE_SPATIAL_PHASE_INCREMENT_RAD = PHI_BODY
ACTIVE_RNG_CROSS_N_MODE = "legacy_nested"


def _spatial_wavelength_increments() -> float:
    """Return the number of inter-segment phase increments in one 2-pi wave."""
    return 2.0 * math.pi / abs(ACTIVE_SPATIAL_PHASE_INCREMENT_RAD)


def _phase_label() -> str:
    return (
        f"spatial phase={math.degrees(ACTIVE_SPATIAL_PHASE_INCREMENT_RAD):+.0f} deg "
        f"({_spatial_wavelength_increments():.0f} intervals/wave)"
    )


def _repo_root_from_script() -> Path:
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "centipede_sim" / "model").is_dir():
        return candidate
    return Path(r"C:\Users\wxy22\Documents\Centipede_robot_MUJOCO")


def _lock_leg_yaw(ctrl: np.ndarray, indices) -> None:
    shoulder_r, _leg_r, shoulder_l, _leg_l, _link_y, _link_p = indices
    neutral_r = math.radians(LEG_OFFSET_DEG)
    neutral_l = -neutral_r
    ctrl[shoulder_r] = neutral_r
    ctrl[shoulder_l] = neutral_l


class BodyWaveOnlyIIDController(base.FixedWindowIIDLegStateController):
    """Canonical body wave plus IID pitch states, with leg yaw locked."""

    def control(self, body_time_s: float, decision_time_s: float) -> np.ndarray:
        ctrl = self.canonical_compute_ctrl(
            body_time_s,
            self.n_ctrl,
            self.n_segments,
            self.indices,
            omega_spine=OMEGA_BODY,
            phi_spine=ACTIVE_SPATIAL_PHASE_INCREMENT_RAD,
            dutyfactor=DUTY_FACTOR,
        )
        _shoulder_r, leg_r, _shoulder_l, leg_l, _link_y, _link_p = self.indices
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
        _lock_leg_yaw(ctrl, self.indices)
        return ctrl


class BodyWaveOnlyPatternController:
    """Canonical or L/R-exchanged pitch gait, with leg yaw locked."""

    def __init__(
        self,
        n_segments: int,
        n_ctrl: int,
        indices,
        canonical_compute_ctrl,
        *,
        flipped: bool,
    ) -> None:
        self.n_segments = int(n_segments)
        self.n_ctrl = int(n_ctrl)
        self.indices = indices
        self.canonical_compute_ctrl = canonical_compute_ctrl
        self.flipped = bool(flipped)

    def _canonical_stance(self, body_time_s: float) -> np.ndarray:
        phase = (
            OMEGA_BODY * body_time_s
            + np.arange(self.n_segments)[:, None]
            * ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
            + np.asarray([[0.0, math.pi]])
        )
        return np.cos(phase) >= 0.0

    def stance_flags(self, body_time_s: float) -> np.ndarray:
        stance = self._canonical_stance(body_time_s)
        return stance[:, ::-1].copy() if self.flipped else stance

    def control(self, body_time_s: float, _decision_time_s: float) -> np.ndarray:
        ctrl = self.canonical_compute_ctrl(
            body_time_s,
            self.n_ctrl,
            self.n_segments,
            self.indices,
            omega_spine=OMEGA_BODY,
            phi_spine=ACTIVE_SPATIAL_PHASE_INCREMENT_RAD,
            dutyfactor=DUTY_FACTOR,
        )
        _lock_leg_yaw(ctrl, self.indices)
        if self.flipped:
            _shoulder_r, leg_r, _shoulder_l, leg_l, _link_y, _link_p = self.indices
            right_pitch = ctrl[leg_r].copy()
            left_pitch = ctrl[leg_l].copy()
            ctrl[leg_r] = left_pitch
            ctrl[leg_l] = right_pitch
        return ctrl


def _controller_factory(
    mode: str,
    n_segments: int,
    n_ctrl: int,
    indices,
    canonical_compute_ctrl,
    seed: int,
):
    if mode == base.RANDOM_MODE:
        return BodyWaveOnlyIIDController(
            n_segments,
            n_ctrl,
            indices,
            canonical_compute_ctrl,
            seed=seed,
            stance_probability=0.5,
            rng_cross_n_mode=ACTIVE_RNG_CROSS_N_MODE,
        )
    if mode == MODE_CANONICAL:
        return BodyWaveOnlyPatternController(
            n_segments,
            n_ctrl,
            indices,
            canonical_compute_ctrl,
            flipped=False,
        )
    if mode == MODE_FLIPPED:
        return BodyWaveOnlyPatternController(
            n_segments,
            n_ctrl,
            indices,
            canonical_compute_ctrl,
            flipped=True,
        )
    raise ValueError(f"unsupported body-wave-only mode: {mode}")


def _semantic_validation(repo_root: Path) -> dict:
    build_actuator_indices, canonical_compute_ctrl = base._install_project_imports(
        repo_root
    )
    n_segments = 4
    n_ctrl = n_segments * 4 + (n_segments - 1) * 2
    indices = build_actuator_indices(n_segments)
    canonical = _controller_factory(
        MODE_CANONICAL,
        n_segments,
        n_ctrl,
        indices,
        canonical_compute_ctrl,
        0,
    )
    flipped = _controller_factory(
        MODE_FLIPPED,
        n_segments,
        n_ctrl,
        indices,
        canonical_compute_ctrl,
        0,
    )
    random = _controller_factory(
        base.RANDOM_MODE,
        n_segments,
        n_ctrl,
        indices,
        canonical_compute_ctrl,
        0,
    )
    n_segments_large = 10
    n_ctrl_large = n_segments_large * 4 + (n_segments_large - 1) * 2
    random_large = _controller_factory(
        base.RANDOM_MODE,
        n_segments_large,
        n_ctrl_large,
        build_actuator_indices(n_segments_large),
        canonical_compute_ctrl,
        0,
    )
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, _link_p = indices
    neutral_r = math.radians(LEG_OFFSET_DEG)
    neutral_l = -neutral_r
    checks = {
        "canonical_leg_yaw_locked": True,
        "flipped_leg_yaw_locked": True,
        "random_leg_yaw_locked": True,
        "body_wave_identical_across_conditions": True,
        "flipped_pitch_is_exact_lr_exchange": True,
        "flipped_stance_is_exact_lr_exchange": True,
        "random_state_constant_within_window": True,
        "random_pitch_matches_binary_state": True,
        "random_stream_reproducible": True,
        "random_cross_n_stream_contract": True,
        "spatial_phase_increment_is_finite_nonzero": bool(
            np.isfinite(ACTIVE_SPATIAL_PHASE_INCREMENT_RAD)
            and abs(ACTIVE_SPATIAL_PHASE_INCREMENT_RAD) > 1.0e-12
        ),
    }
    times = np.linspace(0.0, 2.0 * math.pi / OMEGA_BODY, 17, endpoint=False)
    for time_s in times:
        canonical_ctrl = canonical.control(time_s, time_s)
        flipped_ctrl = flipped.control(time_s, time_s)
        random_ctrl = random.control(time_s, time_s)
        checks["canonical_leg_yaw_locked"] &= bool(
            np.allclose(canonical_ctrl[shoulder_r], neutral_r)
            and np.allclose(canonical_ctrl[shoulder_l], neutral_l)
        )
        checks["flipped_leg_yaw_locked"] &= bool(
            np.allclose(flipped_ctrl[shoulder_r], neutral_r)
            and np.allclose(flipped_ctrl[shoulder_l], neutral_l)
        )
        checks["random_leg_yaw_locked"] &= bool(
            np.allclose(random_ctrl[shoulder_r], neutral_r)
            and np.allclose(random_ctrl[shoulder_l], neutral_l)
        )
        checks["body_wave_identical_across_conditions"] &= bool(
            np.allclose(canonical_ctrl[link_y], flipped_ctrl[link_y])
            and np.allclose(canonical_ctrl[link_y], random_ctrl[link_y])
        )
        checks["flipped_pitch_is_exact_lr_exchange"] &= bool(
            np.allclose(flipped_ctrl[leg_r], canonical_ctrl[leg_l])
            and np.allclose(flipped_ctrl[leg_l], canonical_ctrl[leg_r])
        )
        checks["flipped_stance_is_exact_lr_exchange"] &= bool(
            np.array_equal(
                flipped.stance_flags(time_s),
                canonical.stance_flags(time_s)[:, ::-1],
            )
        )
        random_state = random.stance_flags(time_s)
        expected_pitch = np.where(
            random_state,
            math.radians(LEG_DOWN_DEG),
            math.radians(LEG_UP_DEG),
        )
        actual_pitch = np.stack(
            (random_ctrl[leg_r], random_ctrl[leg_l]), axis=1
        )
        checks["random_pitch_matches_binary_state"] &= bool(
            np.allclose(actual_pitch, expected_pitch)
        )
    for window_index in range(4):
        start = window_index * random.decision_window_s
        state_early = random.stance_flags(start + 0.1 * random.decision_window_s)
        state_late = random.stance_flags(start + 0.9 * random.decision_window_s)
        checks["random_state_constant_within_window"] &= bool(
            np.array_equal(state_early, state_late)
        )
    random_replay = _controller_factory(
        base.RANDOM_MODE,
        n_segments,
        n_ctrl,
        indices,
        canonical_compute_ctrl,
        0,
    )
    small_bits = np.concatenate(
        [random.stance_flags(k * random.decision_window_s).ravel() for k in range(8)]
    )
    replay_bits = np.concatenate(
        [
            random_replay.stance_flags(k * random_replay.decision_window_s).ravel()
            for k in range(8)
        ]
    )
    large_overlap_bits = np.concatenate(
        [
            random_large.stance_flags(k * random_large.decision_window_s)[:n_segments].ravel()
            for k in range(8)
        ]
    )
    checks["random_stream_reproducible"] = bool(
        np.array_equal(small_bits, replay_bits)
    )
    cross_n_equal_fraction = float(np.mean(small_bits == large_overlap_bits))
    streams_are_nested = bool(np.array_equal(small_bits, large_overlap_bits))
    checks["random_cross_n_stream_contract"] = (
        not streams_are_nested
        if ACTIVE_RNG_CROSS_N_MODE == "strict_independent"
        else streams_are_nested
    )
    return {
        "checks": checks,
        "all_pass": all(checks.values()),
        "n_segments_checked": n_segments,
        "times_checked": len(times),
        "decision_windows_checked": 4,
        "spatial_phase_increment_rad": ACTIVE_SPATIAL_PHASE_INCREMENT_RAD,
        "spatial_phase_increment_deg": math.degrees(
            ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
        ),
        "spatial_wavelength_increments": _spatial_wavelength_increments(),
        "rng_cross_n_mode": ACTIVE_RNG_CROSS_N_MODE,
        "rng_entropy_fields": (
            ["seed", "N", "window", "segment", "side", "namespace"]
            if ACTIVE_RNG_CROSS_N_MODE == "strict_independent"
            else ["seed", "window", "segment", "side", "namespace"]
        ),
        "cross_n_overlap_bit_count": int(small_bits.size),
        "cross_n_overlap_equal_fraction": cross_n_equal_fraction,
        "cross_n_streams_are_exactly_nested": streams_are_nested,
    }


def _internal_mode(mode: str) -> str:
    return base.RANDOM_MODE if mode == MODE_RANDOM else mode


def _call_with_controller_factory(function, *args, **kwargs):
    original_factory = base._make_controller
    original_phi_body = base.PHI_BODY
    base._make_controller = _controller_factory
    base.PHI_BODY = ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
    try:
        return function(*args, **kwargs)
    finally:
        base._make_controller = original_factory
        base.PHI_BODY = original_phi_body


def _record_path(output_dir: Path, mode: str, n_segments: int, seed: int) -> Path:
    return (
        output_dir
        / "trial_records"
        / mode
        / f"N{n_segments}_seed{seed:03d}.json"
    )


def _run_or_load_trial(
    repo_root: Path,
    output_dir: Path,
    *,
    mode: str,
    n_segments: int,
    seed: int,
    warmup_cycles: int,
    evaluation_cycles: int,
    sample_hz: float,
    force: bool,
) -> dict:
    record_path = _record_path(output_dir, mode, n_segments, seed)
    if record_path.is_file() and not force:
        cached = json.loads(record_path.read_text(encoding="utf-8"))
        cached_rng_mode = cached.get("rng_cross_n_mode", "legacy_nested")
        if mode != MODE_RANDOM or cached_rng_mode == ACTIVE_RNG_CROSS_N_MODE:
            return cached

    trajectory_dir = output_dir / "trajectories" / mode
    result = _call_with_controller_factory(
        base._run_trial,
        repo_root,
        n_segments=n_segments,
        seed=seed,
        mode=_internal_mode(mode),
        warmup_cycles=warmup_cycles,
        evaluation_cycles=evaluation_cycles,
        sample_hz=sample_hz,
        trajectory_dir=trajectory_dir,
    )
    result["mode"] = mode
    result["experiment_family"] = "body_wave_only_pitch_pattern"
    result["body_wave_enabled"] = True
    result["body_yaw_half_amplitude_deg"] = YAW_HALF_AMPLITUDE_DEG
    result["phi_body_rad"] = ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
    result["rng_cross_n_mode"] = (
        ACTIVE_RNG_CROSS_N_MODE if mode == MODE_RANDOM else "not_applicable"
    )
    result["spatial_phase_increment_rad"] = ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
    result["spatial_phase_increment_deg"] = math.degrees(
        ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
    )
    result["spatial_wavelength_increments"] = _spatial_wavelength_increments()
    result["leg_yaw_locked"] = True
    result["leg_yaw_right_target_deg"] = LEG_OFFSET_DEG
    result["leg_yaw_left_target_deg"] = -LEG_OFFSET_DEG
    result["pitch_pattern_flipped_lr"] = mode == MODE_FLIPPED
    result["pitch_commands_iid"] = mode == MODE_RANDOM
    result["command_not_direct_contact_erasure"] = True
    _write_json(record_path, result)
    return result


def _aggregate(trials: list[dict]) -> list[dict]:
    summary = []
    for mode in MODES:
        for n_segments in ALL_SEGMENTS:
            selected_all = [
                row
                for row in trials
                if row["mode"] == mode and int(row["n_segments"]) == n_segments
            ]
            selected = [row for row in selected_all if row["status"] == "PASS"]
            if not selected:
                continue
            displacement = np.asarray(
                [float(row["forward_displacement_m"]) for row in selected]
            )
            contact = np.asarray(
                [float(row["actual_contact_fraction"]) for row in selected]
            )
            sd = float(np.std(displacement, ddof=1)) if len(displacement) > 1 else 0.0
            summary.append(
                {
                    "mode": mode,
                    "n_segments": n_segments,
                    "n_trials": len(selected_all),
                    "n_pass": len(selected),
                    "mean_forward_displacement_m": float(np.mean(displacement)),
                    "sd_forward_displacement_m": sd,
                    "ci95_forward_displacement_m": (
                        1.96 * sd / math.sqrt(len(displacement))
                        if len(displacement) > 1
                        else 0.0
                    ),
                    "median_forward_displacement_m": float(np.median(displacement)),
                    "q05_forward_displacement_m": float(np.quantile(displacement, 0.05)),
                    "q95_forward_displacement_m": float(np.quantile(displacement, 0.95)),
                    "positive_fraction": float(np.mean(displacement > 0.0)),
                    "mean_actual_contact_fraction": float(np.mean(contact)),
                    "mean_contact_given_commanded_down": float(
                        np.mean(
                            [
                                float(row["contact_given_commanded_down"])
                                for row in selected
                            ]
                        )
                    ),
                    "mean_contact_given_commanded_up": float(
                        np.mean(
                            [
                                float(row["contact_given_commanded_up"])
                                for row in selected
                            ]
                        )
                    ),
                }
            )
    return summary


def _comparison_tables(trials: list[dict], summary: list[dict]):
    baseline_rows = []
    random_rows = []
    available_segments = sorted(
        set.intersection(
            *(
                {
                    int(row["n_segments"])
                    for row in summary
                    if row["mode"] == mode
                }
                for mode in MODES
            )
        )
    )
    for n_segments in available_segments:
        canonical = next(
            row
            for row in summary
            if row["mode"] == MODE_CANONICAL and row["n_segments"] == n_segments
        )
        flipped = next(
            row
            for row in summary
            if row["mode"] == MODE_FLIPPED and row["n_segments"] == n_segments
        )
        random_summary = next(
            row
            for row in summary
            if row["mode"] == MODE_RANDOM and row["n_segments"] == n_segments
        )
        values = np.asarray(
            [
                float(row["forward_displacement_m"])
                for row in trials
                if row["mode"] == MODE_RANDOM
                and int(row["n_segments"]) == n_segments
                and row["status"] == "PASS"
            ]
        )
        canonical_value = float(canonical["mean_forward_displacement_m"])
        flipped_value = float(flipped["mean_forward_displacement_m"])
        baseline_rows.append(
            {
                "n_segments": n_segments,
                "canonical_forward_displacement_m": canonical_value,
                "flipped_forward_displacement_m": flipped_value,
                "flipped_minus_canonical_m": flipped_value - canonical_value,
                "larger_baseline": (
                    "flipped" if flipped_value > canonical_value else "canonical"
                ),
            }
        )
        random_rows.append(
            {
                "n_segments": n_segments,
                "n_random_trials": len(values),
                "random_mean_forward_displacement_m": float(
                    random_summary["mean_forward_displacement_m"]
                ),
                "canonical_baseline_m": canonical_value,
                "flipped_baseline_m": flipped_value,
                "fraction_random_above_canonical": float(
                    np.mean(values > canonical_value)
                ),
                "fraction_random_above_flipped": float(
                    np.mean(values > flipped_value)
                ),
            }
        )
    return baseline_rows, random_rows


def _paired_random_comparisons(trials: list[dict]) -> list[dict]:
    lookup = {
        (int(row["n_segments"]), int(row["seed"])): float(
            row["forward_displacement_m"]
        )
        for row in trials
        if row["mode"] == MODE_RANDOM and row["status"] == "PASS"
    }
    pairs = list(zip(ALL_SEGMENTS[:-1], ALL_SEGMENTS[1:])) + [
        (ALL_SEGMENTS[0], ALL_SEGMENTS[-1])
    ]
    rows = []
    for left, right in pairs:
        left_seeds = {seed for n_segments, seed in lookup if n_segments == left}
        right_seeds = {seed for n_segments, seed in lookup if n_segments == right}
        common_seeds = sorted(left_seeds & right_seeds)
        differences = np.asarray(
            [lookup[(right, seed)] - lookup[(left, seed)] for seed in common_seeds]
        )
        sd = float(np.std(differences, ddof=1))
        rows.append(
            {
                "n_segments_left": left,
                "n_segments_right": right,
                "n_paired_seeds": len(common_seeds),
                "mean_difference_right_minus_left_m": float(np.mean(differences)),
                "sd_difference_m": sd,
                "ci95_half_width_m": 1.96 * sd / math.sqrt(len(differences)),
                "fraction_right_greater_than_left": float(
                    np.mean(differences > 0.0)
                ),
            }
        )
    return rows


def _independent_random_comparisons(trials: list[dict]) -> list[dict]:
    """Compare segment counts without pretending that equal seed labels are paired."""
    samples = {
        n_segments: np.asarray(
            [
                float(row["forward_displacement_m"])
                for row in trials
                if row["mode"] == MODE_RANDOM
                and row["status"] == "PASS"
                and int(row["n_segments"]) == n_segments
            ],
            dtype=float,
        )
        for n_segments in ALL_SEGMENTS
    }
    pairs = list(zip(ALL_SEGMENTS[:-1], ALL_SEGMENTS[1:])) + [
        (ALL_SEGMENTS[0], ALL_SEGMENTS[-1])
    ]
    rows = []
    for left, right in pairs:
        left_values = samples[left]
        right_values = samples[right]
        if len(left_values) == 0 or len(right_values) == 0:
            continue
        mean_difference = float(np.mean(right_values) - np.mean(left_values))
        left_variance = (
            float(np.var(left_values, ddof=1)) if len(left_values) > 1 else 0.0
        )
        right_variance = (
            float(np.var(right_values, ddof=1)) if len(right_values) > 1 else 0.0
        )
        standard_error = math.sqrt(
            left_variance / len(left_values) + right_variance / len(right_values)
        )
        rows.append(
            {
                "n_segments_left": left,
                "n_segments_right": right,
                "n_independent_left": len(left_values),
                "n_independent_right": len(right_values),
                "mean_difference_right_minus_left_m": mean_difference,
                "standard_error_m": standard_error,
                "ci95_low_m": mean_difference - 1.96 * standard_error,
                "ci95_high_m": mean_difference + 1.96 * standard_error,
                "empirical_probability_right_greater_than_left": float(
                    np.mean(right_values[:, None] > left_values[None, :])
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

    def summary_array(mode: str, key: str) -> np.ndarray:
        return np.asarray(
            [
                next(
                    row
                    for row in summary
                    if row["mode"] == mode
                    and int(row["n_segments"]) == n_segments
                )[key]
                for n_segments in ALL_SEGMENTS
            ],
            dtype=float,
        )

    fig, ax = plt.subplots(figsize=(9.2, 5.0))
    fig.subplots_adjust(left=0.12, bottom=0.14, right=0.72, top=0.90)
    random_mean = summary_array(MODE_RANDOM, "mean_forward_displacement_m")
    random_ci = summary_array(MODE_RANDOM, "ci95_forward_displacement_m")
    ax.errorbar(
        ALL_SEGMENTS,
        random_mean,
        yerr=random_ci,
        color=MODE_COLORS[MODE_RANDOM],
        marker="o",
        linewidth=2.0,
        capsize=4,
        label="IID pitch mean (95% CI)",
    )
    for mode, marker in ((MODE_CANONICAL, "s"), (MODE_FLIPPED, "D")):
        ax.plot(
            ALL_SEGMENTS,
            summary_array(mode, "mean_forward_displacement_m"),
            color=MODE_COLORS[mode],
            marker=marker,
            linewidth=2.0,
            label=MODE_LABELS[mode],
        )
    ax.axhline(0.0, color="#777777", linewidth=0.9)
    ax.set_xticks(ALL_SEGMENTS)
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("Forward COM displacement (m)")
    ax.set_title(f"Body wave only: pitch-pattern comparison | {_phase_label()}")
    ax.grid(alpha=0.23)
    ax.legend(
        frameon=False,
        fontsize=9,
        loc="center left",
        bbox_to_anchor=(1.01, 0.5),
    )
    fig.savefig(figure_dir / "displacement_vs_segments_three_conditions.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.3, 4.8))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    ax.errorbar(
        ALL_SEGMENTS,
        random_mean,
        yerr=random_ci,
        color=MODE_COLORS[MODE_RANDOM],
        marker="o",
        linewidth=2.0,
        capsize=4,
    )
    ax.axhline(0.0, color="#777777", linewidth=0.9)
    ax.set_xticks(ALL_SEGMENTS)
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("Mean forward COM displacement (m)")
    ax.set_title(f"Body-wave-only IID pitch (95% CI) | {_phase_label()}")
    ax.grid(alpha=0.23)
    fig.savefig(figure_dir / "random_mean_displacement_vs_segments.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.3, 4.8))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    ax.plot(
        ALL_SEGMENTS,
        summary_array(MODE_RANDOM, "sd_forward_displacement_m"),
        color="#a0444b",
        marker="o",
        linewidth=2.0,
    )
    ax.set_xticks(ALL_SEGMENTS)
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("SD of forward displacement (m)")
    ax.set_title(f"Body-wave-only IID outcome spread | {_phase_label()}")
    ax.grid(alpha=0.23)
    fig.savefig(figure_dir / "random_displacement_sd_vs_segments.png", dpi=220)
    plt.close(fig)

    grouped = []
    for n_segments in ALL_SEGMENTS:
        grouped.append(
            np.asarray(
                [
                    float(row["forward_displacement_m"])
                    for row in trials
                    if row["mode"] == MODE_RANDOM
                    and int(row["n_segments"]) == n_segments
                    and row["status"] == "PASS"
                ]
            )
        )
    colors = plt.cm.viridis(np.linspace(0.08, 0.90, len(ALL_SEGMENTS)))
    all_values = np.concatenate(grouped)
    span = max(float(np.ptp(all_values)), 1.0e-6)
    edges = np.linspace(
        float(np.min(all_values)) - 0.04 * span,
        float(np.max(all_values)) + 0.04 * span,
        16,
    )
    counts = [np.histogram(values, bins=edges)[0] for values in grouped]
    ymax = 1.12 * max(int(np.max(value)) for value in counts)

    histogram_rows = []
    for n_segments, values, count in zip(ALL_SEGMENTS, grouped, counts):
        for bin_index, bin_count in enumerate(count):
            histogram_rows.append(
                {
                    "n_segments": n_segments,
                    "bin_index": bin_index,
                    "bin_left_m": float(edges[bin_index]),
                    "bin_right_m": float(edges[bin_index + 1]),
                    "trial_count": int(bin_count),
                    "trial_fraction": float(bin_count) / len(values),
                }
            )
    _write_csv(output_dir / "random_histogram_counts.csv", histogram_rows)

    fig, axes = plt.subplots(2, 4, figsize=(13.2, 6.9), sharex=True, sharey=True)
    fig.subplots_adjust(
        left=0.07,
        bottom=0.10,
        right=0.99,
        top=0.90,
        wspace=0.16,
        hspace=0.28,
    )
    for ax, n_segments, values, color in zip(
        axes.flat, ALL_SEGMENTS, grouped, colors
    ):
        ax.hist(
            values,
            bins=edges,
            color=color,
            edgecolor="white",
            linewidth=0.65,
            alpha=0.88,
        )
        ax.axvline(float(np.mean(values)), color="#202020", linewidth=1.5)
        ax.axvline(0.0, color="#777777", linewidth=0.8, linestyle="--")
        ax.set_xlim(edges[0], edges[-1])
        ax.set_ylim(0.0, ymax)
        ax.set_title(f"N = {n_segments}")
        ax.grid(axis="y", alpha=0.20)
    axes.flat[-1].axis("off")
    for ax in axes[-1, :3]:
        ax.set_xlabel("Displacement (m)")
    for ax in axes[:, 0]:
        ax.set_ylabel("Trial count")
    fig.suptitle(
        f"Body-wave-only IID pitch distributions ({len(grouped[0])} trials per N) | "
        f"{_phase_label()}"
    )
    fig.savefig(figure_dir / "random_displacement_histograms.png", dpi=220)
    plt.close(fig)

    ecdf_rows = []
    fig, ax = plt.subplots(figsize=(9.2, 5.0))
    fig.subplots_adjust(left=0.12, bottom=0.14, right=0.72, top=0.90)
    contact_values = []
    for n_segments, values, color in zip(ALL_SEGMENTS, grouped, colors):
        sorted_values = np.sort(values)
        probability = np.arange(1, len(values) + 1) / len(values)
        for rank, (value, cumulative) in enumerate(
            zip(sorted_values, probability), start=1
        ):
            ecdf_rows.append(
                {
                    "n_segments": n_segments,
                    "rank": rank,
                    "forward_displacement_m": float(value),
                    "empirical_cumulative_probability": float(cumulative),
                }
            )
        step_x = np.concatenate(([edges[0]], sorted_values, [edges[-1]]))
        step_y = np.concatenate(([0.0], probability, [1.0]))
        ax.step(
            step_x,
            step_y,
            where="post",
            color=color,
            linewidth=2.0,
            label=f"N={n_segments}",
        )
    _write_csv(output_dir / "random_ecdf_points.csv", ecdf_rows)
    ax.axvline(0.0, color="#777777", linewidth=0.9, linestyle="--")
    ax.set_xlim(edges[0], edges[-1])
    ax.set_ylim(0.0, 1.01)
    ax.set_xlabel("Forward COM displacement, d (m)")
    ax.set_ylabel(r"Empirical cumulative probability, $\widehat{F}_N(d)$")
    ax.set_title(f"Body-wave-only IID displacement ECDF | {_phase_label()}")
    ax.grid(alpha=0.22)
    ax.legend(frameon=False, ncol=2, loc="lower right")
    fig.savefig(figure_dir / "random_displacement_ecdf.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    fig.subplots_adjust(left=0.14, bottom=0.14, right=0.98, top=0.90)
    for mode, marker in (
        (MODE_CANONICAL, "s"),
        (MODE_FLIPPED, "D"),
        (MODE_RANDOM, "o"),
    ):
        mode_contact = summary_array(mode, "mean_actual_contact_fraction")
        contact_values.extend(mode_contact.tolist())
        ax.plot(
            ALL_SEGMENTS,
            mode_contact,
            color=MODE_COLORS[mode],
            marker=marker,
            linewidth=2.0,
            label=MODE_LABELS[mode],
        )
    ax.set_xticks(ALL_SEGMENTS)
    ax.set_xlabel("Number of segments, N")
    ax.set_ylabel("Mean measured leg-contact fraction")
    contact_span = max(max(contact_values) - min(contact_values), 0.01)
    ax.set_ylim(
        max(0.0, min(contact_values) - 0.35 * contact_span),
        min(1.0, max(contact_values) + 0.35 * contact_span),
    )
    ax.set_title(f"Measured contact is a dynamical outcome | {_phase_label()}")
    ax.grid(alpha=0.23)
    ax.legend(
        frameon=False,
        fontsize=9,
        loc="center left",
        bbox_to_anchor=(1.01, 0.5),
    )
    fig.savefig(figure_dir / "measured_contact_fraction_vs_segments.png", dpi=220)
    plt.close(fig)


def _representative_random_seeds(trials: list[dict]) -> dict[int, int]:
    selected = {}
    for n_segments in ALL_SEGMENTS:
        rows = [
            row
            for row in trials
            if row["mode"] == MODE_RANDOM
            and int(row["n_segments"]) == n_segments
            and row["status"] == "PASS"
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


def _prepare_for_demo(
    repo_root: Path,
    *,
    n_segments: int,
    mode: str,
    seed: int,
    warmup_cycles: int,
):
    return _call_with_controller_factory(
        base._prepare_simulation,
        repo_root,
        n_segments=n_segments,
        seed=seed,
        mode=_internal_mode(mode),
        warmup_cycles=warmup_cycles,
    )


def _render_comparison_demo(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    random_seed: int,
    warmup_cycles: int,
    duration_s: float,
    fps: int,
) -> dict:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    modes = (MODE_CANONICAL, MODE_FLIPPED, MODE_RANDOM)
    prepared_list = [
        _prepare_for_demo(
            repo_root,
            n_segments=n_segments,
            mode=mode,
            seed=random_seed,
            warmup_cycles=warmup_cycles,
        )
        for mode in modes
    ]
    dt = prepared_list[0].dt
    if not all(math.isclose(prepared.dt, dt) for prepared in prepared_list):
        raise RuntimeError("comparison models do not share one timestep")

    render_width, render_height = 520, 360
    renderers = []
    cameras = []
    options = []
    starts = []
    for prepared in prepared_list:
        prepared.model.vis.global_.offwidth = max(
            prepared.model.vis.global_.offwidth, render_width
        )
        prepared.model.vis.global_.offheight = max(
            prepared.model.vis.global_.offheight, render_height
        )
        renderers.append(
            mujoco.Renderer(
                prepared.model, height=render_height, width=render_width
            )
        )
        camera, option = base._camera_for(
            prepared.model, prepared.root_body_id, n_segments, mujoco
        )
        cameras.append(camera)
        options.append(option)
        starts.append(
            np.asarray(prepared.data.subtree_com[prepared.root_body_id]).copy()
        )

    demo_dir = output_dir / "demos"
    demo_dir.mkdir(parents=True, exist_ok=True)
    video_path = demo_dir / f"N{n_segments}_three_condition_comparison.mp4"
    preview_path = demo_dir / f"N{n_segments}_three_condition_comparison_preview.png"
    writer = imageio.get_writer(
        str(video_path),
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=2,
    )
    steps = int(math.ceil(duration_s / dt))
    next_frame_time = 0.0
    frame_count = 0
    preview_saved = False
    try:
        for step in range(steps):
            evaluation_time = step * dt
            for mode, prepared in zip(modes, prepared_list):
                body_time_s = prepared.warmup_time_s + evaluation_time
                prepared.data.ctrl[:] = prepared.controller.control(
                    body_time_s, evaluation_time
                )
                mujoco.mj_step(prepared.model, prepared.data)
            time_end = (step + 1) * dt
            if time_end + 0.5 * dt < next_frame_time:
                continue

            panels = []
            for mode, prepared, renderer, camera, option, start in zip(
                modes,
                prepared_list,
                renderers,
                cameras,
                options,
                starts,
            ):
                body_time_s = prepared.warmup_time_s + evaluation_time
                if mode == MODE_RANDOM:
                    stance = prepared.controller.stance_flags(evaluation_time)
                else:
                    stance = prepared.controller.stance_flags(body_time_s)
                contact = _actual_contact_flags(
                    prepared.model, prepared.data, prepared.body_map
                )
                _set_leg_colors(prepared.model, prepared.body_map, stance)
                renderer.update_scene(
                    prepared.data, camera=camera, scene_option=option
                )
                image = Image.fromarray(renderer.render().copy())
                panel = Image.new(
                    "RGB", (render_width, render_height + 58), "#f3f4f6"
                )
                panel.paste(image, (0, 34))
                draw = ImageDraw.Draw(panel)
                short_label = {
                    MODE_CANONICAL: "Canonical pitch | yaw locked",
                    MODE_FLIPPED: "L/R-flipped pitch | yaw locked",
                    MODE_RANDOM: f"IID pitch | yaw locked | seed {random_seed}",
                }[mode]
                draw.text(
                    (12, 5), short_label, font=_font(18, bold=True), fill="#17202a"
                )
                displacement = float(
                    prepared.data.subtree_com[prepared.root_body_id][1] - start[1]
                )
                draw.text(
                    (12, render_height + 38),
                    (
                        f"D={displacement:+.3f} m   "
                        f"down/contact={int(np.sum(stance))}/{int(np.sum(contact))}"
                    ),
                    font=_font(15),
                    fill="#273746",
                )
                panels.append(panel)

            canvas = Image.new(
                "RGB",
                (render_width * len(panels), panels[0].height + 52),
                "#f3f4f6",
            )
            draw = ImageDraw.Draw(canvas)
            draw.text(
                (18, 10),
                f"Body wave only | N={n_segments} | leg yaw locked | {_phase_label()}",
                font=_font(23, bold=True),
                fill="#17202a",
            )
            draw.text(
                (canvas.width - 345, 14),
                "green: down command   red: up command",
                font=_font(15),
                fill="#273746",
            )
            for panel_index, panel in enumerate(panels):
                canvas.paste(panel, (panel_index * render_width, 52))
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
        "n_segments": n_segments,
        "random_seed": random_seed,
        "duration_s": duration_s,
        "fps": fps,
        "frame_count": frame_count,
        "video": str(video_path),
        "preview": str(preview_path),
    }


def _render_pitch_down_vs_up_demo(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    warmup_cycles: int,
    duration_s: float,
    fps: int,
) -> dict:
    """Render commanded pitch targets while coloring measured ground contact."""
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    modes = (
        "body_wave_pitch_locked_down",
        "body_wave_pitch_locked_up",
    )
    original_phi_body = base.PHI_BODY
    base.PHI_BODY = ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
    prepared_list = []
    renderers = []
    writer = None
    try:
        prepared_list = [
            base._prepare_simulation(
                repo_root,
                n_segments=n_segments,
                seed=0,
                mode=mode,
                warmup_cycles=warmup_cycles,
            )
            for mode in modes
        ]
        dt = prepared_list[0].dt
        render_width, render_height = 640, 400
        cameras = []
        options = []
        starts = []
        for prepared in prepared_list:
            prepared.model.vis.global_.offwidth = max(
                prepared.model.vis.global_.offwidth, render_width
            )
            prepared.model.vis.global_.offheight = max(
                prepared.model.vis.global_.offheight, render_height
            )
            renderers.append(
                mujoco.Renderer(
                    prepared.model, height=render_height, width=render_width
                )
            )
            camera, option = base._camera_for(
                prepared.model, prepared.root_body_id, n_segments, mujoco
            )
            cameras.append(camera)
            options.append(option)
            starts.append(
                np.asarray(prepared.data.subtree_com[prepared.root_body_id]).copy()
            )

        demo_dir = output_dir / "demos" / "pitch_contact_audit"
        demo_dir.mkdir(parents=True, exist_ok=True)
        video_path = demo_dir / f"N{n_segments}_pitch_down_vs_up_actual_contact.mp4"
        preview_path = (
            demo_dir / f"N{n_segments}_pitch_down_vs_up_actual_contact_preview.png"
        )
        writer = imageio.get_writer(
            str(video_path),
            fps=fps,
            codec="libx264",
            quality=8,
            pixelformat="yuv420p",
            macro_block_size=2,
        )
        steps = int(math.ceil(duration_s / dt))
        next_frame_time = 0.0
        frame_count = 0
        preview_saved = False
        contact_sample_counts = [0, 0]
        contact_sample_totals = [0, 0]
        final_displacements = [0.0, 0.0]
        for step in range(steps):
            evaluation_time = step * dt
            for prepared in prepared_list:
                body_time_s = prepared.warmup_time_s + evaluation_time
                prepared.data.ctrl[:] = prepared.controller.control(
                    body_time_s, evaluation_time
                )
                mujoco.mj_step(prepared.model, prepared.data)
            time_end = (step + 1) * dt
            if time_end + 0.5 * dt < next_frame_time:
                continue

            panels = []
            for index, (mode, prepared, renderer, camera, option, start) in enumerate(
                zip(modes, prepared_list, renderers, cameras, options, starts)
            ):
                actual_contact = _actual_contact_flags(
                    prepared.model, prepared.data, prepared.body_map
                )
                contact_sample_counts[index] += int(np.sum(actual_contact))
                contact_sample_totals[index] += int(actual_contact.size)
                _set_leg_colors(
                    prepared.model, prepared.body_map, actual_contact
                )
                renderer.update_scene(
                    prepared.data, camera=camera, scene_option=option
                )
                image = Image.fromarray(renderer.render().copy())
                panel = Image.new(
                    "RGB", (render_width, render_height + 72), "#f3f4f6"
                )
                panel.paste(image, (0, 38))
                draw = ImageDraw.Draw(panel)
                pitch_target = LEG_DOWN_DEG if mode.endswith("down") else LEG_UP_DEG
                label = "ALL PITCH-DOWN" if mode.endswith("down") else "ALL PITCH-UP"
                draw.text(
                    (12, 7),
                    f"{label} ({pitch_target:+.0f} deg command)",
                    font=_font(19, bold=True),
                    fill="#17202a",
                )
                displacement = float(
                    prepared.data.subtree_com[prepared.root_body_id][1] - start[1]
                )
                final_displacements[index] = displacement
                draw.text(
                    (12, render_height + 43),
                    (
                        f"D_y={displacement:+.3f} m   "
                        f"measured contacts={int(np.sum(actual_contact))}/{actual_contact.size}"
                    ),
                    font=_font(16),
                    fill="#273746",
                )
                panels.append(panel)

            header = 62
            canvas = Image.new(
                "RGB", (2 * render_width, header + panels[0].height), "#e9ecef"
            )
            draw = ImageDraw.Draw(canvas)
            draw.text(
                (16, 8),
                f"Body wave only | N={n_segments} | leg yaw locked | t={time_end:5.2f} s",
                font=_font(22, bold=True),
                fill="#17202a",
            )
            draw.text(
                (850, 12),
                "green: measured contact   red: no contact",
                font=_font(15),
                fill="#273746",
            )
            canvas.paste(panels[0], (0, header))
            canvas.paste(panels[1], (render_width, header))
            writer.append_data(np.asarray(canvas))
            frame_count += 1
            if not preview_saved and time_end >= 0.5 * duration_s:
                canvas.save(preview_path)
                preview_saved = True
            next_frame_time += 1.0 / fps
    finally:
        if writer is not None:
            writer.close()
        for renderer in renderers:
            renderer.close()
        base.PHI_BODY = original_phi_body

    return {
        "family": "pitch_down_vs_up_actual_contact_audit",
        "n_segments": n_segments,
        "duration_s": duration_s,
        "fps": fps,
        "frame_count": frame_count,
        "pitch_down_target_deg": LEG_DOWN_DEG,
        "pitch_up_target_deg": LEG_UP_DEG,
        "pitch_down_measured_contact_fraction": (
            contact_sample_counts[0] / contact_sample_totals[0]
        ),
        "pitch_up_measured_contact_fraction": (
            contact_sample_counts[1] / contact_sample_totals[1]
        ),
        "pitch_down_forward_displacement_m": final_displacements[0],
        "pitch_up_forward_displacement_m": final_displacements[1],
        "video": str(video_path),
        "preview": str(preview_path),
    }


def _write_readme(
    output_dir: Path,
    summary: list[dict],
    baseline_comparison: list[dict],
    demo_manifest: list[dict],
) -> None:
    lines = [
        "# Body-wave-only pitch/contact-opportunity experiment",
        "",
        "The canonical lateral body travelling wave remains enabled at +/-15 deg. ",
        "All Shoulder*_R/L leg-yaw commands are locked at their neutral offsets ",
        "(-5 deg right, +5 deg left). No model parameter is changed.",
        "",
        f"Body spatial phase increment: {ACTIVE_SPATIAL_PHASE_INCREMENT_RAD:+.8f} rad "
        f"({math.degrees(ACTIVE_SPATIAL_PHASE_INCREMENT_RAD):+.1f} deg), corresponding "
        f"to {_spatial_wavelength_increments():.1f} inter-segment phase intervals per "
        "full 2-pi spatial wave.",
        "",
        "## Conditions",
        "",
        "1. `body_wave_canonical_pitch_yaw_locked`: canonical alternating R/L pitch pattern.",
        "2. `body_wave_lr_flipped_pitch_yaw_locked`: R/L pitch commands exchanged while the body wave is unchanged.",
        "3. `body_wave_fixed_window_iid_pitch_yaw_locked`: every physical leg draws down/up independently at each T_g/2 window.",
        "",
        "The experiment randomizes pitch commands/contact opportunities, not physical ",
        "ground contact. Measured contact is recorded separately.",
        "",
        "## Locked stochastic definition",
        "",
        "For segment i, side s, and decision window k,",
        "",
        "    X_(i,s,k) ~ IID Bernoulli(0.5).",
        "",
        f"Cross-N random-stream mode: `{ACTIVE_RNG_CROSS_N_MODE}`. ",
        (
            "The strict mode includes N in the random-stream address, so an N=4 "
            "trajectory is not the prefix of an N=10 trajectory."
            if ACTIVE_RNG_CROSS_N_MODE == "strict_independent"
            else "The legacy mode preserves the historical nested cross-N streams."
        ),
        "",
        f"The random condition uses {DEFAULT_REPLAYS} trials per N in the default run. ",
        "Every trial has four warmup gait cycles and twelve evaluated gait cycles.",
        "",
        "## Summary",
        "",
        "| mode | N | pass/trials | mean D (m) | SD (m) | positive fraction |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            f"| {row['mode']} | {row['n_segments']} | "
            f"{row['n_pass']}/{row['n_trials']} | "
            f"{row['mean_forward_displacement_m']:.4f} | "
            f"{row['sd_forward_displacement_m']:.4f} | "
            f"{row['positive_fraction']:.3f} |"
        )
    lines += [
        "",
        "## Baseline flip comparison",
        "",
        "| N | canonical D (m) | flipped D (m) | flipped - canonical (m) |",
        "|---:|---:|---:|---:|",
    ]
    for row in baseline_comparison:
        lines.append(
            f"| {row['n_segments']} | "
            f"{row['canonical_forward_displacement_m']:.4f} | "
            f"{row['flipped_forward_displacement_m']:.4f} | "
            f"{row['flipped_minus_canonical_m']:.4f} |"
        )
    lines += [
        "",
        "## Evidence",
        "",
        "- `trials.csv` / `trials.json`: all scalar results and stability gates.",
        "- `summary.csv`: mode-by-N aggregates.",
        "- `baseline_flip_comparison.csv`: deterministic original-versus-flipped results.",
        "- `random_vs_baselines.csv`: empirical random-trial exceedance fractions.",
        (
            "- `random_independent_segment_comparisons.csv`: independent-sample "
            "adjacent-N and N10-N4 differences."
            if ACTIVE_RNG_CROSS_N_MODE == "strict_independent"
            else "- `random_paired_segment_comparisons.csv`: same-seed adjacent-N "
            "and N10-N4 differences."
        ),
        "- `random_histogram_counts.csv` and `random_ecdf_points.csv`: plotted distribution data.",
        "- `semantic_validation.json`: exact yaw-lock, body-wave, flip, and IID-command checks.",
        "- `validation_report.json`: trial counts and independent stability-gate totals.",
        "- `trajectories/`: sampled COM, commanded pitch state, and measured contact.",
        "- `figures/`: absolute displacement, spread, ECDF, and measured-contact plots.",
        "- `demos/`: synchronized canonical/flipped/IID videos.",
        "",
        "## Demos",
        "",
    ]
    lines.extend(f"- `{Path(row['video']).name}`" for row in demo_manifest)
    (output_dir / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=_repo_root_from_script())
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--segments", type=int, nargs="+", default=list(ALL_SEGMENTS))
    parser.add_argument("--replays", type=int, default=DEFAULT_REPLAYS)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--warmup-cycles", type=int, default=DEFAULT_WARMUP_CYCLES)
    parser.add_argument(
        "--evaluation-cycles", type=int, default=DEFAULT_EVALUATION_CYCLES
    )
    parser.add_argument(
        "--rng-cross-n-mode",
        choices=("legacy_nested", "strict_independent"),
        default="legacy_nested",
        help=(
            "legacy_nested reproduces Step-18/19; strict_independent includes N in "
            "the deterministic random-stream address"
        ),
    )
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument(
        "--spatial-phase-increment-deg",
        type=float,
        default=math.degrees(PHI_BODY),
        help=(
            "signed body-wave phase increment between adjacent segments in degrees; "
            "the default preserves the historical Step-18 value"
        ),
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--no-demos", action="store_true")
    parser.add_argument("--demo-segments", type=int, nargs="+", default=[4, 7, 10])
    parser.add_argument("--demo-duration", type=float, default=8.0)
    parser.add_argument("--demo-fps", type=int, default=30)
    return parser


def run_experiment(args: argparse.Namespace) -> None:
    global ACTIVE_SPATIAL_PHASE_INCREMENT_RAD, ACTIVE_RNG_CROSS_N_MODE
    ACTIVE_SPATIAL_PHASE_INCREMENT_RAD = math.radians(
        float(args.spatial_phase_increment_deg)
    )
    if not np.isfinite(ACTIVE_SPATIAL_PHASE_INCREMENT_RAD):
        raise ValueError("spatial phase increment must be finite")
    if abs(ACTIVE_SPATIAL_PHASE_INCREMENT_RAD) <= 1.0e-12:
        raise ValueError("spatial phase increment must be nonzero")
    ACTIVE_RNG_CROSS_N_MODE = str(args.rng_cross_n_mode)

    repo_root = args.repo_root.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else repo_root
        / "exp_data"
        / "reliability"
        / "step18_body_wave_only_random_pitch"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    segments = tuple(sorted(set(int(value) for value in args.segments)))
    if segments != ALL_SEGMENTS and not set(segments).issubset(ALL_SEGMENTS):
        raise ValueError(f"segments must be drawn from {ALL_SEGMENTS}")
    if args.replays <= 0:
        raise ValueError("replays must be positive")

    semantic_validation = _semantic_validation(repo_root)
    _write_json(output_dir / "semantic_validation.json", semantic_validation)
    if not semantic_validation["all_pass"]:
        raise RuntimeError(f"controller semantic validation failed: {semantic_validation}")

    trials = []
    for mode in (MODE_CANONICAL, MODE_FLIPPED):
        for n_segments in segments:
            result = _run_or_load_trial(
                repo_root,
                output_dir,
                mode=mode,
                n_segments=n_segments,
                seed=0,
                warmup_cycles=args.warmup_cycles,
                evaluation_cycles=args.evaluation_cycles,
                sample_hz=args.sample_hz,
                force=args.force,
            )
            trials.append(result)
            print(
                f"{mode} N={n_segments}: {result['status']} "
                f"D={result['forward_displacement_m']:+.4f} m",
                flush=True,
            )

    for n_segments in segments:
        for seed in range(args.seed_start, args.seed_start + args.replays):
            result = _run_or_load_trial(
                repo_root,
                output_dir,
                mode=MODE_RANDOM,
                n_segments=n_segments,
                seed=seed,
                warmup_cycles=args.warmup_cycles,
                evaluation_cycles=args.evaluation_cycles,
                sample_hz=args.sample_hz,
                force=args.force,
            )
            trials.append(result)
            if (seed - args.seed_start + 1) % 16 == 0 or seed == args.seed_start:
                print(
                    f"{MODE_RANDOM} N={n_segments}: "
                    f"{seed - args.seed_start + 1}/{args.replays} complete",
                    flush=True,
                )

    summary = _aggregate(trials)
    _write_csv(output_dir / "trials.csv", base._flatten_trials(trials))
    _write_json(output_dir / "trials.json", trials)
    _write_csv(output_dir / "summary.csv", summary)
    baseline_comparison, random_vs_baselines = _comparison_tables(trials, summary)
    _write_csv(output_dir / "baseline_flip_comparison.csv", baseline_comparison)
    _write_csv(output_dir / "random_vs_baselines.csv", random_vs_baselines)
    if segments == ALL_SEGMENTS and ACTIVE_RNG_CROSS_N_MODE == "strict_independent":
        independent_comparisons = _independent_random_comparisons(trials)
        _write_csv(
            output_dir / "random_independent_segment_comparisons.csv",
            independent_comparisons,
        )
    elif segments == ALL_SEGMENTS:
        paired_comparisons = _paired_random_comparisons(trials)
        _write_csv(
            output_dir / "random_paired_segment_comparisons.csv", paired_comparisons
        )
    validation_report = {
        "trial_count": len(trials),
        "pass_count": sum(row["status"] == "PASS" for row in trials),
        "warning_row_count": sum(
            int(row["mujoco_warning_total"]) > 0 for row in trials
        ),
        "inverted_row_count": sum(
            float(row["minimum_root_up_z"]) <= 0.0 for row in trials
        ),
        "nonfinite_failure_count": sum(
            not bool(row["gates"]["all_states_finite"]) for row in trials
        ),
        "window_state_failure_count": sum(
            not bool(row["gates"]["window_state_constant"]) for row in trials
        ),
        "random_trials_per_N": {
            str(n_segments): sum(
                row["mode"] == MODE_RANDOM
                and int(row["n_segments"]) == n_segments
                for row in trials
            )
            for n_segments in segments
        },
        "semantic_validation_all_pass": semantic_validation["all_pass"],
        "rng_cross_n_mode": ACTIVE_RNG_CROSS_N_MODE,
    }
    validation_report["all_trials_pass"] = (
        validation_report["trial_count"] == validation_report["pass_count"]
    )
    validation_report["all_audit_checks_pass"] = all(
        (
            validation_report["all_trials_pass"],
            validation_report["warning_row_count"] == 0,
            validation_report["inverted_row_count"] == 0,
            validation_report["nonfinite_failure_count"] == 0,
            validation_report["window_state_failure_count"] == 0,
            validation_report["semantic_validation_all_pass"],
            all(
                count == args.replays
                for count in validation_report["random_trials_per_N"].values()
            ),
        )
    )
    _write_json(output_dir / "validation_report.json", validation_report)
    _write_json(
        output_dir / "run_configuration.json",
        {
            "schema_version": 1,
            "segments": list(segments),
            "random_trials_per_N": args.replays,
            "deterministic_trials_per_N_per_pattern": 1,
            "warmup_cycles": args.warmup_cycles,
            "evaluation_cycles": args.evaluation_cycles,
            "sample_hz": args.sample_hz,
            "body_wave": {
                "enabled": True,
                "yaw_half_amplitude_deg": YAW_HALF_AMPLITUDE_DEG,
                "omega_rad_s": OMEGA_BODY,
                "spatial_phase_increment_rad": ACTIVE_SPATIAL_PHASE_INCREMENT_RAD,
                "spatial_phase_increment_deg": math.degrees(
                    ACTIVE_SPATIAL_PHASE_INCREMENT_RAD
                ),
                "spatial_wavelength_increments": _spatial_wavelength_increments(),
            },
            "leg_yaw": {
                "locked": True,
                "right_target_deg": LEG_OFFSET_DEG,
                "left_target_deg": -LEG_OFFSET_DEG,
            },
            "leg_pitch": {
                "up_target_deg": LEG_UP_DEG,
                "down_target_deg": LEG_DOWN_DEG,
                "random_stance_probability": 0.5,
                "random_decision_window_s": math.pi / OMEGA_BODY,
                "random_independent_across_segments_sides_windows": True,
                "random_cross_n_mode": ACTIVE_RNG_CROSS_N_MODE,
                "random_entropy_fields": (
                    ["seed", "N", "window", "segment", "side", "namespace"]
                    if ACTIVE_RNG_CROSS_N_MODE == "strict_independent"
                    else ["seed", "window", "segment", "side", "namespace"]
                ),
            },
            "flipped_definition": (
                "exchange canonical right/left leg-pitch commands while leaving "
                "the body wave unchanged"
            ),
            "primary_metric": "signed whole-robot COM displacement along +y (m)",
        },
    )

    if segments == ALL_SEGMENTS:
        _plot_results(output_dir, trials, summary)

    demo_manifest_path = output_dir / "demo_manifest.json"
    demo_manifest = (
        json.loads(demo_manifest_path.read_text(encoding="utf-8"))
        if args.no_demos and demo_manifest_path.is_file()
        else []
    )
    if not args.no_demos and segments == ALL_SEGMENTS:
        representatives = _representative_random_seeds(trials)
        for n_segments in args.demo_segments:
            if n_segments not in representatives:
                continue
            demo_manifest.append(
                _render_comparison_demo(
                    repo_root,
                    output_dir,
                    n_segments=n_segments,
                    random_seed=representatives[n_segments],
                    warmup_cycles=args.warmup_cycles,
                    duration_s=args.demo_duration,
                    fps=args.demo_fps,
                )
            )
            demo_manifest.append(
                _render_pitch_down_vs_up_demo(
                    repo_root,
                    output_dir,
                    n_segments=n_segments,
                    warmup_cycles=args.warmup_cycles,
                    duration_s=args.demo_duration,
                    fps=args.demo_fps,
                )
            )
        _write_csv(output_dir / "demo_manifest.csv", demo_manifest)
        _write_json(output_dir / "demo_manifest.json", demo_manifest)

    if segments == ALL_SEGMENTS:
        _write_readme(output_dir, summary, baseline_comparison, demo_manifest)


def main() -> None:
    run_experiment(build_parser().parse_args())


if __name__ == "__main__":
    main()
