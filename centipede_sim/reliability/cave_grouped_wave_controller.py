"""Cave-collocated body/leg controller for the physical centipede robot.

The body-yaw actuators live between adjacent segment centers.  A body-joint
phase therefore cannot also be used as the phase of the upstream leg pair.
This module places body-yaw samples at joint midpoints and leg commands at
segment centers.  Binary pitch is assigned by the parity of the continuous
body-curvature half-wave (the ``cave``), so every segment in one cave receives
the same right/left stance choice.
"""

from __future__ import annotations

import math

import numpy as np

import run_center_rooted_dynamic_null_wave_flip as legacy
from run_independent_random_leg_phases import (
    LEG_DOWN_DEG,
    LEG_OFFSET_DEG,
    LEG_UP_DEG,
    OMEGA_BODY,
    SHOULDER_HALF_AMPLITUDE_DEG,
)


CAVE_BOUNDARY_TOL = 1.0e-12


def half_wave_index(phase_rad: float) -> int:
    """Return the half-open body-curvature half-wave containing ``phase_rad``.

    Cave ``c`` is the interval ``[c*pi, (c+1)*pi)``.  The tiny positive bias
    makes exact zero crossings deterministic without moving ordinary samples.
    """

    return math.floor((float(phase_rad) + CAVE_BOUNDARY_TOL) / math.pi)


class CaveGroupedContactPatternController(legacy.CoupledContactPatternController):
    """Couple binary leg pitch to body-curvature caves, not pitch extrema."""

    def segment_phase(self, body_time_s: float, segment_index: int) -> float:
        """Phase at a segment center when body joint 0 is the first midpoint."""

        phase_time_s = self._phase_time(body_time_s)
        return (
            OMEGA_BODY * phase_time_s
            + (float(segment_index) - 0.5) * self.phi_body
        )

    def body_joint_phase(self, body_time_s: float, body_joint_index: int) -> float:
        phase_time_s = self._phase_time(body_time_s)
        return OMEGA_BODY * phase_time_s + float(body_joint_index) * self.phi_body

    def cave_index(self, body_time_s: float, segment_index: int) -> int:
        return half_wave_index(self.segment_phase(body_time_s, segment_index))

    def right_stance(self, body_time_s: float, segment_index: int) -> bool:
        """Use one right/left choice for every segment in the same cave."""

        # Even caves define the canonical right-side-down state.  With the
        # physical model's joint-axis convention this is the clustered/inner
        # contact side and preserves the established forward-labelled gait.
        # The pitch-flipped condition exchanges the two sides below.
        return not bool(self.cave_index(body_time_s, segment_index) % 2)

    def control(self, body_time_s: float, _decision_time_s: float = 0.0) -> np.ndarray:
        ctrl = np.zeros(self.n_ctrl, dtype=float)
        shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = self.indices
        shoulder_amplitude = math.radians(SHOULDER_HALF_AMPLITUDE_DEG)
        leg_offset = math.radians(LEG_OFFSET_DEG)
        leg_up = math.radians(LEG_UP_DEG)
        leg_down = math.radians(LEG_DOWN_DEG)

        # Body-yaw samples are joint-centered.  Joint j lies midway between
        # segment centers j and j+1.
        for body_index, actuator_index in enumerate(link_y):
            ctrl[actuator_index] = legacy.BODY_WAVE_HALF_AMPLITUDE_RAD * math.sin(
                self.body_joint_phase(body_time_s, body_index)
            )
        for actuator_index in link_p:
            ctrl[actuator_index] = 0.0

        for segment_index in range(self.n_segments):
            theta_segment = self.segment_phase(body_time_s, segment_index)

            # The +pi/2 yaw offset is evaluated at the collocated segment
            # center, rather than at the upstream inter-segment joint.
            leg_yaw_wave = math.sin(
                theta_segment + legacy.LEG_YAW_BODY_PHASE_OFFSET_RAD
            )
            ctrl[shoulder_r[segment_index]] = (
                leg_offset + shoulder_amplitude * leg_yaw_wave
            )
            ctrl[shoulder_l[segment_index]] = (
                -leg_offset + shoulder_amplitude * leg_yaw_wave
            )

            right_down = self.right_stance(body_time_s, segment_index)
            ctrl[leg_r[segment_index]] = leg_down if right_down else leg_up
            ctrl[leg_l[segment_index]] = leg_up if right_down else leg_down

        if self.flipped:
            right = ctrl[leg_r].copy()
            left = ctrl[leg_l].copy()
            ctrl[leg_r] = left
            ctrl[leg_l] = right
        return ctrl


def validate_cave_contract(
    n_wave_values: tuple[float, ...] = (1.0, 1.5),
    *,
    n_segments: int = 10,
    time_samples: int = 721,
) -> dict:
    """Algebraically audit collocation and cave-uniform pitch assignment."""

    dummy_indices = (
        np.arange(0, 4 * n_segments, 4),
        np.arange(1, 4 * n_segments, 4),
        np.arange(2, 4 * n_segments, 4),
        np.arange(3, 4 * n_segments, 4),
        np.arange(4 * n_segments, 4 * n_segments + 2 * (n_segments - 1), 2),
        np.arange(4 * n_segments + 1, 4 * n_segments + 2 * (n_segments - 1), 2),
    )
    n_ctrl = 4 * n_segments + 2 * (n_segments - 1)
    period_s = 2.0 * math.pi / OMEGA_BODY
    rows = []
    overall = True
    for n_wave in n_wave_values:
        controller = CaveGroupedContactPatternController(
            n_segments,
            n_ctrl,
            dummy_indices,
            None,
            n_wave=float(n_wave),
            global_phase_deg=0.0,
            flipped=False,
        )
        legacy_controller = legacy.CoupledContactPatternController(
            n_segments,
            n_ctrl,
            dummy_indices,
            None,
            n_wave=float(n_wave),
            global_phase_deg=0.0,
            flipped=False,
        )
        uniform = True
        complementary = True
        midpoint = True
        span = True
        legacy_uniform_cave_groups = 0
        legacy_total_cave_groups = 0
        for body_time_s in np.linspace(0.0, period_s, time_samples, endpoint=False):
            ctrl = controller.control(float(body_time_s))
            stance = controller.stance_flags(float(body_time_s))
            legacy_stance = legacy_controller.stance_flags(float(body_time_s))
            caves = np.asarray(
                [controller.cave_index(float(body_time_s), i) for i in range(n_segments)]
            )
            for cave in np.unique(caves):
                mask = caves == cave
                uniform &= bool(np.all(stance[mask, 0] == stance[mask, 0][0]))
                uniform &= bool(np.all(stance[mask, 1] == stance[mask, 1][0]))
                legacy_uniform_cave_groups += int(
                    np.all(legacy_stance[mask, 0] == legacy_stance[mask, 0][0])
                    and np.all(legacy_stance[mask, 1] == legacy_stance[mask, 1][0])
                )
                legacy_total_cave_groups += 1
            complementary &= bool(np.all(np.logical_xor(stance[:, 0], stance[:, 1])))
            segment_phase = np.asarray(
                [controller.segment_phase(float(body_time_s), i) for i in range(n_segments)]
            )
            joint_phase = np.asarray(
                [controller.body_joint_phase(float(body_time_s), j) for j in range(n_segments - 1)]
            )
            midpoint &= bool(
                np.allclose(joint_phase, 0.5 * (segment_phase[:-1] + segment_phase[1:]))
            )
            span &= bool(
                math.isclose(
                    abs(segment_phase[-1] - segment_phase[0]),
                    2.0 * math.pi * float(n_wave),
                    rel_tol=0.0,
                    abs_tol=1.0e-10,
                )
            )
            del ctrl
        passed = bool(uniform and complementary and midpoint and span)
        overall &= passed
        rows.append(
            {
                "N_wave": float(n_wave),
                "same_cave_has_one_pitch_state": bool(uniform),
                "left_right_are_complementary": bool(complementary),
                "body_joints_are_between_segment_phase_samples": bool(midpoint),
                "head_to_tail_phase_span_is_2pi_N_wave": bool(span),
                "legacy_step31_same_cave_uniform_fraction": float(
                    legacy_uniform_cave_groups / legacy_total_cave_groups
                ),
                "passed": passed,
            }
        )
    return {
        "passed": bool(overall),
        "definition": {
            "body_joint_phase": "omega*t + j*Delta_phi",
            "segment_center_phase": "omega*t + (i-1/2)*Delta_phi",
            "Delta_phi": "-2*pi*N_wave/(N_segments-1)",
            "cave_index": "floor((segment_center_phase+tol)/pi)",
            "pitch_rule": "one complementary L/R binary state per cave parity",
        },
        "rows": rows,
    }
