# Original-contact body-amplitude optimization (Step 35)

Only body-yaw amplitude varies. Physical N=10 center-rooted STL model, original cave-grouped binary pitch, fixed leg-yaw targets. No IID, no erasure, no model/armature/gain/solver changes.

Objective: D = COM_y(t_end) - COM_y(t_start), in metres along fixed world +y. Not path length, not absolute value of signed displacement, not normalized performance. Each primary trial has 1.5 s settling, 0.4 s cosine bridge, 4 warm-up cycles, then 12 evaluation cycles (18.850 s after timestep rounding). Warm-up displacement is excluded.

Search: N_wave = 0.5, 1, ..., 4; commanded body-joint half-amplitude A = 0..40 deg, 2.5-deg coarse grid, then 0.5-deg refinement within +/-2.5 deg of each coarse winner. Existing actuator bounds are +/-0.7 rad; these are not relaxed. A denotes joint target half-amplitude, not lateral body excursion or measured achieved joint angle.

Primary ranking uses the common phase-zero initialization. Selected winners and 15-deg baselines are separately checked at 90/180/270-degree starting phase, and for 24 evaluation cycles. These are deterministic sensitivity checks, not independent noise trials or confidence intervals. Numerical gates and tracking diagnostics are separate from displacement.

| N_wave | Best A (deg) | D (m) | D at 15 deg (m) | Gain (%) |
|---|---|---|---|---|
| 0.5 | 40 | 2.2696 | 0.7826 | 190.0 |
| 1 | 40 | 7.4507 | 3.2093 | 132.2 |
| 1.5 | 40 | 6.3194 | 2.6939 | 134.6 |
| 2 | 40 | 5.0863 | 2.1173 | 140.2 |
| 2.5 | 40 | 4.5920 | 1.7353 | 164.6 |
| 3 | 40 | 4.0783 | 1.5777 | 158.5 |
| 3.5 | 40 | 3.6702 | 1.3474 | 172.4 |
| 4 | 40 | 3.4404 | 1.2747 | 169.9 |

## Limits of the claim

- Best means the best feasible point sampled under this protocol; not a proof of a continuous global optimum. A winner at 40 deg is boundary-limited; a larger unconstrained optimum has not been ruled out.
- Existing model enables explicit robot-ground contact pairs but not robot self-collision. Large-amplitude winners require hardware-clearance validation before real-robot use; no claim of self-collision safety is made.
- Actuator torque clipping and tracking lag remain part of the unchanged model. Target amplitude is not achieved amplitude.
- The original 15-degree baseline and previous datasets remain unchanged. No optimizer winner is silently promoted into shared controller defaults.
- All upright checks inspect all ten chassis at 20 Hz; this is not a timestep/solver convergence proof.

Files: trials.csv/json; optimal_amplitudes.csv; results.json; controller_validation.json; figures/; demos/ (6-second demonstrations, not the full 18.850-second objective).
