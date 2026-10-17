# Step 33: physical robot IID contact-command wave sweep

The center-rooted measured-STL robot has 10 segments. Leg-yaw joints
are locked at their neutral offsets. The body-yaw wave uses
`Delta_phi=-2*pi*N_wave/(N_segments-1)`. Every half gait cycle, each of the 20
leg-pitch joints independently draws down/stance-opportunity or up/swing with
probability 0.5. This randomizes commanded contact opportunity; measured ground
contact remains a dynamical output.

Each wave condition contains 128 trials. Each trial has
4 warm-up cycles and 12 evaluated cycles
(18.850 s). Random streams are independent
across wave number, trial, time window, segment, and side.

Files:
- `iid_trials.csv`: one row per simulation.
- `iid_summary.csv`: absolute-unit ensemble statistics and bootstrap intervals.
- `figures/`: signed/absolute displacement and measured-contact plots.
- `demos/`: one median-displacement representative video per wave number.
- `semantic_validation.json` and `validation.json`: controller and run audits.
- `rollout_control_validation.json`: physical actuator-name mapping, model hash,
  and vectorized-versus-scalar command checks.
- `artifact_validation.json`: final table/media integrity checks.
- `METHODS.md`: precise definitions, timing, diagnostics and interpretation limits.

## Completed results — 2026-09-01

All 1,024 trials passed the implemented numerical gates; no MuJoCo warnings
were recorded. There are 128 unique trial seeds in every wave condition.
The physics model was not modified for this sweep.

| Spatial wave number | Mean signed D (m) | Pointwise 95% bootstrap CI (m) | Mean absolute D (m) |
|---:|---:|---:|---:|
| 0.5 | 0.0594 | [0.0497, 0.0691] | 0.0668 |
| 1.0 | 0.1864 | [0.1715, 0.2019] | 0.1891 |
| 1.5 | 0.2684 | [0.2469, 0.2902] | 0.2705 |
| 2.0 | 0.1379 | [0.1199, 0.1556] | 0.1453 |
| 2.5 | 0.0658 | [0.0500, 0.0824] | 0.0933 |
| 3.0 | 0.0348 | [0.0223, 0.0470] | 0.0650 |
| 3.5 | 0.0104 | [-0.0015, 0.0223] | 0.0555 |
| 4.0 | -0.0155 | [-0.0253, -0.0053] | 0.0485 |

The response is non-monotonic. Among the tested wave numbers, 1.5 has the
largest sample mean forward displacement. The 3.5 interval includes zero;
4.0 has a small negative sample mean. These observations are not a proof of
a redundancy scaling law or an optimum between the sampled wave numbers.

The eight videos are six-second excerpts from median-displacement trial seeds.
Green means commanded pitch down, red means commanded pitch up; colors do not
indicate an externally erased contact channel. The full-trial displacement
statistics above use 18.850 seconds of evaluation, excluding warm-up.
Leg yaw is held at constant servo targets, not mechanically welded.

## Tracking limitation observed in representative demos

In the eight six-second demo replays, maximum leg-yaw target error ranges
from 1.36 to 3.14 degrees; RMS leg-pitch target error ranges from 11.18 to
14.69 degrees; RMS body-yaw target error ranges from 2.96 to 10.12 degrees.
In particular, body-yaw RMS error is about 10 degrees for the 0.5 and 1.0
examples. RMS target error includes transient/phase lag and must not be
interpreted as an amplitude difference alone.

These are representative-demo diagnostics, not estimates over all 1,024 trials.
The results therefore include wave-dependent mechanical tracking effects under
the unchanged servo parameters. Do not attribute the displacement curve solely
to redundancy or assume commanded pitch states are instantaneous ground-contact
states. Quantifying/controlling tracking across conditions is a necessary next
check before such a causal claim. No gains, inertia, armature or geometry were
retuned to remove this limitation.
