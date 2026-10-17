# Center-rooted dynamic-null audit and IID random-contact rerun

## Why this rerun was necessary

The earlier primitive null replaced asymmetric meshes and inertias, but it
retained a one-ended kinematic tree. Reversing the tree root reversed the sign
of the deterministic displacement (`+14.2808 mm` versus `-14.2045 mm` in the
N=7 diagnostic). Therefore that model was geometrically symmetric but was not
a valid *dynamic* null for deciding whether the IID contact process has zero
signed mean.

This step constructs a separate family of center-rooted models in which the
front and rear halves are mirror copies and the ground-contact constraints are
also emitted in mirror pairs. The aim is to remove a privileged head-to-tail
assembly and constraint-enumeration direction before rerunning the experiment.

## Model construction

For every mirrored pair of segments, links, legs, inertial elements, and ground
contacts, the model generator enforces the corresponding reflected placement.
All models have whole-robot center of mass coordinate `y=0` in the reference
pose.

- Odd N: the middle physical segment is the free-joint root; equal front and
  rear branches are attached to it.
- Even N: the middle connector is the free-joint root. The central relative yaw
  is represented by two mirrored half joints, constrained to have equal and
  opposite angles. The actuator transmission is scaled so that the commanded
  coordinate remains the full intersegment yaw.
- Intersegment pitch is retained structurally but equality-locked at zero,
  matching the planar body-yaw experiment.
- Every robot-floor collision is represented by an explicit contact pair.
  Mirrored pairs are adjacent in the XML and their first-listed member
  alternates, so the contact list itself does not scan monotonically from one
  end of the robot to the other.
- Original total mass and actuator count are preserved exactly. For even N,
  the original central connector mass is divided symmetrically among the new
  center-root elements.
- Newton contact solve, elliptic friction cone, 200 solver iterations,
  tolerance `1e-12`, and 50 no-slip iterations are used.

The generated models are in `models/center_rooted_dynamic_null/`.

## Structural validation

All N=4,...,10 models compile and pass the mass and actuator-count checks.

| N | Total mass (kg) | Actuators | Explicit contact pairs | Static mirror error (m) |
|---:|---:|---:|---:|---:|
| 4 | 2.0196 | 22 | 44 | 0 |
| 5 | 2.5538 | 28 | 51 | 2.22e-17 |
| 6 | 3.0880 | 34 | 66 | 0 |
| 7 | 3.6222 | 40 | 73 | 0 |
| 8 | 4.1564 | 46 | 88 | 0 |
| 9 | 4.6906 | 52 | 95 | 0 |
| 10 | 5.2248 | 58 | 110 | 5.55e-17 |

The validation data are in `validation/structure.csv` and
`validation/validation_summary.json`.

## Symmetric initialization

The canonical-gait initialization used by the earlier runner was not suitable
for this null test: it injected signed momentum before the evaluation window.
Every trial in this step instead starts straight, with body wave off and every
leg in the same pitch-down state; the controller is then introduced through a
smooth bridge, followed by four unmeasured warm-up cycles.

With body wave disabled, the maximum displacement over 12 evaluated cycles is
`4.90e-7 m` (`0.00049 mm`). With body wave enabled and all legs held down, the
maximum absolute displacement among all tested N and the two opposite initial
body phases is `3.64 mm`. For the directly comparable N=7, phase-zero test, the
new center-rooted model moves only `0.0317 mm`, versus `14.2808 mm` for the old
head-rooted primitive null (about 450 times smaller).

These deterministic results are in
`validation/deterministic_all_down.csv`.

## IID experiment

The retained body motion is the `+/-15 deg` yaw wave with spatial phase offset
`-pi`; leg yaw is locked. Every physical left and right leg independently draws
a Bernoulli(0.5) pitch-down or pitch-up command in each half-cycle decision
window. Random streams are strictly independent across N rather than nested.

- 4 warm-up cycles per trial
- 12 evaluated cycles per trial
- 100 Hz recorded trajectory samples
- 128 ordinary IID trials for N=4,5,7,8,9,10
- 512 ordinary IID trials for N=6, expanded after its first 128 trials showed a
  finite-sample negative mean
- 1,280 ordinary IID trials in total; all passed
- 16 additional bitwise-complement diagnostic pairs per N; these are not used
  to force or compute the ordinary IID means

## IID results

| N | Trials | Mean signed displacement (mm) | SD (mm) | Bootstrap 95% CI (mm) | Positive fraction |
|---:|---:|---:|---:|---:|---:|
| 4 | 128 | +0.358 | 36.794 | [-5.935, +6.968] | 0.492 |
| 5 | 128 | +4.925 | 29.724 | [-0.298, +10.134] | 0.586 |
| 6 | 512 | -2.944 | 36.720 | [-6.063, +0.234] | 0.484 |
| 7 | 128 | +5.382 | 34.226 | [-0.544, +11.354] | 0.578 |
| 8 | 128 | +0.612 | 35.599 | [-5.590, +6.791] | 0.508 |
| 9 | 128 | -1.290 | 35.808 | [-7.496, +4.932] | 0.445 |
| 10 | 128 | +3.726 | 40.330 | [-3.245, +10.557] | 0.484 |

Every 95% confidence interval contains zero. The fitted signed-mean slope is
`+0.0440 mm per added segment`, compared with `+2.263 mm per segment` in the
old one-ended primitive null: a reduction of approximately 98%. The remaining
slope is also tiny compared with the 30--40 mm single-trial standard deviation.

The correct interpretation is therefore:

1. individual IID trials still move appreciably in either direction because a
   finite random contact sequence need not balance;
2. the rebuilt null shows no resolved nonzero ensemble mean at any tested N;
3. there is no evidence that mean signed displacement systematically grows
   with segment number once the head-tail ordering and signed initialization
   are removed.

The N=6 extension is an important check: its initial 128-trial mean looked
negative, but after 512 trials the confidence interval included zero. This is
the expected behavior of finite-sample fluctuation, not a stable direction.

## Complement diagnostic and limitation

Bitwise-complementing all pitch commands is not an exact dynamical sign
inversion: contact has memory, impacts are unilateral, and the complementary
trajectory does not retrace the original state history. Complement pairs are
therefore reported only as a diagnostic and are never pair-averaged into the
main result.

MuJoCo must still store and solve a finite ordered constraint list. The paired,
alternating contact construction removes the obvious front-to-back enumeration
and the experiments show no statistically resolved systematic drift, but this
is empirical evidence rather than a mathematical proof of bitwise
order-independence.

## Main artifacts

- `iid_summary.csv`: numerical ensemble summary
- `validation.json`: run-level checks and trend values
- `trial_records/ordinary/`: all ordinary trial records
- `trial_records/complement/`: complement diagnostics
- `trajectories/`: recorded trajectories
- `figures/iid_mean_signed_displacement_vs_N.png`
- `figures/iid_displacement_histograms.png`
- `figures/iid_positive_fraction_vs_N.png`
- `demos/`: representative N=4,7,10 videos

## Reproduction scripts

- `centipede_sim/reliability/generate_center_rooted_dynamic_null_models.py`
- `centipede_sim/reliability/validate_center_rooted_dynamic_null.py`
- `centipede_sim/reliability/run_center_rooted_dynamic_null_iid.py`
- `centipede_sim/reliability/render_center_rooted_dynamic_null_iid.py`
