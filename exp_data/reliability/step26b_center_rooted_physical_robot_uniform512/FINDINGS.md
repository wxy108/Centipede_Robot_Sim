# Uniform-512 center-rooted physical-robot IID sweep

## Scope

This experiment repeats the production symmetric-null IID sweep on the
measured Centipede robot.  The robot uses the audited center-root dynamics
tree, while retaining the source STL geometry, measured mass and inertia, and
real fore-aft boundary asymmetry.  It is therefore not a symmetric null.

The earlier 32-seed head/center/tail experiment remains the root-location
audit.  It was not expanded to 512 seeds here.  This production sweep samples
only the center-root representation.

## Fixed protocol

- Segment numbers: `N = 4, 5, 6, 7, 8, 9, 10`
- Ordinary trials: `512` unique seeds per `N`, seeds `0,...,511`
- Body-yaw half-amplitude: `15 deg`
- Spatial phase increment: `-pi rad`
- Leg yaw: locked
- Leg pitch: independent `Bernoulli(0.5)` command per physical leg and
  half-cycle decision window
- Warm-up: `4` gait cycles
- Evaluation: `12` gait cycles (`18.85 s`)
- Trajectory sampling: `100 Hz`
- Initialization: straight, all-down, body-wave-off settle followed by a
  smooth bridge and warm-up
- MuJoCo: `3.10.0`

The strict-independent random key contains `seed`, `N`, decision-window
index, segment index, and physical side.  Random streams are therefore not
nested across different segment numbers.

## Provenance and integrity

- Ordinary records: `3,584`
- Records per `N`: `512`
- Unique seeds per `N`: `512`
- Missing seeds: `0`
- PASS trials: `3,584 / 3,584`
- Maximum MuJoCo warning count: `0`
- Every model hash matches the audited Step-26 center-root source model
- Source STL and inertial parameters were preserved
- Fore-aft geometric symmetrization was not imposed

Historical Step-26 trials were not copied into this dataset.  A replay of
`N=4, seed=0` under the current environment differed slightly from the old
record despite identical random-command statistics.  The new 512-seed
ensemble was therefore generated entirely under one current software/model
state instead of mixing provenance.

## Physical-robot signed-displacement results

| N | Trials | Mean (mm) | SD (mm) | Bootstrap 95% CI (mm) | Mean absolute (mm) | Positive fraction | Actual-contact fraction |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 512 | 30.640 | 46.035 | [26.625, 34.509] | 44.014 | 0.730 | 0.432 |
| 5 | 512 | 23.128 | 45.296 | [19.189, 27.065] | 40.765 | 0.717 | 0.428 |
| 6 | 512 | 26.480 | 41.432 | [22.881, 30.120] | 38.341 | 0.719 | 0.416 |
| 7 | 512 | 26.626 | 39.974 | [23.136, 30.292] | 38.174 | 0.736 | 0.409 |
| 8 | 512 | 33.026 | 37.502 | [29.722, 36.201] | 40.089 | 0.822 | 0.405 |
| 9 | 512 | 31.289 | 35.424 | [28.334, 34.479] | 38.521 | 0.820 | 0.400 |
| 10 | 512 | 32.307 | 35.104 | [29.339, 35.306] | 38.716 | 0.818 | 0.395 |

Every physical-robot mean confidence interval lies above zero.  The ensemble
means are not pointwise monotonic in `N`, but their fitted linear trend is

`+0.995 mm/segment`, bootstrap 95% CI `[+0.307, +1.637] mm/segment`.

The mean actual-contact fraction decreases from `0.432` at `N=4` to `0.395`
at `N=10`.  Thus the positive displacement trend cannot be explained by a
larger fraction of feet contacting the floor.  The absolute number of
available/contacting feet nevertheless grows with `N`, so contact fraction
and embodied multiplicity must be kept separate.

## Matched comparison with the symmetric null

Robot and null trials were paired by `(N, seed)`.  The following command
statistics were verified equal for every pair: commanded-down fraction,
both-down/both-up/one-down pair fractions, and mean/max pairwise state
correlation.

| N | Robot mean (mm) | Null mean (mm) | Robot - null (mm) | Paired 95% CI (mm) |
|---:|---:|---:|---:|---:|
| 4 | 30.640 | 1.142 | 29.498 | [24.246, 34.625] |
| 5 | 23.128 | 1.965 | 21.164 | [16.475, 26.022] |
| 6 | 26.480 | -2.655 | 29.134 | [24.443, 33.802] |
| 7 | 26.626 | 2.447 | 24.179 | [19.689, 28.797] |
| 8 | 33.026 | 0.760 | 32.266 | [27.799, 36.648] |
| 9 | 31.289 | 0.434 | 30.855 | [26.580, 35.139] |
| 10 | 32.307 | 1.956 | 30.352 | [26.210, 34.548] |

Across equally weighted segment numbers:

- Physical-robot pooled mean: `29.071 mm`
- Symmetric-null pooled mean: `0.864 mm`
- Matched physical-minus-null offset: `28.207 mm`, paired 95% CI
  `[26.957, 29.498] mm`
- Null slope: `+0.100 mm/segment`, 95% CI
  `[-0.480, +0.666] mm/segment`
- Robot-minus-null slope: `+0.896 mm/segment`, paired 95% CI
  `[+0.251, +1.512] mm/segment`

## Interpretation

The IID command process alone does not select the positive direction: the
center-rooted mirror null remains centered around zero.  Applying exactly the
same command realizations to the physical robot produces a robust positive
offset.  The strongest current explanation is rectification by retained
fore-aft geometry, inertia, and finite-chain boundary polarity through
frictional contact switching.

The positive robot-versus-null slope shows that this physical rectification
changes with segment number under the fixed IID-contact protocol.  It is not,
by itself, a redundancy-robustness law: this experiment does not sweep an
external noise probability or evaluate a task reliability threshold.  It is
the controlled baseline needed before those noise sweeps.

Center rooting removes the dominant first-order endpoint-tree bias, but the
Step-26 matched head/center/tail audit still bounds residual representation
sensitivity.  Publication claims should report or cite that audit rather
than labeling all remaining displacement as proven geometry-only motion.

## Main artifacts

- `iid_trials.csv`: all 3,584 current-provenance ordinary trials
- `iid_summary.csv`: per-`N` statistics
- `validation.json`: gate, seed, root, and warning audit
- `model_manifest.json`: source/experiment model hashes
- `robot_null_comparison.csv`: matched per-`N` comparison
- `comparison_statistics.json`: paired-bootstrap slopes and pooled offset
- `figures/`: Python figures
- `plot_physical_robot_iid_distributions.m`: verified MATLAB plotting script
- `matlab_figures/`: figures produced by MATLAB R2025a
- `demos/`: representative `N=4,7,10` videos and previews
- `demo_manifest.json`: median-seed selection and video paths
