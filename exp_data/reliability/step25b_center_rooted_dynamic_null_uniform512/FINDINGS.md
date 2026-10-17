# Uniform-512 center-rooted dynamic-null IID experiment

## Status

This directory is the production uniform-sample replacement for the earlier
mixed-history `step25_center_rooted_dynamic_null` dataset.  It was generated
from scratch under one software and model state.  No ordinary trial record was
copied from the earlier directory.

## Fixed experiment definition

- Segment numbers: `N = 4, 5, 6, 7, 8, 9, 10`
- Ordinary IID trials: `512` unique seeds per `N`, seeds `0,...,511`
- Complement diagnostics: `16` pairs per `N`, excluded from ordinary means
- Body-yaw half amplitude: `15 deg`
- Spatial body phase increment: `-pi rad`
- Leg yaw: locked
- Leg-pitch process: independent `Bernoulli(0.5)` per physical leg and
  half-cycle decision window
- Warm-up: `4` gait cycles
- Evaluation: `12` gait cycles
- Trajectory sampling: `100 Hz`
- Initialization: straight, all-pitch-down, body-wave-off symmetric settle
- Model: center-rooted, fore-aft mirror-paired dynamic null with explicit
  mirror contact pairs

The simulation used the MuJoCo `3.10.0` core solver.  Windows Application
Control blocked MuJoCo's eager loading of optional bundled plugin DLLs.  These
DLLs were skipped because the null-model XML files contain no plugin elements.
An already generated current-pipeline trial (`N=10`, seed `128`) was replayed
under this launch path and reproduced its signed displacement exactly.

## Integrity audit

- Ordinary records: `3,584`
- Records per `N`: `512`
- Unique seeds per `N`: `512`
- Missing seeds in `0,...,511`: `0`
- Ordinary trials passing all gates: `3,584 / 3,584`
- All trajectories use symmetric initialization: yes
- All trajectories use the center-rooted model: yes
- All trajectories use explicit mirror contact pairs: yes

## IID signed-displacement results

| N | Trials | Mean (mm) | SD (mm) | Bootstrap 95% CI (mm) | Positive fraction |
|---:|---:|---:|---:|---:|---:|
| 4 | 512 | +1.142 | 37.295 | [-1.959, +4.334] | 0.512 |
| 5 | 512 | +1.965 | 33.159 | [-0.934, +4.865] | 0.539 |
| 6 | 512 | -2.655 | 35.652 | [-5.806, +0.460] | 0.480 |
| 7 | 512 | +2.447 | 34.737 | [-0.527, +5.503] | 0.514 |
| 8 | 512 | +0.760 | 33.712 | [-2.231, +3.723] | 0.500 |
| 9 | 512 | +0.434 | 34.413 | [-2.532, +3.344] | 0.516 |
| 10 | 512 | +1.956 | 32.942 | [-1.009, +4.713] | 0.520 |

Every bootstrap 95% confidence interval contains zero.  The fitted signed-mean
slope is

`+9.979958e-05 m per segment = +0.0998 mm per segment`.

The maximum absolute ensemble mean is `2.655 mm`, while the individual-trial
standard deviations are approximately `33--37 mm`.  The data therefore show
substantial finite-trial motion in both directions but no resolved directional
ensemble mean and no evidence that mean signed displacement grows
systematically with segment number.

## Canonical files

- `iid_trials.csv`: all 3,584 ordinary trial records
- `iid_summary.csv`: per-`N` statistics and bootstrap confidence intervals
- `validation.json`: count, gate, symmetry, and trend audit
- `run_configuration.json`: fixed experimental parameters
- `complement_pairs.csv`: diagnostics only; excluded from ordinary means
- `figures/iid_displacement_histograms.png`
- `figures/iid_mean_signed_displacement_vs_N.png`
- `figures/iid_positive_fraction_vs_N.png`
- `figures/complement_pair_bias.png`
