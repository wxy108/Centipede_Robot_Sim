# Center-rooted physical Centipede robot audit

## Scope

This stage transfers the non-geometric bias controls developed for the null
model to the measured Centipede robot.  The real STL geometry is deliberately
**not** made fore-aft symmetric.  The purpose is to remove or quantify
displacement caused by model representation, initialization, random-number
reuse, contact ordering, and dynamics-tree direction, while retaining genuine
displacement caused by the physical geometry and inertial distribution.

No source STL, measured body mass, body inertia, inertial-frame position,
joint name, actuator name, or actuator ordering was changed.  The source
joint/actuator parameters, including the existing armature settings, were
preserved.  Reversed tree edges retain the same world-frame hinge anchor; the
local hinge axis is negated so that the original positive controller
coordinate is preserved.

## Model representations

Three physically matched model families were generated for every
\(N\in\{4,5,6,7,8,9,10\}\):

1. **Head-rooted reference**: original endpoint-rooted dynamics tree, with the
   same tightened solver and explicit-contact representation used below.
2. **Center-rooted production model**: the freejoint is placed at the middle
   chassis for odd \(N\), or at the middle yaw-motor rigid body for even \(N\).
   The kinematic graph then expands outward as two branches.
3. **Tail-rooted reference**: the same physical robot re-rooted from the
   opposite endpoint.  This is an audit model, not a second physical robot.

The floor contacts are explicit and ordered from the center outward.  Equal
head/tail shells are paired and their within-pair order is alternated.  The
solver uses Newton iterations, an elliptic friction cone, 200 solver
iterations, tolerance \(10^{-12}\), and 50 no-slip iterations.

The initialization protocol is identical for every model: 1.5 s straight,
all-legs-down, body-wave-off settling; a 0.4 s smooth bridge; and four complete
warm-up gait cycles.  Roll, pitch, height, and uprightness are evaluated using
the physical `Chasis1` frame when the freejoint root is a CAD motor/link frame.
This avoids confusing a CAD local axis with the robot's physical up direction.

## Structural validation

All 21 generated models (seven segment counts times three root
representations) passed the following checks:

| Check | Result |
|---|---:|
| Body mass and inertia invariants | PASS |
| Body, geometry, joint, and actuator counts/names | PASS |
| Maximum random-configuration position error | \(8.54\times10^{-12}\,\mathrm m\) |
| Maximum random-configuration rotation error | \(6.32\times10^{-8}\,\mathrm{rad}\) |
| Stability smoke tests | PASS |
| Maximum MuJoCo warning count | 0 |

Thus the re-rooted model has the same static geometry and physical parameters;
the intentional difference is only the generalized-coordinate/dynamics-tree
representation and balanced contact enumeration.

## Matched IID experiment

For every \(N\), 32 independent seeds were run on each of the three root
representations, for \(7\times32\times3=672\) trials.  Each matched trio uses
the same seed and the same command trajectory.

- Body yaw half-amplitude: \(15^\circ\)
- Spatial phase increment: \(-\pi\)
- Leg yaw: locked
- Leg pitch: independently sampled for every physical left/right leg in every
  half-cycle, \(X_{i,s,k}\sim\operatorname{Bernoulli}(0.5)\)
- Gait angular frequency: \(4\,\mathrm{rad\,s^{-1}}\)
- Evaluation: 12 gait cycles, approximately \(18.85\,\mathrm s\)
- Trajectory sampling: 100 Hz

All 672 trials passed and produced zero MuJoCo warnings.

## Separating physical displacement from root bias

Let \(D_H\), \(D_C\), and \(D_T\) be the signed COM displacements of the
head-, center-, and tail-rooted models under the same IID command sequence.
The root-debiased endpoint estimate and first-order root-bias amplitude are

\[
D_{\mathrm{debiased}}=\frac{D_H+D_T}{2},
\qquad
B_{\mathrm{root}}=\frac{D_H-D_T}{2}.
\]

The center-root representation residual is

\[
E_C=D_C-D_{\mathrm{debiased}}.
\]

| \(N\) | \(D_H\) mean (m) | \(D_C\) mean (m) | \(D_T\) mean (m) | \(D_{\mathrm{debiased}}\) (m) | \(B_{\mathrm{root}}\) (mm) | \(E_C\) (mm) | First-order bias reduction |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 0.04815 | 0.03469 | 0.01994 | 0.03405 | 14.10 | 0.64 | 95.4% |
| 5 | 0.03688 | 0.02283 | 0.00731 | 0.02209 | 14.79 | 0.74 | 95.0% |
| 6 | 0.05345 | 0.03772 | 0.01927 | 0.03636 | 17.09 | 1.36 | 92.0% |
| 7 | 0.05068 | 0.03113 | 0.01057 | 0.03063 | 20.05 | 0.50 | 97.5% |
| 8 | 0.05882 | 0.03754 | 0.01502 | 0.03692 | 21.90 | 0.62 | 97.2% |
| 9 | 0.05360 | 0.02915 | 0.00430 | 0.02895 | 24.65 | 0.20 | 99.2% |
| 10 | 0.06311 | 0.03620 | 0.00876 | 0.03594 | 27.17 | 0.27 | 99.0% |

The center-rooted mean agrees with the head/tail endpoint average to within
0.20--1.36 mm across all \(N\).  Relative to the 14.10--27.17 mm first-order
root-bias amplitude, the center-rooted construction removes 92.0--99.2% of the
measured dynamics-tree directional bias.

The remaining positive displacement, approximately 22--37 mm in the
root-debiased ensemble mean, is not forced to zero because this model retains
the robot's real fore-aft-asymmetric STL geometry and inertial distribution.
It must therefore be treated as a physical-model result rather than as proof
of a solver artifact.

## Deterministic ablation

An additional 63 matched trials used all legs down, with the body wave either
disabled or enabled at two initial phases.

- With body wave disabled, the maximum absolute displacement was
  \(2.21\times10^{-6}\,\mathrm m\).  Static integration drift is therefore
  negligible relative to the centimeter-scale IID results.
- With body wave enabled, root location still affected the nonlinear contact
  solution.  The maximum center-versus-endpoint-average residual was 7.31 mm
  for one deterministic phase/N condition.

This second result is important: a single serial-tree MuJoCo model cannot be
claimed to be mathematically independent of generalized-coordinate ordering
during non-smooth multi-contact.  The center-root model removes the dominant
first-order endpoint bias, but the dual-root matched audit is still required
when a publication-quality displacement estimate is needed.

## Recommended production protocol

1. Use the **center-rooted physical model** for the main IID/noise sweeps.
2. Preserve strict-independent random streams across \(N\) and matched seeds
   within each head/center/tail trio.
3. For headline displacement values, report
   \(D_{\mathrm{debiased}}=(D_H+D_T)/2\) and
   \(B_{\mathrm{root}}=(D_H-D_T)/2\) alongside the center-root result.
4. Interpret the root-debiased/center-root residual mean as physical only after
   the body-wave-off drift and root-bias audit have passed.
5. Do not describe the measured robot as a fore-aft symmetric null.  Geometry
   symmetry belongs only to the separate null-model experiment.

## Main artifacts

- Generated model manifest: `model_manifest.json`
- Structure/FK/stability validation: `validation/validation_summary.json`
- IID trial table: `iid_root_trials.csv`
- IID root summary: `iid_root_summary.csv`
- Matched root-pair table: `root_paired_trials.csv`
- Matched root-pair summary: `root_paired_summary.csv`
- Deterministic ablation: `deterministic_root_trials.csv`
- Synchronized video manifest: `root_comparison_demo_manifest.csv`
- Figures: `figures/`
- Videos and previews: `demos/root_comparison/`
