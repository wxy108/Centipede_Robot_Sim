# Step 33: scope and interpretation

## What changes and what stays fixed

The **physical measured-STL robot**, not the biological model or symmetric
null, has ten segments and twenty legs. Its center-rooted dynamics tree,
geometry, mass/inertia, armature, gains, contact pairs, solver and initialization
are unchanged across conditions. Spatial wave number is the independent variable:

\[
N_{\mathrm{wave}}\in\{0.5,1,1.5,2,2.5,3,3.5,4\},\qquad N=10.
\]

At body joint index \(j=0,\ldots,N-2\), the yaw position target is

\[
\theta_j^{\mathrm{cmd}}(t)
=A\sin(\omega t+j\Delta\phi),\qquad
\Delta\phi=-\frac{2\pi N_{\mathrm{wave}}}{N-1},\quad
A=15^{\circ},\quad\omega=4\ \mathrm{rad/s}.
\]

The leg-yaw actuators keep their existing neutral targets (right \(-5^{\circ}\),
left \(+5^{\circ}\)). "Locked" means **no time-varying yaw command**, not a
welded joint: finite actuator compliance remains. Body-pitch targets are zero.

## IID pitch process

Let \(i=1,\ldots,10\) identify the physical segment, \(s\in\{L,R\}\) the
side, and \(n\) the fixed decision-window index. With

\[
T_g=\frac{2\pi}{\omega},\quad h=\frac{T_g}{2}=\frac{\pi}{4}\ \mathrm{s},
\quad X_{n,i,s}\overset{\mathrm{iid}}{\sim}\mathrm{Bernoulli}(0.5),
\]

the pitch target is held throughout each decision window:

\[
\alpha_{i,s}^{\mathrm{cmd}}(t)=
\begin{cases}
+5^{\circ},&X_{n,i,s}=1\quad\text{(down)},\\
-40^{\circ},&X_{n,i,s}=0\quad\text{(up)},
\end{cases}
\qquad t\in[nh,(n+1)h).
\]

Random keys include wave number, trial, window, segment and side. Trials with
the same displayed seed in two wave conditions do **not** share a random
sequence. Consecutive windows can repeat the same state; neither strict
alternation nor left-right complementarity is imposed. For each segment pair,
both-down and both-up each have probability 1/4, and exactly-one-down has
probability 1/2.

This intentionally replaces the canonical half-wave/cave pitch pattern.
Consequently, different legs inside one cave need not share a command in these
IID demos. That is the experimental intervention, not a regression to the
earlier incorrect canonical coupling. Contact is not digitally disabled.
Down/up is a command; ground contact is a dynamical outcome.

## Timing, replication and measurement

- 128 independently keyed trials per wave condition; 1,024 total.
- Time step: 0.001 s, unchanged across all conditions.
- Initialization: 1.5 s straight/all-down settling, 0.4 s cosine bridge,
  then four body-wave/IID warm-up cycles (6.284 s after rounding to time steps).
- Evaluation: twelve cycles, 18.850 s after rounding, or 18,850 physics steps.
  There are 24 independent decision windows in the evaluated interval.
- One complete simulated trajectory has 27,034 physics steps including
  initialization and warm-up. The displacement statistic excludes those stages.
- Directional displacement is total-robot center-of-mass translation in the
  fixed initial world-forward axis: \(D=y_{\mathrm{COM}}(t_1)-y_{\mathrm{COM}}(t_0)\).
  It is not a final absolute world position, head position or path length.
- Forward values retain their sign. The separate absolute plot estimates
  \(\mathbb E[|D|]\), **not** \(|\mathbb E[D]|\). Neither is normalized by a baseline.
- Error bars are pointwise 95% percentile bootstrap intervals for the ensemble
  mean (5,000 resamples). Histograms and the ECDF use trial-level signed D.
- Geometric ground-contact flags and posture are sampled at 20 Hz, plus the
  evaluation endpoint. Contact fractions are sample-based, not force-weighted
  support fractions or an independent-erasure probability.

## Validation boundaries

`semantic_validation.json` audits random-command semantics and scalar targets.
`rollout_control_validation.json` additionally checks the XML actuator-name
mapping and vectorized rollout targets against scalar targets across all eight
waves, including switching boundaries. The model SHA-256 is recorded there.

The rollout checks finite states at every physics step, MuJoCo warnings, and
posture at the sampled states. Thus "no inversion" means no inversion detected
at those sampled states, not a continuous-time proof. `iid_trials.csv` retains
all trial rows, including any failure; summary plots use passing rows. Consult
`validation.json` and the trial counts rather than assuming all trials pass.

Each six-second demo is a **short excerpt** replaying a seed closest to its
wave condition's median full-trial signed displacement. The mean and histogram
use the full 18.850 s evaluation, not the excerpt. Green marks commanded down;
red marks commanded up. The colors do not certify contact or contact erasure.
The demo manifest also records actual joint tracking errors over the excerpt:
maximum leg-yaw target error, RMS leg-pitch target error, and RMS body-yaw
target error. These are diagnostics of finite servo tracking, not new control
commands or full-ensemble statistics.

## What this experiment can and cannot establish

It measures how body-wave spatial organization changes locomotion under one
fixed IID pitch-command law on one ten-segment physical robot. It is not a
segment-number sweep, a fully correlated-noise experiment, or an erasure-q
sweep. Wave number alone is not a measured count of independent support
channels. A monotone redundancy gain or information-theoretic reliability law
cannot be inferred from this sweep without a separate channel definition and
controlled comparison.
