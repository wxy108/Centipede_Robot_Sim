# Step 12 results: long-horizon marginal-noise sweep

**Status:** completed PASS, 9 August 2026  
**Frozen protocol:** 14 gait cycles, $U=8$ uses per phase group,
$H=16$ logical group uses per trial, $K=1$, flat ground, nominal open-loop
controller, bilateral segment-pair contact erasure, and yaw half-amplitude
$\pm15^\circ$.  
**Production size:** $4$ morphologies $\times 2$ spatial channel laws
$\times 6$ values of $q$ $\times 128$ paired replicate seeds $=6144$
trials.

Step 10 and Step 11 were not modified or pooled with this experiment. Their
shorter horizons estimate different block-level quantities.

## 1. Registered variables

The morphology has $N\in\{4,6,8,10\}$ bilateral segment-pairs and two fixed
spatial phase groups, so the number of homologous copies per group is

\[
m=\frac{N}{2}\in\{2,3,4,5\}.
\]

For every registered logical group use $e$, copy $j$ has Bernoulli erasure
indicator $Z_{e,j}$ with marginal probability

\[
\Pr(Z_{e,j}=1)=q,
\qquad
q\in\{0.01,0.03,0.06,0.10,0.20,0.35\}.
\]

The logical group symbol fails only if all $m$ homologous copies are erased:

\[
F_e=\prod_{j=1}^{m}Z_{e,j}.
\]

Hence the exact per-use logical-erasure probability is

\[
p_F(q,m)=
\begin{cases}
q^m, & \text{independent copies},\\
q, & \text{fully correlated copies}.
\end{cases}
\]

The 16 registered uses are temporally independent. The probability that at
least one logical group symbol is erased in a trial is therefore

\[
p_{\mathrm{any}F}(q,m,H)
=1-[1-p_F(q,m)]^H,
\qquad H=16.
\]

This is why the exponent in the present block-level formula is 16. The exponent
was 4 in Step 11 because that experiment contained only four registered group
uses. It is the number of logical channel uses, not the number of phase groups,
legs, or gait cycles.

## 2. Primary task reliability

For each morphology, the 14-cycle clean displacement $D_{0,14}(N)$ was
frozen from three exact clean replays. The displacement success indicator is

\[
Y_r=
\mathbf 1\!\left\{
\left|\widehat D_r-D_{0,14}(N)\right|
\le 0.1D_{0,14}(N)
\right\},
\]

and the reported reliability is

\[
\widehat{\mathcal R}_{D,14}
=\frac{1}{128}\sum_{r=1}^{128}Y_r.
\]

The entries below are

\[
\widehat{\mathcal R}_{D,14}^{\mathrm{ind}}
\;/\;
\widehat{\mathcal R}_{D,14}^{\mathrm{corr}}.
\]

| $q$ | $N=4$, $m=2$ | $N=6$, $m=3$ | $N=8$, $m=4$ | $N=10$, $m=5$ |
|---:|---:|---:|---:|---:|
| 0.01 | 1.000 / 0.852 | 1.000 / 0.977 | 1.000 / 1.000 | 1.000 / 1.000 |
| 0.03 | 0.984 / 0.734 | 1.000 / 0.977 | 1.000 / 0.984 | 1.000 / 0.984 |
| 0.06 | 0.930 / 0.484 | 1.000 / 0.883 | 1.000 / 0.945 | 1.000 / 0.945 |
| 0.10 | 0.836 / 0.289 | 1.000 / 0.711 | 1.000 / 0.805 | 1.000 / 0.805 |
| 0.20 | 0.594 / 0.102 | 0.977 / 0.328 | 1.000 / 0.508 | 1.000 / 0.539 |
| 0.35 | 0.133 / 0.031 | 0.516 / 0.086 | 0.883 / 0.156 | 0.945 / 0.172 |

All cells contain 128 trials and retain their Wilson 95% intervals in the
machine-readable summary. The paired reliability benefit

\[
\Delta\widehat{\mathcal R}_D
=\widehat{\mathcal R}_{D,14}^{\mathrm{ind}}
-\widehat{\mathcal R}_{D,14}^{\mathrm{corr}}
\]

is nonnegative in all 24 $(N,q)$ cells. Its largest observed value is 0.773
at $N=10,q=0.35$. This is a paired finite-sample result, not a claim that
reliability must be monotone for every individual trajectory.

### 2.1 Threshold sensitivity is secondary, not a fitted validation target

Changing the task band from $0.10D_0$ to $\alpha D_0$ changes only the
empirical task-failure curves and paired task-reliability difference. It does
not change $q$, $p_F$, or the theoretical 16-use logical-erasure curves.

As a descriptive audit, the mean absolute gap between task-failure frequency
and theoretical any-logical-erasure probability was evaluated over the frozen
trials. The separate minima were

\[
\alpha_{\mathrm{ind}}^*=0.098,
\qquad
\alpha_{\mathrm{corr}}^*=0.030,
\qquad
\alpha_{\mathrm{combined}}^*=0.036.
\]

Thus no common threshold makes both spatial channel modes agree with the same
task curve. The registered $\alpha=0.10$ remains the primary task definition;
the post-hoc minima are not used to retune it or claim channel validation.

## 3. What the sweep establishes

### 3.1 Spatial diversity suppresses logical erasures

Under independent copy noise, adding homologous copies changes the logical
erasure probability from $q^2$ at $N=4$ to $q^5$ at $N=10$. Under fully
correlated copy noise, it remains $q$ for every $N$. For example, at
$q=0.10$, the theoretical probability of at least one logical erasure in 16
uses is

\[
0.1485,\;0.0159,\;0.0016,\;0.00016
\]

for independent $N=4,6,8,10$, respectively, but is 0.8147 for every fully
correlated morphology. The empirical group-erasure frequencies follow these
registered laws.

### 3.2 Embodiment also changes the dynamical decoder

The fully correlated logical channel has the same $p_F=q$ for every
morphology, yet the measured task reliability generally improves with $N$.
For example, at $q=0.20$, correlated task reliability rises from 0.102 at
$N=4$ to 0.539 at $N=10$. This improvement cannot come from the logical
channel code, because the correlated group-erasure sequence is shared and its
law is unchanged. It is evidence of a separate whole-body dynamical robustness
effect.

Conversely, a logical erasure is neither necessary nor sufficient for the
registered displacement task to fail. Across all 6144 trials:

- 64 trials failed the task without any complete logical group erasure;
- 1024 trials had at least one logical group erasure but still passed;
- 1332 trials had both; and
- 3724 trials had neither.

Partial erasures can accumulate through dynamics, while complete logical
erasures can be tolerated or recovered. Therefore, the analytical
$p_{\mathrm{any}F}$ curve and the measured task-failure curve must remain
distinct.

## 4. Precise information-theory correspondence

The experiment implements a deliberately simple hierarchy:

\[
\boxed{
\text{copy Bernoulli noise }Z_{e,j}
\longrightarrow
\text{logical erasure }F_e
\longrightarrow
\text{16-use erasure block}
\longrightarrow
\text{robot task outcome }Y
}
\]

At the copy level, the uncertainty injected per binary erasure variable is

\[
h_2(q)=-q\log_2q-(1-q)\log_2(1-q).
\]

At the logical-symbol level, the binary-erasure-channel analogy has capacity
proxy

\[
C_{\mathrm{logical}}=1-p_F
=
\begin{cases}
1-q^m,&\text{independent copies},\\
1-q,&\text{fully correlated copies}.
\end{cases}
\]

The robot is not being claimed to transmit an optimized message at this
capacity. The open-loop spatial gait supplies repeated physical copies; the
body and contact dynamics act as a stateful, nonlinear decoder; and $Y$ is
the task-level decoding criterion. Thus information theory gives the exact
copy-to-logical-erasure scaling, while the MuJoCo sweep measures the missing
logical-erasure-to-task map. This division is the central scientific result,
not an inconvenience to be fitted away.

## 5. Integrity and retained artifacts

All 6144 mechanism rows passed. The global merge independently verified exact
trial coverage, exact event registration, zero selected contact impulse, all
seven inter-round restorations, nominal control, body--ground collision,
collision-bit restoration, zero warnings, common-random-number coupling,
morphology-prefix nesting, and $q$-nested masks. Forty-eight registered
terminal-state replays had maximum state error 0.

Twelve exact clean/independent/correlated triptychs were retained: $N=4$ and
$N=10$ at every registered $q$. Every video has 660 frames, exactly matches
the saved displacements within $10^{-12}\,\mathrm m$, and passes independent
MP4 decode and dynamic-frame checks.

Primary artifacts:

- `long_horizon_q_sweep_trials.json` and `.csv`: all 6144 trial rows;
- `long_horizon_q_sweep_summary.json`: aggregate estimates and paired effects;
- `integrity_audit.json`: global acceptance gates;
- `replay_checks.json`: 48 exact terminal-state replays;
- `figures/long_horizon_q_sweep_summary.png`: registered summary figure;
- `figures/separate/`: concise independent, correlated, paired-benefit, and
  descriptive threshold-sensitivity figures;
- `figures/separate/threshold_sensitivity.json`: exact threshold-gap audit;
- `demos/video_manifest.json`: exact demo selection and replay audit; and
- `demos/video_decode_audit.json`: independent media decode audit.

## 6. Next controlled extension

The next experiment should keep the flat ground, morphology grid, spatial
channel laws, controller, task band, and signed displacement outcome fixed,
then introduce only temporal persistence $K>1$ through a preregistered,
marginal-matched burst-start law. Intermediate spatial correlation and terrain
should remain separate later steps. This preserves the causal chain from a
known artificial channel to a dynamical decoder before an empirical channel is
estimated from physical terrain.
