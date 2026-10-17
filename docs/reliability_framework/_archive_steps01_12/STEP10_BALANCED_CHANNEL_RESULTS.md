# Step 10: balanced one-use independent versus correlated channel

**Date:** 8 August 2026  
**Status:** PASS  
**Protocol:** frozen before production in **STEP10_HELD_OUT_CHANNEL_PROTOCOL.md**  
**Trials:** 2048  
**Morphologies:** $N\in\{4,6,8,10\}$ bilateral segments  
**Channel use:** one balanced phase-group support event per trial  
**Marginal erasure probability:** $q=0.10$ per copy  
**Terrain:** flat  
**Yaw half-amplitude:** $15^\circ$  
**Restoration:** 2 mm clearance-gated  
**Primary task band:** $|\widehat D-D_0(N)|\le0.10|D_0(N)|$

## 1. Experiment

Each trial randomized exactly one complete support event $e^*$ and forced every
other event clean. Four event strata were weighted equally:

\[
\mathcal S
=\{(g,\sigma):g\in\{1,2\},\ 
\sigma\in\{\mathrm L,\mathrm R\}\}.
\]

For each morphology, channel mode, and stratum, 64 frozen replicate seeds were
used. Thus each morphology--mode cell contains 256 trials.

The two mask laws had the same per-copy marginal:

\[
\begin{aligned}
\text{independent:}\quad
&Z_{e^*,1},\ldots,Z_{e^*,m}\overset{\mathrm{iid}}{\sim}
\operatorname{Bernoulli}(q),\\
\text{fully correlated:}\quad
&Z_{e^*,1}=\cdots=Z_{e^*,m}=B_{e^*},
\quad B_{e^*}\sim\operatorname{Bernoulli}(q).
\end{aligned}
\]

Independent and correlated trials shared the same event and seed; the
correlated draw was the first independent-copy uniform draw. Independent masks
were exact prefixes across morphology.

## 2. Mechanism and reproducibility audit

All 2048 rows and all eight registered replays passed. In particular:

- 2048 unique trial IDs and exactly 256 rows per morphology--mode cell;
- complete independent/correlated pairing for every stratum and replicate;
- exact sampled-mask realization;
- exactly zero bilateral contact impulse for every erased segment-window;
- exact nominal actuator commands in every row;
- body--ground collision enabled in every model;
- every collision bit eventually restored;
- zero MuJoCo warnings and zero inverted trials;
- every zero-mask row reproduced repaired $D_0(N)$ with exactly zero error;
- exact common-random-number coupling and cross-morphology prefix nesting.

The largest post-restoration single-step impulse was
$0.096266\,\mathrm{N\,s}$. The authoritative audit is
**integrity_audit.json**.

Across all copy uses, the measured marginal erasure rates were 0.09961 for
independent copies and 0.09375 for fully correlated copies. Both are finite-
sample realizations of the registered $q=0.10$ marginal; their difference is
not a change in the channel parameter.

## 3. Primary reliability result

The estimator is

\[
\widehat{\mathcal R}_D^{(a)}(q,N)
=\frac{1}{256}
\sum_{s\in\mathcal S}\sum_{r=1}^{64}
\mathbf 1\!\left\{
|\widehat D^{(a)}_{s,r}-D_0(N)|
\le0.10|D_0(N)|
\right\}.
\]

| $N$ | $m=N/2$ | $\widehat{\mathcal R}_D$ independent | Wilson 95% CI | $\widehat{\mathcal R}_D$ correlated | Wilson 95% CI | $\Delta\widehat{\mathcal R}_D$ ind. $-$ corr. |
|---:|---:|---:|---:|---:|---:|---:|
| 4 | 2 | 0.99219 | [0.97197, 0.99785] | 0.90625 | [0.86430, 0.93619] | +0.08594 |
| 6 | 3 | 1.00000 | [0.98522, 1.00000] | 0.90625 | [0.86430, 0.93619] | +0.09375 |
| 8 | 4 | 1.00000 | [0.98522, 1.00000] | 0.90625 | [0.86430, 0.93619] | +0.09375 |
| 10 | 5 | 1.00000 | [0.98522, 1.00000] | 0.90625 | [0.86430, 0.93619] | +0.09375 |

Task-failure counts were 2/1024 for independent masks and 96/1024 for
correlated masks.

## 4. Logical group failure and task failure

For the logical group symbol,

\[
F_{e^*}=\prod_{j=1}^{m}Z_{e^*,j},
\qquad
\Pr(F_{e^*}=1)=
\begin{cases}
q^m,&\text{independent},\\
q,&\text{fully correlated}.
\end{cases}
\]

The observed counts were:

| $N$ | Independent group failures | Theory | Correlated group failures | Theory |
|---:|---:|---:|---:|---:|
| 4 | 2/256 = 0.00781 | 0.01000 | 24/256 = 0.09375 | 0.10000 |
| 6 | 0/256 | 0.00100 | 24/256 = 0.09375 | 0.10000 |
| 8 | 0/256 | 0.00010 | 24/256 = 0.09375 | 0.10000 |
| 10 | 0/256 | 0.00001 | 24/256 = 0.09375 | 0.10000 |

The absence of rare independent full-group events for $m\ge3$ is expected at
256 samples and is not used to re-estimate the mask law; Step 5 already
validated that law with 500,000 masks per cell.

Within this registered one-use flat-ground dataset,

\[
\mathbf 1\{\text{task failure}\}=F_{e^*}
\]

for all 2048 rows: there was no task failure without group failure and no group
failure that remained inside the task band. This is a useful empirical bridge,
not a universal identity. Multi-event sequences, different tolerances,
terrain, slip, or a different controller can break the equality.

## 5. Signed dynamical response

| $N$ | Mean $\Delta D/D_0$ independent | Mean $\Delta D/D_0$ correlated | Max loss independent | Max loss correlated |
|---:|---:|---:|---:|---:|
| 4 | -0.757% | -2.319% | 30.264% | 30.264% |
| 6 | -0.212% | -1.414% | 5.642% | 16.615% |
| 8 | +0.466% | -1.178% | 4.704% | 13.705% |
| 10 | +0.685% | -1.099% | 4.939% | 12.601% |

The N=4 independent maximum is large because two independent trials happened
to erase both copies; independence reduces the probability of this event, not
its conditional physical severity.

Increasing $N$ reduces the conditional displacement loss caused by a
fully-correlated full-group erasure (maximum loss falls from 30.3% at $N=4$ to
12.6% at $N=10$), but every such event still lies outside the frozen 10% band.
Therefore the correlated binary reliability remains 0.90625 at all four
morphologies. Under independent masks, additional homologous copies suppress
the full-group event probability as $q^m$, so the reliability benefit appears
directly.

This is the central first result:

\[
\boxed{
\text{the same per-leg noise level does not imply the same robot reliability;}
\quad
\text{the dependence structure controls whether redundancy is usable.}
}
\]

## 6. Scope and next experiment

This result is intentionally narrow: one randomized group event, flat ground,
one $q$, one controller, and a fixed 10% displacement tolerance. It establishes
the simplest channel-to-task bridge without temporal or terrain confounds.

The next registered extension should introduce multiple channel uses over
$T_{\mathrm{eval}}$ while keeping event draws temporally independent. Only
after that baseline is understood should a burst process or $K>1$ be added.
Terrain-derived noise remains later, where its empirical event law can be
matched to the artificial flat-ground channel.

## 7. Saved artifacts

The canonical package is
**exp_data/reliability/step10_balanced_one_use_channel/** and contains:

- frozen configuration and balanced target-event table;
- all 2048 JSON/CSV trial rows;
- eight exact replay checks;
- aggregate and paired-mode summaries;
- **integrity_audit.json**;
- the four-panel summary figure;
- four decoded 189-frame clean/independent/correlated triptych videos,
  preview strips, and video audits;
- per-morphology production parts retained for provenance.
