# Step 10 protocol: balanced one-use independent versus correlated channel

**Frozen before production execution:** 8 August 2026  
**Execution status:** completed PASS; results are in
**STEP10_BALANCED_CHANNEL_RESULTS.md**  
**Primary question:** under the same per-copy erasure probability, does spatial
correlation reduce task-level locomotion reliability?  
**Terrain:** flat  
**Yaw half-amplitude:** $15^\circ$  
**Morphologies:** $N\in\{4,6,8,10\}$, with $m=N/2$ homologous copies per phase
class  
**Erasure probability:** $q=0.10$ per copy and channel use  
**Restoration:** registered 2 mm clearance gate  
**Actuator commands:** nominal in every trial

## 1. Why the first stochastic experiment has one channel use

The first comparison deliberately contains exactly one randomized group event
per trial. This makes one simulation trial correspond to one information-
theoretic channel use and prevents spatial correlation from being confounded
with temporal bursts, event count, or terrain.

For target event $e^*$ and copies $j\in\{1,\ldots,m\}$,

\[
Z_{e^*,j}\sim\operatorname{Bernoulli}(q).
\]

The two registered dependence structures are

\[
\begin{aligned}
\text{independent:}\quad
&Z_{e^*,1},\ldots,Z_{e^*,m}\ \text{independent},\\
\text{fully correlated:}\quad
&Z_{e^*,1}=\cdots=Z_{e^*,m}=B_{e^*},
\quad B_{e^*}\sim\operatorname{Bernoulli}(q).
\end{aligned}
\]

All other event masks in that trial are fixed to zero. Thus $K=1$ is literal:
there is no adjacent randomized support window.

## 2. Balanced event strata

The alternating gait has $G=2$ phase classes and physical stance side
$\sigma\in\{\mathrm L,\mathrm R\}$. Four strata are therefore used:

\[
\mathcal S
=\{(g,\sigma):g\in\{1,2\},\ \sigma\in\{\mathrm L,\mathrm R\}\}.
\]

For each stratum, the target is the complete support window whose midpoint is
closest to $T_{\mathrm{eval}}/2$. This rule is deterministic, independent of
outcome, and identical across morphologies. Strata receive equal weight; no
post-hoc event selection is allowed.

## 3. Seeds, pairing, and sample size

Use 64 replicate seeds per stratum:

\[
S=64,
\qquad n_{\mathrm{cell}}=4S=256
\]

for each morphology--channel cell. The full production grid therefore has

\[
4\ \text{morphologies}
\times2\ \text{channels}
\times4\ \text{strata}
\times64\ \text{replicates}
=2048\ \text{trials}.
\]

The master seed is 20260808 and replicate $r\in\{0,\ldots,63\}$ uses channel
seed $20260808+r$. Each independent/correlated pair uses the same seed and
event key. The correlated draw equals the first independent-copy uniform draw;
larger morphologies preserve smaller independent masks as exact prefixes.

No failed, extreme, zero-erasure, or group-erasure row may be removed from the
task analysis.

## 4. Primary task estimand

For each morphology, the repaired clean reference $D_0(N)$ and tolerance
$\varepsilon_D(N)=0.10|D_0(N)|$ are frozen by Step 8. Trial reliability is

\[
Y_D
=\mathbf 1\!\left{
|\widehat D-D_0(N)|\le\varepsilon_D(N)
\right}.
\]

For channel mode $a\in\{\mathrm{ind},\mathrm{corr}\}$,

\[
\widehat{\mathcal R}_D^{(a)}(q,N)
=\frac1{4S}\sum_{s\in\mathcal S}\sum_{r=1}^{S}Y_{D,s,r}^{(a)}.
\]

The primary comparison is the paired reliability difference

\[
\Delta\widehat{\mathcal R}_D(N)
=\widehat{\mathcal R}_D^{(\mathrm{ind})}(q,N)
-\widehat{\mathcal R}_D^{(\mathrm{corr})}(q,N).
\]

Report cellwise Wilson 95% intervals and the raw paired outcome table. This
first sample size is a mechanistic pilot, not a definitive small-effect test;
the maximum binomial standard error at $n=256$ is 0.03125.

## 5. Secondary channel and dynamical outputs

For each row retain:

- the complete mask $\mathbf Z_{e^*}$ and erased-copy count;
- exact group failure $F_{e^*}=\prod_{j=1}^{m}Z_{e^*,j}$;
- signed $\Delta D=\widehat D-D_0(N)$ and normalized response;
- lateral and vertical displacement;
- absolute and positive actuator-space mechanical work;
- minimum/final body up-axis and minimum root height;
- bilateral erased-window impulse and restoration-step impulse;
- every manipulation, finite-state, and warning gate.

The theory check remains

\[
\Pr(F_{e^*}=1)=
\begin{cases}
q^m,&\text{independent},\\
q,&\text{fully correlated}.
\end{cases}
\]

This check describes the logical contact symbol. It is never substituted for
$\mathcal R_D$.

## 6. Acceptance and stop rules

A production row is valid only if the sampled mask is realized exactly, every
masked segment has zero bilateral contact impulse over the registered support
window, every unmasked collision channel remains original, actuator commands
remain nominal, all collision bits are eventually restored, the model has
body--ground collision enabled, states remain finite, and MuJoCo emits no
warning.

If any row fails a mechanism gate, production stops and no task conclusion is
reported until the implementation is repaired. A zero task-failure count is a
valid result and must not trigger widening of $q$ or the tolerance after seeing
the data.
