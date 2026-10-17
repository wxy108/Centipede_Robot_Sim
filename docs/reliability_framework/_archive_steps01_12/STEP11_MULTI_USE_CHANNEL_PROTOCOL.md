# Step 11 protocol: temporally independent multi-use channel

**Frozen before production execution:** 8 August 2026  
**Execution status:** completed PASS; results are in
**STEP11_MULTI_USE_CHANNEL_RESULTS.md**  
**Primary question:** does the one-use independent-versus-correlated reliability
advantage compose across multiple temporally independent support events?  
**Terrain:** flat  
**Yaw half-amplitude:** $15^\circ$  
**Morphologies:** $N\in\{4,6,8,10\}$, with $m=N/2$ copies per phase group  
**Marginal erasure probability:** $q=0.10$ per copy and group event  
**Temporal burst parameter:** $K=1$  
**Actuator commands:** nominal  
**Restoration:** registered 2 mm clearance gate

## 1. Event set

The four-cycle evaluation horizon contains seven complete support-window
ordinals. This first multi-use experiment randomizes only ordinals

\[
\mathcal H=\{h_2,h_5\}.
\]

At each ordinal, both phase groups generate one event. The four registered
group channel uses are

\[
\mathcal E_{\mathrm{multi}}
=\{
(g_1,\mathrm R,c_0),
(g_2,\mathrm L,c_0),
(g_1,\mathrm L,c_2),
(g_2,\mathrm R,c_2)
\}.
\]

Their timing is

| Round | Window | Onset [s] | Offset [s] | Group 1 side | Group 2 side |
|---:|---:|---:|---:|---|---|
| 1 | $h_2$ | 1.180 | 1.964 | R | L |
| 2 | $h_5$ | 3.536 | 4.320 | L | R |

Thus every phase group is used twice, once per physical side. The two rounds
are separated by two complete clean support ordinals, so clearance restoration
must finish before the next randomized event. This avoids introducing
restoration overlap or an implicit burst mechanism.

All other event masks are fixed to zero. Let

\[
U=2
\]

denote the number of randomized temporal uses per phase group; the total number
of randomized group events per trial is

\[
H=GU=4.
\]

The notation $U$ is separate from burst length $K$. Here $U=2$ and $K=1$.

## 2. Spatial and temporal probability laws

For event $e\in\mathcal E_{\mathrm{multi}}$ and copy
$j\in\{1,\ldots,m\}$:

\[
\Pr(Z_{e,j}=1)=q.
\]

Within one event,

\[
\begin{aligned}
\text{independent copies:}\quad
&Z_{e,1},\ldots,Z_{e,m}\ \text{independent},\\
\text{fully correlated copies:}\quad
&Z_{e,1}=\cdots=Z_{e,m}=B_e.
\end{aligned}
\]

Across distinct registered events, all draws are independent in both channel
modes. This is temporal independence, not a burst process.

For group failure

\[
F_e=\prod_{j=1}^{m}Z_{e,j},
\]

the per-event law is

\[
p_F^{(a)}=
\Pr(F_e=1)=
\begin{cases}
q^m,&a=\mathrm{independent},\\
q,&a=\mathrm{correlated}.
\end{cases}
\]

The probability that at least one of the four group symbols fails is therefore

\[
p_{\mathrm{any}F}^{(a)}
=1-\left(1-p_F^{(a)}\right)^H,
\qquad H=4.
\]

This is a channel-level prediction. Task reliability is still measured from
the final robot output rather than substituted by
$1-p_{\mathrm{any}F}^{(a)}$.

## 3. Seeds, pairing, and sample size

Use 128 frozen replicate seeds per morphology--mode cell:

\[
S=128.
\]

The production grid contains

\[
4\ \text{morphologies}
\times2\ \text{channel modes}
\times128\ \text{replicates}
=1024\ \text{trials}.
\]

The new master seed is 20260809; replicate
$r\in\{0,\ldots,127\}$ uses channel seed $20260809+r$. This block is distinct
from Step 10.

Each independent/correlated pair uses the same seed and all four event keys.
For every event, the correlated Bernoulli draw equals the first independent-
copy uniform draw. Larger morphologies retain smaller independent masks as
exact prefixes.

The four event keys are all included in the counter seed, so event draws are
independent of loop ordering and reproducible.

## 4. Primary task estimand

For channel mode $a$ and morphology $N$,

\[
\widehat{\mathcal R}_{D,U=2}^{(a)}(q,N)
=\frac1S\sum_{r=1}^{S}
\mathbf 1\!\left\{
|\widehat D_r^{(a)}-D_0(N)|
\le0.10|D_0(N)|
\right\}.
\]

The primary paired comparison is

\[
\Delta\widehat{\mathcal R}_{D,U=2}(N)
=
\widehat{\mathcal R}_{D,U=2}^{(\mathrm{ind})}(q,N)
-
\widehat{\mathcal R}_{D,U=2}^{(\mathrm{corr})}(q,N).
\]

Report Wilson 95% intervals, raw paired outcomes, signed displacement, and the
complete event-mask sequence. No failed, zero-mask, partial-erasure, or
multi-failure trial may be removed.

## 5. Secondary composition diagnostics

For every trial retain:

- all four masks in chronological and phase-group order;
- erased-copy count and group-failure indicator for every event;
- total and maximum simultaneous erased-copy count;
- number of group failures in the four-use sequence;
- whether any logical group symbol failed;
- signed final displacement and frozen-band task outcome;
- lateral/vertical displacement and actuator-space work;
- minimum/final body up-axis and root height;
- event-specific bilateral erased-window impulse;
- event-specific restoration timing and peak restoration impulse.

The main composition diagnostic compares

\[
\mathbf 1\{\text{task failure}\}
\quad\text{with}\quad
\mathbf 1\left\{\sum_{e\in\mathcal E_{\mathrm{multi}}}F_e\ge1\right\}.
\]

Agreement is an empirical result, not an acceptance requirement.

## 6. Mechanism acceptance gates

A production row is valid only if:

1. exactly the four registered event masks are sampled and every other event is
   zero;
2. the sampled masks replay exactly from the saved seed;
3. each masked module has zero bilateral contact impulse for its complete
   support window;
4. disable and both staggered restoration records match every sampled one;
5. the first round is fully restored before the second round begins;
6. all unselected collision channels remain original;
7. all actuator commands remain nominal;
8. all collision bits are restored at trial completion;
9. body--ground collision is enabled;
10. states remain finite and MuJoCo emits no warning;
11. zero-mask trials reproduce repaired $D_0(N)$ exactly;
12. registered replay trials reproduce the terminal state exactly.

Any failed mechanism gate stops interpretation. The task outcome itself never
determines mechanism PASS.

## 7. Stop rule

Do not add adjacent randomized events, burst persistence, terrain, intermediate
spatial correlation, or a new $q$ in this experiment. Do not change the event
set, seed block, sample size, or 10% task tolerance after observing results.
