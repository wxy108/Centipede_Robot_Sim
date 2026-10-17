# Step 11: temporally independent multi-use channel results

**Date:** 8 August 2026  
**Status:** PASS  
**Protocol:** frozen before production in **STEP11_MULTI_USE_CHANNEL_PROTOCOL.md**  
**Trials:** 1024  
**Morphologies:** $N\in\{4,6,8,10\}$ bilateral segments  
**Phase groups:** $G=2$, with balanced multiplicity $m=N/2$  
**Temporal uses:** $U=2$ per phase group; $H=GU=4$ group uses per trial  
**Temporal burst length:** $K=1$  
**Marginal erasure probability:** $q=0.10$ per copy and use  
**Terrain:** flat  
**Yaw half-amplitude:** $15^\circ$  
**Restoration:** 2 mm clearance-gated  
**Primary task band:** $|\widehat D-D_0(N)|\le0.10|D_0(N)|$

## 1. Registered experiment

Each trial randomizes four group events arranged as two separated rounds:

\[
\mathcal E_{\mathrm{multi}}
=\{(g_1,\mathrm R,h_2),(g_2,\mathrm L,h_2),
(g_1,\mathrm L,h_5),(g_2,\mathrm R,h_5)\}.
\]

Two intervening clean support ordinals separate $h_2$ and $h_5$. The first
round must be fully restored before the second begins. Thus the experiment
adds repeated, temporally independent channel uses without introducing a
burst: $K=1$ remains fixed.

For event $e$ and homologous copy $j\in\{1,\ldots,m\}$,

\[
Z_{e,j}\sim\operatorname{Bernoulli}(q).
\]

The spatial dependence law is

\[
\begin{aligned}
\text{independent:}\quad
&Z_{e,1},\ldots,Z_{e,m}\overset{\mathrm{iid}}{\sim}
\operatorname{Bernoulli}(q),\\
\text{fully correlated:}\quad
&Z_{e,1}=\cdots=Z_{e,m}=B_e,
\quad B_e\sim\operatorname{Bernoulli}(q).
\end{aligned}
\]

Draws at distinct registered events are independent in both modes. The same
128 replicate seeds are paired across modes, and independent masks are exact
prefixes across morphology. The complete grid is

\[
4\ \text{morphologies}\times2\ \text{modes}\times128\ \text{replicates}
=1024\ \text{trials}.
\]

## 2. Mechanism and reproducibility audit

All 1024 production rows and all eight exact replays passed. In particular:

- all trial IDs, mode pairs, and registered event sets are complete and unique;
- sampled masks were realized exactly at all four registered events;
- every erased segment-window had exactly zero bilateral ground-contact impulse;
- every actuator command remained nominal;
- body--ground collision remained enabled;
- every first-round erasure was fully restored before round two;
- every collision bit was restored by the end of a trial;
- all zero-mask trials reproduced frozen $D_0(N)$ with exactly zero error;
- common-random-number mode coupling and cross-$N$ prefix nesting were exact;
- there were no MuJoCo warnings and no inverted trials.

The largest selected erased-window impulse was exactly
$0\,\mathrm{N\,s}$. The largest post-restoration single-step impulse was
$0.121254\,\mathrm{N\,s}$. Root-height flags occurred in 339 trials but were
retained only as preregistered diagnostics; none inverted and no row was
excluded. The authoritative machine-readable record is
**integrity_audit.json**.

## 3. Channel-level composition law

Define logical group failure at event $e$ by

\[
F_e=\prod_{j=1}^{m}Z_{e,j}.
\]

The per-event probability is

\[
p_F(m)=\Pr(F_e=1)=
\begin{cases}
q^m,&\text{independent},\\
q,&\text{fully correlated}.
\end{cases}
\]

Because the $H=4$ registered group events are temporally independent,

\[
\Pr\!\left(\max_{e\in\mathcal E_{\mathrm{multi}}}F_e=1\right)
=1-[1-p_F(m)]^4.
\]

The production realizations are:

| $N$ | $m$ | Independent: any group failure | Theory | Correlated: any group failure | Theory |
|---:|---:|---:|---:|---:|---:|
| 4 | 2 | 6/128 = 0.046875 | 0.039404 | 49/128 = 0.382812 | 0.343900 |
| 6 | 3 | 0/128 | 0.003994 | 49/128 = 0.382812 | 0.343900 |
| 8 | 4 | 0/128 | 0.000400 | 49/128 = 0.382812 | 0.343900 |
| 10 | 5 | 0/128 | 0.000040 | 49/128 = 0.382812 | 0.343900 |

At the event level, the corresponding counts are 6/512 for independent
$N=4$, 0/512 for independent $N=6,8,10$, and 57/512 for every correlated
morphology. These finite-sample counts are consistent with the frozen law;
the high-sample mask-law validation remains Step 5 and is not re-estimated
from this physical pilot.

## 4. Task-level reliability

For mode $a\in\{\mathrm{ind},\mathrm{corr}\}$,

\[
\widehat{\mathcal R}_D^{(a)}(N)
=\frac1{128}\sum_{r=1}^{128}
\mathbf 1\!\left\{
|\widehat D_r^{(a)}-D_0(N)|\le0.10|D_0(N)|
\right\}.
\]

| $N$ | Independent $\widehat{\mathcal R}_D$ | Wilson 95% CI | Correlated $\widehat{\mathcal R}_D$ | Wilson 95% CI | $\Delta\widehat{\mathcal R}_D$ ind. $-$ corr. |
|---:|---:|---:|---:|---:|---:|
| 4 | 0.93750 | [0.88151, 0.96799] | 0.61719 | [0.53073, 0.69681] | +0.32031 |
| 6 | 1.00000 | [0.97086, 1.00000] | 0.61719 | [0.53073, 0.69681] | +0.38281 |
| 8 | 1.00000 | [0.97086, 1.00000] | 0.61719 | [0.53073, 0.69681] | +0.38281 |
| 10 | 1.00000 | [0.97086, 1.00000] | 0.61719 | [0.53073, 0.69681] | +0.38281 |

Across all morphologies there were 8 independent task failures and 196
correlated task failures. In the paired table, correlated-only success never
occurred; independent-only success occurred in 41, 49, 49, and 49 matched
pairs for $N=4,6,8,10$, respectively.

## 5. Why channel failure is not identical to task failure

For all correlated rows and all independent rows with $N\ge6$, the observed
binary task outcome was concordant with whether any complete logical group
failure occurred. At $N=4$, however, two independent trials failed the task
without a complete group failure:

\[
\begin{aligned}
&\texttt{N4\_ind\_U2\_r061},\quad
\texttt{N4\_ind\_U2\_r118},\\
&\mathbf Z:
\texttt{g1R\_h2:10|g2L\_h2:00|g1L\_h5:10|g2R\_h5:00},\\
&\frac{\widehat D-D_0}{D_0}=-10.872\%.
\end{aligned}
\]

Each trial erased one of the two homologous copies at two separated events,
never both copies at the same event. The realized partial losses accumulated
through whole-body dynamics and crossed the fixed task threshold. Therefore

\[
\boxed{
\Pr(\text{logical group failure})
\text{ is an analytically composable channel quantity, but }
\mathcal R_D
\text{ must still be measured through dynamics.}
}
\]

The one-use equality observed in Step 10 was a dataset-specific bridge, not a
general identity. Step 11 supplies the first controlled counterexample while
keeping terrain, controller, $q$, $K$, and morphology definitions fixed.

## 6. Signed dynamical response

| $N$ | Mean $\Delta D/D_0$ independent | Mean $\Delta D/D_0$ correlated | Max loss independent | Max loss correlated |
|---:|---:|---:|---:|---:|
| 4 | -2.910% | -9.852% | 30.733% | 57.719% |
| 6 | -0.833% | -6.268% | 6.847% | 30.255% |
| 8 | +1.451% | -5.225% | 5.055% | 25.045% |
| 10 | +1.927% | -4.968% | 2.867% | 24.128% |

Under the common-random-number construction, the correlated channel realizes
the same 49 any-group-failure trials at every $N$. Every such trial remains
outside the fixed 10% task band, so correlated binary reliability is identical
across morphology. Nevertheless, the mean and worst displacement losses both
shrink as $N$ increases. Thus redundancy has two distinct effects:

1. under independent copy noise, it suppresses complete logical failure from
   $q^m$ and rapidly drives the observed task failure rate toward zero;
2. conditional on a fully correlated group failure, extra body modules still
   reduce dynamical severity, although not enough here to cross the registered
   10% success boundary.

## 7. Interpretation and next registered step

Step 11 completes the simplest temporal-composition experiment. It supports
the proposed information-theoretic story in the precise channel-level sense:
independent spatial copies yield an exponential $q^m$ group-failure law, while
fully correlated copies retain an $O(q)$ failure probability, and independent
channel uses compose as $1-(1-p_F)^H$.

It also establishes the necessary limit of that analogy: the robot is a
dynamical decoder with memory, so multiple partial erasures can accumulate
even when channel draws are temporally independent. A task scaling law will
therefore require either a measured dynamical transfer model or a conservative
bound that includes partial-erasure histories, not only $F_e$.

The next experiment should add temporal persistence on the same flat ground by
registering a burst-start law and $K>1$, while keeping spatial modes restricted
to independent and fully correlated. Intermediate spatial correlation and
terrain should remain later, one factor at a time. After the burst baseline,
terrain-derived event sequences can be mapped to empirical $(q,\rho,K)$ and
replayed against the artificial channel.

## 8. Saved artifacts

The canonical package is
**exp_data/reliability/step11_temporally_independent_multi_use_channel/** and
contains:

- the frozen run configuration and four registered target events;
- all 1024 JSON/CSV trial rows and per-morphology production parts;
- eight exact final-state replay checks;
- aggregate and paired-mode summaries;
- **integrity_audit.json**;
- the four-panel summary figure;
- four exact-replay clean/independent/correlated MP4 triptychs, preview strips,
  **video_manifest.json**, and **video_decode_audit.json**.
