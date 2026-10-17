# Step 9: corrected single-erasure dynamical sensitivity

**Date:** 8 August 2026  
**Status:** PASS  
**Interpretation:** deterministic intervention map, not a stochastic
independent-versus-correlated result  
**Restoration:** clearance-gated at $2\,\mathrm{mm}$  
**Body--ground collision:** enabled  
**Yaw half-amplitude:** $15^\circ$  
**Burst length:** $K=1$

## 1. Registered intervention

For support event $e=(c,g,\sigma)$, homologous copy $j$, and physical
segment $i_{g,j}$, one forced symbol $Z_{e,j}=1$ erases both sides of

\[
\mathcal M_{i_{g,j}}
=\{(i_{g,j},\mathrm L),(i_{g,j},\mathrm R)\}
\]

throughout the complete support window

\[
\mathcal W_e=[t_e,t_e+\tau_{\mathrm{st}}),
\qquad \tau_{\mathrm{st}}=\tfrac12T_{\mathrm{gait}},
\]

while preserving every nominal actuator command exactly. Collision is restored
only after the corresponding mesh has at least $2\,\mathrm{mm}$ clearance.

The paired response and positive-is-worse loss are

\[
\Delta D_{e,j}=D^{(1)}_{e,j}-D^{(0)},
\qquad
L_{e,j}=D^{(0)}-D^{(1)}_{e,j}=-\Delta D_{e,j}.
\]

## 2. Coverage and integrity

Each segment has seven support windows whose onset and offset both lie within
the four-period horizon. Therefore

\[
7(4+6+8+10)=196
\]

paired trials were required and all 196 were completed. Every trial passed:

- exact bilateral zero contact during the selected window;
- exact nominal-control pass-through;
- no change to unselected collision channels;
- exact paired-clean agreement with repaired $D_0(N)$;
- complete eventual collision restoration;
- finite states and zero MuJoCo warnings.

Eight representative reruns reproduced their terminal states exactly. Two
late-boundary cases required at most 43 post-evaluation steps to audit eventual
restoration; task outputs were frozen first at $T_{\mathrm{eval}}$.

## 3. Corrected task response

| $N$ | $m$ | Trials | Mean loss / $D_0$ | Mean $|\Delta D|/D_0$ | Max loss | Max increase | Outside $\pm10\%$ |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 2 | 28 | +2.978% | 3.072% | 7.333% | 0.534% | 0 |
| 6 | 3 | 42 | +0.477% | 0.921% | 1.966% | 1.218% | 0 |
| 8 | 4 | 56 | -1.559% | 1.674% | 0.997% | 3.716% | 0 |
| 10 | 5 | 70 | -1.995% | 2.054% | 0.553% | 4.655% | 0 |

The worst corrected trial is `N4_g1_j1_i1_R_c0_h2`, with a $7.333\%$
loss. The largest increase is `N10_g2_j1_i2_L_c2_h6`, which travels
$4.655\%$ farther than its paired clean reference.

The old $77.411\%$ $N=4$ loss disappears after body collision repair and
clearance-gated restoration. No corrected trial inverted
(body up-axis $z<0$); 47 trials crossed a deliberately conservative
root-height heuristic and remain labeled only as a posture diagnostic.

## 4. Scientific interpretation

The result supports two distinct statements.

First, the physical realization is now controlled well enough to use in the
stochastic experiment: complete contact erasure is exact, commands remain
nominal, and restoration impulses are bounded. The largest post-restoration
single-step impulse in the entire map was $0.106073\,\mathrm{N\,s}$.

Second, the channel-to-task map remains nonlinear and sign-indefinite:

\[
Z_{e,j}=1\not\Rightarrow\widehat D<D_0,
\qquad
F_e=1\not\equiv\text{task failure}.
\]

The information-theoretic group-erasure law determines survival of a logical
support symbol,

\[
1-\Pr(F_e=1)=
\begin{cases}
1-q^{m_g},&\text{independent copies},\\
1-q,&\text{perfectly correlated copies},
\end{cases}
\]

but whole-body dynamics maps the realized event pattern to signed displacement.
Consequently the stochastic experiment must estimate

\[
\mathcal R_D(q,N)=
\Pr\!\left(|\widehat D-D_0(N)|\le\varepsilon_D(N)\right)
\]

directly; it may not substitute $1-\Pr(F_e=1)$ for task reliability.

## 5. Saved evidence

The 196-row trial table, ranked map, schedule, replay checks, configuration,
figures, five paired 189-frame MP4 demos, previews, and decode audit are under
`exp_data/reliability/step9_clearance_gated_single_erasure_sensitivity/`.
