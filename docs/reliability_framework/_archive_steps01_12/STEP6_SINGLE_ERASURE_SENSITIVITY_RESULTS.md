# Forced single-erasure dynamical sensitivity

> **SUPERSEDED AS TASK-PERFORMANCE EVIDENCE (8 August 2026).** This historical
> run used a floor collision filter that admitted leg geoms but rejected every
> non-leg body geom, and it restored collision immediately at the nominal
> offset. Its raw files are retained for audit and for documenting how the
> failure was discovered, but the $77.411\%$ $N=4$ result must not be cited
> as a result of the repaired physical model. Use
> `STEP9_CORRECTED_SINGLE_ERASURE_RESULTS.md` and the Step 9 output directory.

**Date:** 7 August 2026  
**Morphologies:** \(N\in\{4,6,8,10\}\) bilateral segments, or \(2N\) legs  
**Output root:** `exp_data/reliability/step6_single_erasure_sensitivity/`  
**Terrain:** flat  
**Yaw half-amplitude:** \(15^\circ\)  
**Evaluation horizon:** four gait periods  
**Temporal burst length:** \(K=1\)

## 1. Question and scope

This experiment measures the deterministic whole-body response to exactly one
bilateral segment-pair contact erasure. It is the dynamical bridge between the
validated binary channel and the later stochastic independent-versus-
correlated task experiment. It is not itself a stochastic channel comparison.

For support event \(e=(c,g,\sigma)\) and homologous copy \(j\), the selected
physical segment is \(i_{g,j}\), with bilateral contact object

\[
\mathcal M_{i_{g,j}}
=\{(i_{g,j},\mathrm L),(i_{g,j},\mathrm R)\}.
\]

One forced symbol \(Z_{e,j}=1\) sets both leg-ground collision channels to zero
throughout the complete support window

\[
\mathcal W_e=[t_e,t_e+\tau_{\mathrm{st}}),
\qquad
\tau_{\mathrm{st}}=\tfrac12T_{\mathrm{gait}},
\]

while preserving the nominal actuator command exactly:

\[
\mathbf u(t)=\mathbf u^0(t).
\]

## 2. Paired response definition

Each erased simulation is stepped beside an identical clean simulation using
the same model state and actuator commands. Let \(D_{e,j}^{(1)}\) be the final
forward COM displacement with the forced erasure and let \(D^{(0)}\) be its
paired clean displacement. The primary signed response is

\[
\Delta D_{e,j}=D_{e,j}^{(1)}-D^{(0)}.
\]

For plots, define the positive-is-worse loss

\[
L_{e,j}=D^{(0)}-D_{e,j}^{(1)}=-\Delta D_{e,j},
\qquad
\lambda_{e,j}=100\frac{L_{e,j}}{|D_0(N)|}.
\]

Thus \(\lambda>0\) denotes reduced forward displacement and \(\lambda<0\)
denotes increased forward displacement. The registered task-band diagnostic is

\[
B_{e,j}
=\mathbf 1\!\left\{
|D_{e,j}^{(1)}-D_0(N)|\le 0.10|D_0(N)|
\right\}.
\]

This symmetric criterion is essential: an unusually large increase and an
unusually large decrease are both departures from the intended task output.

## 3. Coverage and acceptance audit

Only support windows whose onset and offset both lie inside the four-period
evaluation horizon were used. Each segment has seven such windows, giving

\[
7(4+6+8+10)=196
\]

paired trials. All 196 passed every mechanism gate:

- exactly one selected copy and one complete support window;
- one collision disable and one exact restoration;
- zero selected left and right contact impulse during erasure;
- no change to unselected collision channels;
- exact actuator-command pass-through;
- identical paired states before erasure;
- exact agreement of the paired clean run with frozen \(D_0(N)\);
- finite states and zero MuJoCo numerical warnings.

Eight independently rerun representative trials reproduced their terminal
states exactly. The maximum timing error was
\(1.3982\times10^{-3}\,\mathrm s<\Delta t=2\times10^{-3}\,\mathrm s\).

## 4. Displacement results

| \(N\) | \(m=N/2\) | Trials | Mean loss / \(D_0\) | Mean \(|\)response\(|/D_0\) | Maximum loss | Maximum increase | Outside \(\pm10\%\) band |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 2 | 28 | +15.254% | 15.254% | 77.411% | 0.000% | 13 |
| 6 | 3 | 42 | -0.034% | 0.995% | 1.702% | 3.027% | 0 |
| 8 | 4 | 56 | -1.881% | 1.925% | 1.239% | 5.350% | 0 |
| 10 | 5 | 70 | -2.495% | 2.507% | 0.424% | 5.121% | 0 |

The sign counts were:

| \(N\) | Reduced displacement | Increased displacement | Unchanged |
|---:|---:|---:|---:|
| 4 | 28 | 0 | 0 |
| 6 | 25 | 17 | 0 |
| 8 | 1 | 55 | 0 |
| 10 | 1 | 69 | 0 |

The worst case was early erasure of segment 4 in the \(N=4\) robot,
`N4_g2_j2_i4_R_c0_h1`, with \(77.411\%\) displacement loss. The largest
increase was `N8_g2_j1_i2_L_c2_h6`, which traveled \(5.350\%\) farther than
its paired clean reference.

## 5. Interpretation

The central result is that the channel-to-task map is not monotone:

\[
Z_{e,j}=1
\;\not\Rightarrow\;
\widehat D<D_0,
\qquad
F_e=1
\;\not\equiv\;
\text{whole-robot task failure}.
\]

For \(N=4\), single erasures can cause severe and spatially localized losses.
For \(N=6\), the response is small and of mixed sign. For \(N=8\) and
\(N=10\), nearly every single erasure increases final displacement by a few
percent. Plausible dynamical explanations include removal of a drag-producing
or over-constraining contact and the impulse introduced by immediate hard
collision restoration. These are hypotheses, not established mechanisms.

The maximum post-restoration step impulse was \(1.414\,\mathrm{N\,s}\) for
\(N=4\), compared with approximately \(0.42\)--\(0.45\,\mathrm{N\,s}\) for
\(N=6,8,10\). Across all trials, displacement loss and restoration impulse
were positively associated, but this aggregate association is dominated by
the severe \(N=4\) cases. A causal recontact claim requires a separately
registered restoration-policy sensitivity test.

The heuristic `collapsed` field is retained only as a diagnostic. The primary
task outcome remains signed final forward displacement and the symmetric
reference band; no trial is excluded on the basis of posture, work, or drift.

## 6. Information-theoretic consequence

The previously validated channel statement remains

\[
\frac{I(X_e;\mathbf Y_e)}{H(X_e)}
=1-\Pr(F_e=1)
=
\begin{cases}
1-q^m,&\text{independent copies},\\
1-q,&\text{perfectly correlated copies}.
\end{cases}
\]

This quantifies survival of a logical contact symbol, not the sign or magnitude
of the robot's displacement response. The present map supplies empirical
evidence that whole-body dynamics is a necessary second layer:

\[
\text{joint erasure law}
\longrightarrow
\text{event pattern}
\longrightarrow
\text{nonlinear dynamical response}
\longrightarrow
\mathcal R_D.
\]

Accordingly, the later reliability estimate must be computed from the final
task output,

\[
\mathcal R_D(q,N)
=\Pr\!\left(|\widehat D-D_0(N)|\le\varepsilon_D(N)\right),
\]

and must not substitute \(\Pr(F_e=0)\) for \(\mathcal R_D\).

## 7. Registered next action

The prerequisites for designing the held-out independent-versus-correlated
performance experiment are complete. Production execution remains on hold
until its seed set, common-random-number pairing, sample size, signed primary
outcome, symmetric \(\pm10\%\) band, and analysis of exact versus partial
group erasures are frozen in a protocol document.

## 8. Saved artifacts

- `single_erasure_trials.csv` and `.json`: all 196 trial outcomes and gates;
- `ranked_sensitivity.csv`: trials ordered by response severity;
- `event_schedule.csv`: the seven complete windows per segment;
- `replay_checks.json`: eight exact reruns;
- `single_erasure_summary.json`: machine-readable aggregate results;
- `figures/`: heatmap and summary figure;
- `demos/`: five paired clean-versus-erased MP4 videos and previews.
