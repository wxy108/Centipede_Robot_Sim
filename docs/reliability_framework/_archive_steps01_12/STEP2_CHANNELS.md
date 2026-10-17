# Step 2: Independent and Fully Correlated Commanded Stance-Omission Channels

**Status:** complete historical pilot  
**Date:** 2026-08-05  
**Models:** `centipede_6seg.xml`, `centipede_8seg.xml`  
**Locked yaw half-amplitude:** \(15^\circ\)

> Notation was migrated to the canonical standard on 2026-08-07. In particular, \(\mathrm L/\mathrm R\) now mean left/right only; \(G\) is the phase-group count and \(m\) is the balanced copy multiplicity. The physical intervention in this file remains the historical leg-lifting experiment and must not be confused with the later exact module-window contact-erasure channel.

## 1. Purpose and scope

This step implemented the first controlled-noise pilot on flat terrain. It answered two narrow questions:

1. Can a nominal stance command be omitted reproducibly without changing the body-wave, shoulder, or unrelated leg commands?
2. Does complete phase-group command omission scale as \(q^m\) for independent copies and \(q\) for perfectly correlated copies?

No task reliability was claimed because the success tolerance and sample-size rule were not preregistered. Displacements and collapse counts were diagnostics, not optimization objectives or evidence of a winning controller.

## 2. Phase groups and canonical indices

Let \(N\) be the number of bilateral body segments. The current gait has

\[
G=2,
\qquad
\mathcal G_1=\{1,3,5,\ldots\},
\qquad
\mathcal G_2=\{2,4,6,\ldots\}.
\]

For these balanced even-\(N\) models,

\[
m=\frac{N}{G}=\frac{N}{2}.
\]

Therefore \(N=6\Rightarrow m=3\) and \(N=8\Rightarrow m=4\). A group support event is

\[
e=(c,g,\sigma),
\]

where \(c\) is the gait-cycle/event occurrence index, \(g\in\{1,2\}\) is the phase-group index, and \(\sigma\in\{\mathrm L,\mathrm R\}\) is the nominal stance side. The within-group copy index is \(j\in\{1,\ldots,m\}\).

## 3. Historical channel definition

For event \(e\), define

\[
Z_{e,j}\in\{0,1\},
\]

where \(Z_{e,j}=1\) commanded omission of copy \(j\)'s nominal stance and \(Z_{e,j}=0\) retained the nominal command. The marginal probability was

\[
\Pr(Z_{e,j}=1)=q,
\qquad q=0.10.
\]

For the independent channel,

\[
Z_{e,1},\ldots,Z_{e,m}
\overset{\mathrm{iid}}{\sim}
\operatorname{Bernoulli}(q).
\]

For the perfectly correlated channel,

\[
U_e\sim\operatorname{Bernoulli}(q),
\qquad
Z_{e,j}=U_e\quad\forall j.
\]

Define complete group command omission by

\[
F_e=\prod_{j=1}^{m}Z_{e,j}.
\]

Then

\[
\Pr(F_e=1)=
\begin{cases}
q^m, & \text{independent copies},\\
q, & \text{perfectly correlated copies}.
\end{cases}
\]

## 4. Reproducibility rule

Each event mask was a deterministic function of

\[
(\text{master seed},g,\sigma,c,j).
\]

The multiplicity \(m\) was not included in the per-copy seed. Thus the first three independent draws were identical in the \(N=6\) and \(N=8\) experiments with the same master seed; the \(N=8\) case appended a fourth copy instead of resampling the first three.

## 5. Historical physical intervention

The mask was sampled once at nominal stance onset and latched for the complete stance interval. If \(Z_{e,j}=1\), only the selected limb-elevation target was replaced by

\[
u^{\mathrm{leg}}_{e,j}(t)=-40^\circ.
\]

Yaw/body-wave commands, shoulder commands, nonselected leg commands, gait phase, frequency, duty factor, and event timing remained nominal.

This was a **commanded stance omission**, not exact contact deletion. Whole-body motion or collapse could still produce incidental leg-ground contact. It is therefore retained as a historical pilot rather than used as the physical definition of the new channel.

## 6. Verification layers

### Layer A: mask law

For each channel and \(m\in\{3,4\}\), 100,000 masks were sampled with seed 20260805.

| Channel | \(m\) | Marginal \(\widehat q\) | Empirical \(\Pr(F=1)\) | Theory | Mean pairwise correlation |
|---|---:|---:|---:|---:|---:|
| Independent | 3 | 0.099057 | 0.001000 | \(q^3=0.001000\) | 0.001220 |
| Independent | 4 | 0.098888 | 0.000090 | \(q^4=0.000100\) | 0.000939 |
| Perfectly correlated | 3 | 0.097380 | 0.097380 | \(q=0.100000\) | 1.000000 |
| Perfectly correlated | 4 | 0.097380 | 0.097380 | \(q=0.100000\) | 1.000000 |

All marginal-rate, complete-group-rate, dependence-structure, and deterministic-replay checks passed.

### Layer B: implementation isolation

Thirteen unit tests passed. Across 32 MuJoCo trials:

- 32/32 trial-level implementation checks passed;
- maximum within-group stance-onset spread was \(0\,\mathrm{s}\);
- maximum change to any non-leg-elevation control was zero;
- maximum selected-leg target error was zero; and
- maximum stance-duration error was approximately \(1.398\times10^{-3}\,\mathrm{s}\), within two simulation steps.

### Layer C: realized physical contact

The XML `touch_*` sites did not produce usable contact readings, so contact truth was computed from `mj_contactForce`. For the historical single-limb event,

\[
J_{e,j}=\int_{\mathcal W_e}F^{\perp}_{e,j}(t)\,dt,
\qquad
C_{e,j}=\mathbf 1\{J_{e,j}\geq J_{\min}\},
\]

with \(J_{\min}=2.5\,\mathrm{N\,s}\). Here \(C_{e,j}=0\) was a physically effective omission. This diagnostic was not used to decide whether the mask software passed.

## 7. MuJoCo batch

The final batch used \(q=0.10\), \(N\in\{6,8\}\), \(m\in\{3,4\}\), both channel modes, eight seeds per condition, four gait cycles, flat terrain, and a \(15^\circ\) yaw half-amplitude.

| Channel | \(N\) | \(m\) | Empirical limb-omission rate | Empirical complete-group rate | Theory | Effective command-to-contact conversion | Collapses |
|---|---:|---:|---:|---:|---:|---:|---:|
| Independent | 6 | 3 | 0.08594 | 0.00000 | 0.00100 | 0.89286 | 4/8 |
| Independent | 8 | 4 | 0.08398 | 0.00000 | 0.00010 | 0.94737 | 4/8 |
| Perfectly correlated | 6 | 3 | 0.09375 | 0.09375 | 0.10000 | 1.00000 | 4/8 |
| Perfectly correlated | 8 | 4 | 0.09375 | 0.09375 | 0.10000 | 1.00000 | 4/8 |

The absence of an independent complete-group event in 128 physical group samples is expected: the event counts are only \(128q^3=0.128\) for \(m=3\) and \(128q^4=0.0128\) for \(m=4\). The 100,000-mask test, not the short physical batch, verifies the \(q^m\) law.

The equal collapse counts are not evidence that the channels are equivalent. The batch is small, the success threshold was not registered, and collapse was only a diagnostic.

## 8. Files and provenance

Implementation:

- `centipede_sim/reliability/channels.py`
- `centipede_sim/reliability/stance_omission.py`
- `centipede_sim/reliability/run_channels.py`
- `centipede_sim/reliability/validate_channel_laws.py`
- `centipede_sim/reliability/tests/test_channels_and_omission.py`

Results:

- `exp_data/reliability/step2_channels/channel_law_validation.json`
- `exp_data/reliability/step2_channels/channel_experiment_summary.json`
- per-trial files under `exp_data/reliability/step2_channels/{channel}/q_0p1000/N{6,8}/`

Raw files retain legacy keys `L` and `R` because they are immutable records of the completed run. In current notation, read those fields as `G` and `m`. New outputs must emit `G` and `m`.

## 9. Audited conclusion

Step 2 validated the two mask laws:

\[
\boxed{\Pr(F_e=1)=q^m\ \text{under independence}},
\qquad
\boxed{\Pr(F_e=1)=q\ \text{under perfect correlation}}.
\]

It did not establish a task-level reliability law because the intervention did not guarantee zero contact and the performance success set was not fixed. The next experiment uses exact bilateral segment contact erasure over one support window, nominal actuator commands, and fixed-duration forward displacement relative to the clean reference. See `NOTATION_STANDARD.md` and the revised formal framework.
