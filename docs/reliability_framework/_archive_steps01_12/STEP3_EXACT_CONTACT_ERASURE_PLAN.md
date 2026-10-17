# Step 3 plan: exact module-window contact erasure

**Status:** repaired mechanism, corrected sensitivity map, and first 2048-trial independent-versus-correlated task comparison completed 8 August 2026; multiple temporally independent channel uses are next  
**Canonical notation:** `NOTATION_STANDARD.md`  
**Primary models:** \(N=6\) first, then \(N=8\)  
**Terrain:** flat  
**Yaw half-amplitude:** locked at \(15^\circ\)  
**Temporal burst length:** \(K=1\)

For the locked alternating gait,

\[
\mathcal A_\phi=\{0,\pi\},
\qquad
G=|\mathcal A_\phi|=2,
\qquad
P_\phi=2,
\qquad
\nu=\frac{N}{2}.
\]

The two phase classes are balanced, so their common multiplicity happens to satisfy \(m=\nu=N/2\). This equality is specific to the alternating motif; the implementation must interpret \(m_g\) as phase-class multiplicity, not as a general wave or repetition count.

## 1. Scientific question

The next experiment must answer one question before any performance comparison:

> Can the simulator erase the complete bilateral leg-ground contact channel of one selected segment for exactly one support window, while leaving the complete actuator command and every unselected collision channel unchanged?

Until this manipulation passes, independent-versus-correlated locomotion results are not interpretable.

## 2. Canonical event and intervention

For support event

\[
e=(c,g,\sigma),
\]

let \(j\in\{1,\ldots,m_g\}\) select a copy in phase group \(g\), and let \(i_{g,j}\) be its physical segment. The bilateral module is

\[
\mathcal M_{i_{g,j}}
=
\{(i_{g,j},\mathrm L),(i_{g,j},\mathrm R)\}.
\]

The erasure indicator is

\[
Z_{e,j}\in\{0,1\}.
\]

For \(Z_{e,j}=1\), both leg collision geoms belonging to segment \(i_{g,j}\) must have leg-ground collision availability disabled during

\[
\mathcal W_e=[t_e,t_e+\tau_{\mathrm{st}}),
\qquad
\tau_{\mathrm{st}}=dT_{\mathrm{gait}}.
\]

All actuator commands must remain nominal:

\[
\mathbf u(t)=\mathbf u^0(t).
\]

In the current XML, each leg has one collision mesh and there is no separate foot collision geom. The implementable operation is therefore exact **leg-mesh-to-ground contact erasure**, not a foot-only erasure.

## 3. Immediate first step

**Implementation status:** completed for N=6 Cases A-C. The dedicated
`ModuleContactErasureLatch` changes collision filtering only and leaves the
control vector untouched. Evidence is stored under
`exp_data/reliability/step3_exact_contact_erasure/`.

Implement and validate a deterministic module-level contact latch before introducing random masks.

### 3.1 Required software object

Create a dedicated `ModuleContactErasureLatch`. Do not modify or reuse the historical `StanceErasureLatch`, because that class changes leg-elevation commands and represents a different intervention.

The new object must:

1. receive group support transitions from the existing gait-event detector;
2. map \((g,j)\) to physical segment \(i_{g,j}\);
3. locate the left and right leg collision geom IDs for that segment;
4. store their original `geom_contype` and `geom_conaffinity` values;
5. set both collision channels to zero at event onset when \(Z_{e,j}=1\);
6. leave every actuator command untouched;
7. restore the exact stored collision values at event offset;
8. process offset before onset if both occur at the same control update;
9. log every state change and mapping.

### 3.2 Deterministic test cases

Run exactly three forced-mask cases on \(N=6\):

#### Case A: clean-equivalence control

\[
Z_{e,j}=0\quad\forall e,j.
\]

Purpose: prove that adding the new latch does not alter the original simulation.

#### Case B: one module-window erasure

Force one selected \(Z_{e,j}=1\) and every other mask value to zero.

Purpose: verify segment mapping, bilateral shutdown, isolation, timing, restoration, and contact impulse.

#### Case C: one complete phase-group erasure

For one chosen event in group \(g\), force

\[
Z_{e,1}=\cdots=Z_{e,m_g}=1.
\]

Purpose: verify the physical meaning of \(F_e=1\) before it is generated stochastically.

## 4. First-step acceptance gates

The deterministic implementation passes only if all gates below pass.

| Gate | Required evidence |
|---|---|
| Segment mapping | Every \((g,j)\) maps to the preregistered \(i_{g,j}\). For \(N=6\), \(\mathcal G_1=\{1,3,5\}\) and \(\mathcal G_2=\{2,4,6\}\). |
| Bilateral shutdown | Both left and right leg geoms of the selected segment have collision availability zero throughout \(\mathcal W_e\). |
| Isolation | Every unselected geom retains its original collision parameters. |
| Control identity | \(\max_t\|\mathbf u(t)-\mathbf u^0(t)\|_\infty=0\) to machine precision. |
| Exact timing | Disable at \(t_e\); restore at \(t_e+\tau_{\mathrm{st}}\), within one simulation step. |
| Contact erasure | Both selected sides have \(J^\perp_{e,j,\mathrm L}\le\varepsilon_J\) and \(J^\perp_{e,j,\mathrm R}\le\varepsilon_J\). |
| Exact restoration | Restored `contype/conaffinity` equals the pre-event values exactly. |
| Reactivation safety | The contact impulse immediately after restoration has no unregistered numerical spike. |
| Replay | The forced-mask trial reproduces the same event log and trajectory from the saved configuration. |

Failure of any gate is an implementation result to fix; it is not locomotion-performance evidence.

## 5. Required outputs from the first step

Store the new experiment separately under

`exp_data/reliability/step3_exact_contact_erasure/`

without overwriting Step 2. The first-step package must contain:

- machine-readable run configuration;
- event-to-segment mapping table;
- per-step selected collision state;
- nominal and applied actuator command comparison;
- bilateral normal-contact impulses;
- restoration and post-reactivation impulse log;
- deterministic summary with one PASS/FAIL entry per acceptance gate;
- at least three short top-view or oblique-view demo videos corresponding to Cases A-C;
- a manipulation-check figure showing mask, collision availability, contact impulse, and control delta on a common time axis.

## 6. Work sequence after the first step

### Registered result of the first step

All deterministic mechanism gates passed: exact segment mapping, bilateral
shutdown, isolation, control identity, support-window timing, zero contact
impulse during erasure, exact restoration, numerical stability, and replay.
The maximum support-window duration error was \(6.02\times10^{-4}\,\mathrm{s}\)
at timestep \(\Delta t=2\times10^{-3}\,\mathrm{s}\). Every selected left and
right leg accumulated exactly \(0\,\mathrm{N\,s}\) during the erased window.

Restoring collision at the nominal event offset produced a finite and
repeatable recontact transient. The peak per-step impulse was 15.80 times the
paired clean reference in Case B and 10.21 times in Case C. No non-finite state
or MuJoCo numerical warning occurred. This impulse is therefore recorded as a
physical consequence of hard reactivation, not silently classified as a
solver failure.

**Correction registered 8 August 2026.** The historical floor collision filter
rejected non-leg body geoms. It was repaired from `conaffinity=4` to
`conaffinity=6`, so both body type 2 and leg type 4 collide with the floor. A
three-policy diagnostic then showed that immediate hard restoration produced
a $0.169995\,\mathrm{N\,s}$ step impulse, whereas clearance-gated restoration
reduced it to $0.016629\,\mathrm{N\,s}$ while retaining exact nominal-control
identity. The registered environmental channel therefore uses a 2 mm
clearance gate. Retraction is kept as a separate actuation intervention. See
`STEP7_BODY_CONTACT_AND_RESTORATION_POLICY_RESULTS.md`.

### Step 2: freeze the clean displacement reference

**Status:** recomputed after body--ground collision repair for
\(N\in\{4,6,8,10\}\), with three exact deterministic replays per morphology.
The canonical position is the whole-robot COM returned by the `robot_root`
MuJoCo subtree, projected onto fixed model \(+y\).

\[
\begin{aligned}
D_0(4)&=1.4002376\,\mathrm m, &
D_0(6)&=1.3302456\,\mathrm m,\\
D_0(8)&=1.3140344\,\mathrm m, &
D_0(10)&=1.3300460\,\mathrm m.
\end{aligned}
\]

The replay standard deviation and paired exact-latch-versus-bare-model state
difference are both zero. The preregistered first-pilot tolerance is

\[
\varepsilon_D(N)=0.10|D_0(N)|.
\]

The repaired frozen configuration, per-replay trajectories, COM definition,
work diagnostics, and clean demos are stored under
`exp_data/reliability/step8_body_contact_clean_reference/`. The historical
Step 4 files remain provenance only.

### Step 3: validate stochastic mask laws in the exact-contact implementation

**Status:** completed for \((N,m)\in\{(4,2),(6,3),(8,4),(10,5)\}\).
Every statistical and physical-realization gate passed. Each channel-
morphology pair used 500,000 mask samples and a separate 20-cycle MuJoCo
contact audit. Full evidence is in `STEP5_EXACT_CHANNEL_VALIDATION_RESULTS.md`.

With \(q=0.10\) and \(K=1\), verify

\[
\Pr(F_e=1)=q^{m_g}
\]

for independent copies and

\[
\Pr(F_e=1)=q
\]

for perfectly correlated copies. This includes both mask statistics and realized zero-contact checks.

### Step 4: identify the single-erasure dynamical response

**Status:** recomputed for all 196 combinations of physical segment and complete
support window after enabling body--ground collision and registering
clearance-gated restoration. Every manipulation and replay gate passed. Full
results are in `STEP9_CORRECTED_SINGLE_ERASURE_RESULTS.md`.

Move one forced module-window erasure across physical segment, phase group, side-triggered event, and cycle. Measure

\[
\Delta D_{e,j}=\widehat D_{e,j}-D_0(N).
\]

This produces a spatial-temporal sensitivity map and reveals whether nominally homologous copies are dynamically equivalent.

The corrected map remains non-monotone, but the historical severe $N=4$
artifact disappears. No trial in any morphology left the frozen
\(\pm10\%\) displacement band; the maximum loss was $7.333\%$ for $N=4$,
$1.966\%$ for $N=6$, $0.997\%$ for $N=8$, and $0.553\%$ for $N=10$.
No trial inverted. Most $N=8$ and $N=10$ erasures increased displacement,
so \(Z_{e,j}=1\) and \(F_e=1\) remain channel events rather than task failures.

### Step 5: run the first held-out performance comparison

**Status:** completed under the preregistered balanced one-use protocol. Every
morphology--mode cell contains 256 trials. Independent reliability was 0.992
for $N=4$ and 1.000 for $N=6,8,10$; fully correlated reliability was 0.906 for
all four morphologies. All 2048 mechanism rows and eight replays passed. Full
results are in **STEP10_BALANCED_CHANNEL_RESULTS.md**.

Use flat terrain, fixed \(q\), \(K=1\), \(N\in\{4,6,8,10\}\), and independent versus perfectly correlated channels. The primary output is signed final forward displacement

\[
\widehat D
=
[\mathbf p_{\mathrm{COM}}(T_{\mathrm{eval}})-\mathbf p_{\mathrm{COM}}(0)]
\cdot\widehat{\mathbf e}_{\mathrm f}.
\]

After preregistering \(\varepsilon_D\), estimate

\[
\mathcal R_D(q,N)
=
\Pr\!\left(|\widehat D-D_0(N)|\le\varepsilon_D\right).
\]

Collapse, body contact, drift, and actuator work remain diagnostics and do not remove poor-displacement trials.

### Step 6: test redundancy scaling

**Initial one-use result:** under independent masks, complete group failure
scales as $q^m$ and only two $N=4$ group failures occurred; under fully
correlated masks it remains $q$ and 24 failures occurred in every morphology.
Task failure was exactly concordant with complete group failure in this
registered dataset. Increasing $N$ reduced the conditional loss magnitude of a
correlated group erasure but did not move those events inside the fixed 10%
task band.

Hold \(G=2\) fixed and analyze the completed morphology grid

\[
(N,m)\in\{(4,2),(6,3),(8,4),(10,5)\}.
\]

Only this grid begins to support a scaling-law claim; the mask-law result alone
does not establish a performance scaling law.

### Step 7: add temporal persistence

**Temporally independent baseline status:** completed PASS. Step 11 used two
separated temporal uses per phase group, four group uses per trial, 128 paired
replicates per morphology, and retained \(K=1\). All 1024 rows and eight exact
replays passed. Independent reliability was 0.938 for \(N=4\) and 1.000 for
\(N=6,8,10\); fully correlated reliability was 0.617 for every morphology.
The registered channel composition law was preserved, but two \(N=4\)
independent trials showed task failure from repeated partial erasures without
complete group failure. Full results are in
**STEP11_MULTI_USE_CHANNEL_RESULTS.md**.

The next extension is burst length \(K>1\) with an explicit burst-start law on
the same flat ground. Do not combine temporal persistence with intermediate
spatial correlation in that first burst experiment.

### Step 8: infer an empirical channel from terrain

Run the unchanged robot on terrain, estimate the event-level marginal and joint erasure law, replay the matched artificial channel on flat ground, and compare displacement distributions. Disagreement identifies missing physics such as slip, graded force loss, impact, or state dependence.

### Step 9: redundancy-aware control

Only after the uncontrolled channel-to-performance map is validated should feedback or reinforcement learning be used to increase effective diversity or reduce the minimum required \(N\).

## 7. Stop rule

The one-use and temporally independent multi-use production sweeps are complete
and must not be rerun with altered seeds, $q$, or tolerance as if they were the
same experiments. Before the burst sweep, freeze the burst-start process,
burst length set, placement and truncation rules, common-random-number
schedule, sample size, and the same signed primary outcome. Do not add
intermediate spatial correlation or terrain in that same experiment.
