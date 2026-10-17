# Step 12 protocol: long-horizon marginal-noise sweep

**Frozen before production execution:** 9 August 2026  
**Execution status:** completed PASS on 9 August 2026; 6144/6144 production
rows, 48/48 exact replays, and 12/12 decoded demos passed. The protocol below
is retained as frozen and was not changed after observing results.  
**Primary question:** how does task reliability vary with the marginal
copy-erasure probability $q$ when morphology and spatial dependence are fixed?  
**Completed experiments remain immutable:** Step 10 and Step 11 are inputs for
interpretation only; none of their code, rows, seeds, figures, or conclusions
is overwritten.  
**Terrain:** flat  
**Yaw half-amplitude:** $15^\circ$  
**Morphologies:** $N\in\{4,6,8,10\}$, with $G=2$ and $m=N/2$  
**Evaluation horizon:** $T_{\mathrm{eval}}=14T_g$  
**Temporal burst length:** $K=1$  
**Restoration:** registered 2 mm clearance gate  
**Actuator commands:** nominal

## 1. Independent long-horizon clean reference

The four-cycle Step 8 clean displacement is not reused as the task reference.
Before stochastic production, freeze a separate 14-cycle reference

\[
D_{0,14}(N)
=\big[
\mathbf p_{\mathrm{COM}}(14T_g)-\mathbf p_{\mathrm{COM}}(0)
\big]\cdot\widehat{\mathbf e}_{\mathrm f}
\]

from three exact $Z=0$ replays per morphology. The Step 12 task tolerance is

\[
\varepsilon_{D,14}(N)=0.10|D_{0,14}(N)|.
\]

The clean package must pass paired bare-versus-zero-latch state equivalence,
body--ground collision, finite-state, positive-displacement, and exact-replay
gates before stochastic trials begin.

## 2. Registered event schedule

The 14-cycle horizon contains 27 complete support-window ordinals. Register
eight temporally separated rounds:

\[
\mathcal H
=\{h_2,h_5,h_8,h_{11},h_{14},h_{17},h_{20},h_{23}\}.
\]

At every round, each of the two phase groups contributes one event. Hence

\[
U=8\quad\text{uses per phase group},
\qquad
H=GU=16\quad\text{group uses per trial}.
\]

Successive registered rounds are separated by two complete clean support
ordinals. The selected stance side alternates at every registered round, so
each phase group is used four times on each physical side. The final registered
window ends more than two gait periods before $T_{\mathrm{eval}}$, providing a
clean terminal recovery interval.

All unregistered event masks are exactly zero. Event draws are mutually
independent across the 16 registered group uses. This is a longer independent-
use experiment, not a temporal burst: $K=1$ remains fixed.

## 3. Registered marginal-noise grid

Use

\[
\mathcal Q
=\{0.01,0.03,0.06,0.10,0.20,0.35\}.
\]

The grid is chosen for the longer $H=16$ horizon. For fully correlated copies,
$1-(1-q)^{16}$ spans approximately 0.149 to 0.999 over this range; for
independent $N=10$ copies, the high endpoint still gives a non-negligible
$1-(1-q^5)^{16}$. Thus the grid covers low, transition, and near-saturation
regimes without changing $H$ or $K$.

For every registered event $e$, homologous copy $j$, and $q\in\mathcal Q$,

\[
Z_{e,j}(q)=\mathbf 1\{U_{e,j}<q\},
\qquad U_{e,j}\sim\operatorname{Uniform}(0,1).
\]

The same counter-seeded uniforms are reused across $q$, channel mode, and
morphology. Therefore

\[
q_a<q_b
\Longrightarrow
Z_{e,j}(q_a)\le Z_{e,j}(q_b),
\]

independent masks are exact prefixes across morphology, and the fully
correlated draw equals the first independent-copy draw broadcast within that
event. These are common-random-number variance reductions, not additional
statistical samples.

## 4. Sample size and seed block

Use 128 frozen replicate seeds per morphology--mode--$q$ cell:

\[
S=128,
\]

giving

\[
4\ \text{morphologies}
\times2\ \text{spatial modes}
\times6\ q\text{ values}
\times128\ \text{replicates}
=6144\ \text{production trials}.
\]

The new master seed is 20260810 and replicate
$r\in\{0,\ldots,127\}$ uses seed $20260810+r$. No outcome may be removed,
including zero-mask, partial-erasure, multiple-erasure, group-failure, or task-
failure rows.

## 5. Information-theoretic predictions

For one registered logical group event,

\[
F_e=\prod_{j=1}^{m}Z_{e,j}.
\]

Its erasure probability is

\[
p_F(q,m)=
\begin{cases}
q^m,&\text{independent copies},\\
q,&\text{fully correlated copies}.
\end{cases}
\]

Across the registered $H=16$ independent uses,

\[
p_{\mathrm{any}F}(q,m,H)
=1-[1-p_F(q,m)]^{16}.
\]

Equivalently, the probability that all 16 logical contact symbols retain at
least one physical copy is

\[
P_{\mathrm{all\ survive}}
=
\begin{cases}
(1-q^m)^{16},&\text{independent copies},\\
(1-q)^{16},&\text{fully correlated copies}.
\end{cases}
\]

Under the ideal binary-erasure-channel analogy, a logical use has the capacity
proxy

\[
C_{\mathrm{logical}}=1-p_F
=
\begin{cases}
1-q^m,&\text{independent copies},\\
1-q,&\text{fully correlated copies}.
\end{cases}
\]

and the copy-level Bernoulli uncertainty is

\[
h_2(q)=-q\log_2 q-(1-q)\log_2(1-q).
\]

No mutual information is estimated from the deterministic gait command in this
experiment. The capacity expression is an explicit BEC analogy. The measured
robot is treated as a dynamical decoder whose task reliability need not equal
$1-p_{\mathrm{any}F}$.

## 6. Primary task estimand

For spatial mode $a$, morphology $N$, and marginal noise $q$,

\[
\widehat{\mathcal R}_{D,14}^{(a)}(q,N)
=\frac1S\sum_{r=1}^{S}
\mathbf 1\!\left\{
|\widehat D_r^{(a)}-D_{0,14}(N)|
\le\varepsilon_{D,14}(N)
\right\}.
\]

Report Wilson 95% intervals, paired independent-minus-correlated reliability,
signed displacement response, work, posture diagnostics, complete 16-event
mask sequences, logical group-failure counts, and the difference between task
failure and logical group failure.

The central comparison is between the analytical channel curve
$p_{\mathrm{any}F}(q,m,16)$ and the empirical task curve
$1-\widehat{\mathcal R}_{D,14}(q,N)$. Their gap is a measured whole-body
dynamical-decoder effect, not a channel-law error.

## 7. Mechanism acceptance gates

Every production row must satisfy:

1. exactly the 16 registered event masks are sampled and all others are zero;
2. saved masks replay exactly from $(q,\text{seed},e)$;
3. every erased module has exactly zero bilateral contact impulse over its
   registered support window;
4. disable and staggered restoration records match every erased copy-event;
5. every registered round is fully restored before the next begins;
6. all unselected collision channels remain original;
7. actuator commands remain exactly nominal;
8. all collision bits are restored at trial completion;
9. body--ground collision remains enabled;
10. states remain finite, with no MuJoCo warning;
11. zero-mask trials reproduce $D_{0,14}(N)$ exactly;
12. registered replay rows reproduce the terminal state exactly;
13. masks are nested across $q$ and morphology and exactly coupled across
    channel modes.

Any failed mechanism gate stops interpretation. Task failure is an outcome,
not a mechanism gate.

## 8. Demonstration artifacts

After production, render exact clean/independent/fully-correlated triptychs for
$N=4$ and $N=10$ at every registered $q$. Each video must replay the saved
16-event mask sequence and final displacement with error at most
$10^{-12}\,\mathrm m$, then pass independent MP4 frame-count and dynamic-frame
decoding.

## 9. Stop rule

Do not change $H$, $U$, $K$, terrain, controller, restoration rule, morphology,
task tolerance, seed block, or event schedule after observing results. Do not
introduce burst persistence or intermediate spatial correlation in this
experiment. A later $K>1$ experiment must use a new directory and a marginal-
matched burst-start law.
