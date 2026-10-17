# Step 40 final report: high-frequency IID contact erasure

## Scope

This is the center-rooted, measured-STL physical centipede robot experiment.
It contains six morphologies, (N=5,\ldots,10), and seven spatial phase
labels, (N_{\mathrm{wave}}^{(10)}=1,1.5,\ldots,4). The adjacent body-yaw
phase offset is locked by the ten-segment reference,

\[
\Delta\phi=-\frac{2\pi N_{\mathrm{wave}}^{(10)}}{9}.
\]

Leg yaw is fixed, every leg-pitch command remains (+5^\circ) down, and body
yaw uses a (19^\circ) half-amplitude. Each physical leg independently draws
a Bernoulli contact erasure with (q=0.5) in each of 50 windows per gait
cycle. Each trial has four clean warm-up cycles and twelve evaluated cycles
(600 channel windows). Performance is direct signed forward COM displacement
in metres; it is not normalized.

## Completion and validation

- 42 morphology-phase conditions, 512 trials each: **21,504 trials**.
- 21,504 unique condition/trial keys and **21,504 unique erasure masks**.
- Every numerical trial gate and all 42 matched (q=0) references passed.
- Maximum erased-contact violation count: **0**; MuJoCo warnings: **0**.
- Global empirical erasure probability: **0.4999699**.
- Mean same-window cross-leg correlation: **(4.39\times10^{-6})**.
- Mean lag-one temporal correlation: **(-2.47\times10^{-4})**.
- Fourteen median-representative videos (N=5 and N=10 for every phase label)
  passed exact-mask and exact-displacement replay.
- All **6,356 video frames** were decoded and screened: no blank frames,
  large luminance strobes, frozen transitions, missing red erasure marks, or
  frame-size/count errors were found.

The final canonical `validation.json`, `demo_validation.json`, and the
additional visual audit are the acceptance records.

## Main empirical result

The clearest redundancy result is a concentration effect, not a universal
increase in mean displacement. At every one of the seven locked phase
offsets, the displacement standard deviation is lower for (N=10) than for
(N=5). The ratio

\[
\frac{\operatorname{SD}(\widehat D\mid N=10)}
     {\operatorname{SD}(\widehat D\mid N=5)}
\]

ranges from **0.527 to 0.728**. Averaged across phase labels, the standard
deviation falls from **0.0512 m** at (N=5) to **0.0323 m** at (N=10), a
**36.9% reduction**.

The mean displacement, however, is phase dependent and non-monotone in
segment count. Its phase-averaged values for (N=5,\ldots,10) are 0.297,
0.337, 0.368, 0.301, 0.300, and 0.333 m. Therefore this experiment supports
the statement that additional independently perturbed legs can make
performance **more concentrated/predictable**, but it does not support the
stronger claim that adding segments always increases mean forward progress.

Spatial phase remains a major determinant of absolute displacement. The
largest condition mean is 1.015 m at (N=7),
(N_{\mathrm{wave}}^{(10)}=1); the smallest is 0.0358 m at (N=10),
(N_{\mathrm{wave}}^{(10)}=4). These values compare different gait
geometries and must not be interpreted as a pure redundancy effect.

## Information-theoretic interpretation

Each trial contains (2N\times600=1200N) independent binary channel uses.
The empirical erasure fraction therefore has

\[
\operatorname{SD}(\widehat q)
=\sqrt{\frac{q(1-q)}{1200N}},
\]

so the channel state itself concentrates as (N^{-1/2}). The measured
decrease in displacement spread is the embodied-dynamics analogue: more
independent contact opportunities reduce trial-to-trial sensitivity to a
particular erasure realization. The mapping from channel state to motion is
nonlinear, so performance variance need not follow the binomial coefficient
exactly.

The matched clean comparison also shows that contact erasure is not purely a
negative scalar penalty: depending on phase and morphology, removing contact
can reduce either useful propulsion or detrimental drag/slip. Consequently,
reliability should be defined from the full task-performance distribution,
while the (q=0) reference remains condition matched.

## Canonical deliverables

- `iid_contact_erasure_trials.csv`: all raw trial rows.
- `iid_contact_erasure_summary.csv`: condition means, medians, SDs, quartiles,
  and IQRs.
- `matched_clean_references.csv`: the 42 matched (q=0) references.
- `figures/`: mean, SD, change-from-clean, channel diagnostics, heat map, and
  one histogram panel for each phase label.
- `demos/`: fourteen exact-mask replay videos and preview frames.
- `demo_manifest.csv`: selected trial identity and replay checks.
- `validation.json`, `demo_validation.json`, and `audit/`: acceptance evidence.

Worker-level condition files are frozen in the dated reliability archive and
are not active analysis inputs. The consolidated CSVs above are authoritative.
