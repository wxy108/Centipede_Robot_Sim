# Step 37 findings: locked phase offset, varying segment count

## Experimental contract

The wave-number label is defined on a 10-segment reference robot:

\[
\Delta\phi_{\nu}^{(10)}=-\frac{2\pi\nu}{9},
\qquad
\nu\in\{1,1.5,2,2.5,3,3.5,4\}.
\]

For a selected \(\nu\), the adjacent-segment phase offset \(\Delta\phi\) is
held fixed while the physical segment count varies over \(N=5,\ldots,10\).
Consequently, the number of complete waves actually represented on an
\(N\)-segment robot is

\[
\nu_{\mathrm{eff}}(N)=\nu\frac{N-1}{9}.
\]

Each of the 42 phase-by-segment conditions contains 128 independent trials,
for 5,376 trials in total. Each trial uses four warm-up gait cycles followed
by twelve evaluated cycles (18.85 s). The body-yaw half-amplitude is fixed at
19 degrees, leg yaw is fixed, and each physical leg independently receives a
Bernoulli(0.5) pitch-down/up command once per half-cycle. This random variable
therefore controls a **contact opportunity**; realized ground contact remains
a measured dynamics outcome and is not directly erased.

## Body-yaw tracking gate

The source XML files are unchanged. A runtime-only pilot compared the source
controller with two stronger alternatives. The selected setting doubles
position gain and force capacity and multiplies velocity gain by
\(\sqrt{2}\):

- \(K_p=65.5342\);
- \(K_v=2.95008\);
- \(|\tau|_{\max}=1.88\ \mathrm{N\,m}\).

Across the pilot conditions, this reduced mean body-yaw RMS tracking error
from 5.93 degrees to 3.73 degrees (37.1%) while all stability gates passed.
The stronger candidate improved tracking by only another 0.46 degrees while
requiring 50% more gain and force authority, so it was not selected.

## Predeclared distribution-width result

The primary width metric is the interquartile range (IQR) of signed forward
displacement. For every locked phase offset, a linear IQR slope versus
segment count was estimated and its 95% interval was obtained by resampling
trials within each segment count.

| \(N_{\mathrm{wave}}^{(10)}\) | \(\Delta\phi\) | IQR slope (m/segment) | 95% interval | Result |
|---:|---:|---:|---:|---|
| 1.0 | -40 deg | -0.03587 | [-0.05149, -0.01782] | dispersed to concentrated |
| 1.5 | -60 deg | -0.01218 | [-0.02713, 0.00539] | unresolved |
| 2.0 | -80 deg | -0.00787 | [-0.02044, 0.00309] | unresolved |
| 2.5 | -100 deg | -0.00920 | [-0.01733, -0.00132] | dispersed to concentrated |
| 3.0 | -120 deg | -0.00290 | [-0.00882, 0.00358] | unresolved |
| 3.5 | -140 deg | -0.01251 | [-0.01945, -0.00542] | dispersed to concentrated |
| 4.0 | -160 deg | -0.01126 | [-0.01677, -0.00535] | dispersed to concentrated |

All seven observed IQR slopes are nonpositive. Four are statistically
resolved in the dispersed-to-concentrated direction; none is resolved in the
opposite direction. The clearest effect occurs at
\(N_{\mathrm{wave}}^{(10)}=1\), where IQR changes from 0.386 m at \(N=5\) to
0.276 m at \(N=10\). This is an overall trend, not a claim that every adjacent
segment-count step is monotone.

## Endpoint comparison

| \(N_{\mathrm{wave}}^{(10)}\) | Mean at \(N=5\) (m) | Mean at \(N=10\) (m) | IQR at \(N=5\) (m) | IQR at \(N=10\) (m) |
|---:|---:|---:|---:|---:|
| 1.0 | 0.342 | 0.581 | 0.386 | 0.276 |
| 1.5 | 0.255 | 0.475 | 0.261 | 0.233 |
| 2.0 | 0.142 | 0.204 | 0.206 | 0.168 |
| 2.5 | 0.014 | 0.079 | 0.192 | 0.134 |
| 3.0 | -0.001 | 0.030 | 0.145 | 0.143 |
| 3.5 | -0.003 | -0.014 | 0.174 | 0.109 |
| 4.0 | -0.017 | -0.073 | 0.149 | 0.090 |

Mean displacement and distribution width answer different questions. For
example, \(N_{\mathrm{wave}}^{(10)}=1\) has a resolved negative IQR slope but
its mean displacement is nonmonotone over all six segment counts. The evidence
therefore supports phase-dependent concentration of trial outcomes, not a
universal monotone improvement of expected displacement.

## Requested SD-versus-segment follow-up

The follow-up SD analysis uses the same 5,376 raw trials, not new or selected
trials. SD is the sample standard deviation (ddof=1); separate per-phase
plots include within-condition bootstrap 95% intervals. Every phase offset
has a negative fitted SD slope whose bootstrap 95% interval excludes zero.
This is an overall linear trend, not pointwise monotonicity.

| Locked phase | SD at N=5 (m) | SD at N=10 (m) | SD slope (m/segment) | 95% slope interval |
|---:|---:|---:|---:|---:|
| -40 deg | 0.2676 | 0.2102 | -0.02143 | [-0.02807, -0.01435] |
| -60 deg | 0.2318 | 0.1757 | -0.01234 | [-0.01782, -0.00672] |
| -80 deg | 0.1698 | 0.1336 | -0.00697 | [-0.01115, -0.00234] |
| -100 deg | 0.1452 | 0.1014 | -0.00706 | [-0.01055, -0.00350] |
| -120 deg | 0.1068 | 0.0995 | -0.00329 | [-0.00606, -0.00041] |
| -140 deg | 0.1156 | 0.0803 | -0.00674 | [-0.00934, -0.00409] |
| -160 deg | 0.1047 | 0.0724 | -0.00689 | [-0.00943, -0.00428] |

The intervals are individual 95% bootstrap intervals; they are not adjusted
for testing seven phase offsets simultaneously.

SD and IQR need not yield identical decisions: SD reflects the complete
distribution including tails, whereas IQR only measures the central 50%.
The earlier predeclared primary IQR result remains unchanged (four resolved
negative slopes); this requested secondary SD result resolves seven.
See `sd_by_phase_and_segment.csv`, `sd_dispersion_trends.csv`, and the
`iid_SD_vs_segment_*.png` figures.

## Scope of the claim

This experiment isolates a morphology/controller relationship under IID
pitch-commanded contact opportunities. It does not yet establish an
information-theoretic scaling law because the random input is not a direct
binary erasure channel and because the realized contact fraction varies with
the robot dynamics. The appropriate next comparison is between these
empirical displacement distributions and a channel-level failure statistic
defined from measured contact events.

## Why mean displacement decreases with reference wave number

The horizontal comparison across \(N_{\mathrm{wave}}^{(10)}\) is not a pure
redundancy comparison. Increasing the label also changes the locked spatial
phase offset and shortens the body wavelength. It therefore changes the
deterministic carrier gait at the same time that the IID contact-opportunity
process is held statistically fixed.

At \(N=10\), the following diagnostics reject two simple explanations:

| \(N_{\mathrm{wave}}^{(10)}\) | Mean displacement (m) | 95% mean interval (m) | Actual-contact fraction | Body-yaw tracking error (deg) |
|---:|---:|---:|---:|---:|
| 1.0 | 0.581 | [0.544, 0.618] | 0.287 | 9.27 |
| 1.5 | 0.475 | [0.445, 0.506] | 0.299 | 3.15 |
| 2.0 | 0.204 | [0.182, 0.227] | 0.324 | 2.73 |
| 2.5 | 0.079 | [0.061, 0.097] | 0.342 | 2.62 |
| 3.0 | 0.030 | [0.013, 0.048] | 0.355 | 2.58 |
| 3.5 | -0.014 | [-0.028, -0.001] | 0.362 | 2.56 |
| 4.0 | -0.073 | [-0.086, -0.061] | 0.365 | 2.55 |

Mean displacement falls even though realized contact becomes more frequent
and body-yaw tracking improves. These diagnostics contradict a simple
"fewer contacts" or "worse yaw tracking" explanation. They do not exclude
changes in contact timing, loading, slip, or controller transients.

An earlier version of this report attributed the decline to loss of useful
contact--body-wave phase coordination and cancellation of longitudinal
impulses. That was a hypothesis, not an identified force-level mechanism.
The all-pitch-down follow-up below demonstrates decreasing displacement and
backward motion without IID commands. IID randomization is therefore not
necessary for the observed sign reversal in the current simulation. A
force-resolved audit and matched-runtime control comparison would be needed
to identify the mechanism; these results alone do not separate physical
contact effects from numerical effects.

For a redundancy claim, raw displacement should not be compared directly
across these carrier gaits. Each phase offset needs its own deterministic
clean/original-contact reference, followed by a matched fractional loss or
task-reliability comparison under IID noise.

## All-pitch-down reference: no random commands

Seven deterministic N=10 trials retain the same locked spatial offsets,
19-degree body-yaw half-amplitude, moderate body-yaw PD, and fixed leg yaw.
After settling, every leg is commanded to the same +5-degree pitch-down
target throughout the warm-up and evaluation. There is no alternating
pitch command and no IID input. This is not the canonical phase-coupled
stance/swing gait, nor does a down command guarantee ground contact.

| Reference wave label | Locked phase | Signed displacement (m) | Actual-contact fraction |
|---:|---:|---:|---:|
| 1.0 | -40 deg | +0.8404 | 0.4741 |
| 1.5 | -60 deg | +0.7708 | 0.4993 |
| 2.0 | -80 deg | +0.4444 | 0.5466 |
| 2.5 | -100 deg | +0.2812 | 0.5854 |
| 3.0 | -120 deg | +0.1454 | 0.6288 |
| 3.5 | -140 deg | +0.0144 | 0.6558 |
| 4.0 | -160 deg | -0.1514 | 0.6634 |

Each entry is one deterministic simulation over twelve evaluated cycles
(18.85 s), not a trial-distribution mean. All seven passed finite-state,
command-semantics, sampled-orientation, warning, and media-readability gates.
The source model XML is unchanged. The contact fraction is measured over
all physical legs and sampled evaluation times.

The decrease and negative endpoint already occur without IID. This limits
the earlier interpretation but does not establish a universal wave-number
effect or identify the mechanism. The increase in contact fraction also
shows why a contact count alone cannot explain displacement direction.

Runtime provenance is important: these references explicitly use MuJoCo
3.3.7, whereas the original IID sweep did not record its MuJoCo version.
The IID/reference overlay is descriptive, not a matched-runtime estimate of
the causal effect of randomizing pitch. Do not interpret curve differences
as an isolated IID penalty. See `all_pitch_down_reference/` for the seven
videos, raw summaries, time series, configuration, and validation.

## Representative demos

Seven N=10 videos use the raw-table seed nearest to each condition's median
displacement. Every video spans the full 12-cycle evaluation and displays
commanded-down and realized-contact counts separately. All files are readable,
all retain the raw trial's displacement sign, and no MuJoCo warnings occur.

Exact displacement equality is retained as a diagnostic rather than a gate:
the original sweep did not record its MuJoCo version, whereas the approved
rendering runtime is MuJoCo 3.3.7. The maximum cross-runtime difference is
0.0188 m. See `demo_manifest.csv` and `demo_validation.json`.

The small-robot extension adds fourteen videos: all seven phase offsets for
N=5 and N=6. Each uses the raw-table median-nearest seed and shows the full
evaluation, with green/red denoting commanded down/up rather than a forced
contact state. All fourteen files contain 378 readable frames at 20 fps;
all states are finite, no warnings occur, and no sampled state is inverted.

These are illustrative re-simulations, not exact saved-trajectory playback.
The largest displacement difference from the selected raw trial is 0.0330 m.
For N=5, reference wave 2.5, the selected raw trial is +0.0101 m whereas the
rendered run is -0.0172 m. The near-zero sign disagreement is retained and
reported rather than hidden or used to replace the raw trial. None of these
rendered outcomes is substituted into the original distribution statistics.
See `small_N_demo_manifest.csv`, `small_N_demo_validation.json`, and
`DEMO_INDEX.md`.

All 5,376 trials passed the numerical and semantic validation gates. See
`validation.json`, `semantic_validation.json`, `iid_trials.csv`, and
`dispersion_trends.csv` for the auditable records.
