# Step 3 deterministic exact-contact-erasure results

**Date:** 7 August 2026  
**Model:** `centipede_6seg.xml`  
**Terrain:** flat  
**Yaw half-amplitude:** (15^\circ)  
**Gait:** (G=2), (m=3), (K=1)  
**Output root:** `exp_data/reliability/step3_exact_contact_erasure/`

## Intervention

For the preregistered phase class \(g=1\), trigger side
\(\sigma=\mathrm L\), and side-specific cycle/event index \(c=0\), the collision
`contype` and `conaffinity` of both leg geoms belonging to each selected segment
are set to zero at event onset and restored exactly at event offset. The nominal
and applied actuator commands are identical:

\[
\max_t\|\mathbf u(t)-\mathbf u^0(t)\|_\infty=0.
\]

The one-based phase classes are

\[
\mathcal G_1=\{1,3,5\},
\qquad
\mathcal G_2=\{2,4,6\}.
\]

## Deterministic cases

| Case | Forced mask in \(\mathcal G_1\) | Selected segments | Purpose |
|---|---:|---:|---|
| A | `000` | none | Clean-equivalence control |
| B | `010` | 3 | One module-window erasure |
| C | `111` | 1, 3, 5 | Complete phase-group erasure |

## Acceptance evidence

| Check | Case A | Case B | Case C |
|---|---:|---:|---:|
| Segment mapping | PASS | PASS | PASS |
| Bilateral shutdown | PASS | PASS | PASS |
| Unselected collision isolation | PASS | PASS | PASS |
| Exact control identity | PASS | PASS | PASS |
| Timing within one simulation step | PASS | PASS | PASS |
| Selected bilateral impulse during erasure | N/A | (0\,\mathrm{N\,s}) | (0\,\mathrm{N\,s}) for every selected leg |
| Exact restoration | PASS | PASS | PASS |
| Finite state and zero MuJoCo numerical warnings | PASS | PASS | PASS |
| Deterministic replay | PASS | PASS | PASS |

The expected support-window duration is

\[
\tau_{\mathrm{st}}=0.785398\,\mathrm{s},
\]

and the maximum measured duration error is

\[
6.02\times10^{-4}\,\mathrm{s}<\Delta t=2.0\times10^{-3}\,\mathrm{s}.
\]

Case A reproduces the paired reference exactly. Over the two-cycle diagnostic
trial, signed forward displacement along model (+y) was (0.63060\,\mathrm m)
for both systems. Cases B and C produced (0.64330\,\mathrm m) and
(0.44987\,\mathrm m), respectively; these are diagnostics, not reliability
estimates.

## Registered restoration transient

Hard collision restoration produces a finite, deterministic recontact impulse:

| Case | Observed peak step impulse | Paired clean peak | Ratio |
|---|---:|---:|---:|
| B | (0.43155\,\mathrm{N\,s}) | (0.027322\,\mathrm{N\,s}) | 15.80 |
| C | (0.27903\,\mathrm{N\,s}) | (0.027322\,\mathrm{N\,s}) | 10.21 |

There were no non-finite states and no MuJoCo warnings. Consequently, the
software mechanism passes its numerical-stability check, but the impulse must
not be interpreted as negligible. It is a registered physical transient caused
by restoring a collision channel after the robot has evolved without that
support constraint.

Before stochastic independent/correlated trials, decide and preregister whether
hard recontact belongs to the intended binary-channel physics. If it does not,
a guarded-restoration model must be defined as a separate extension with its
own timing and state rules. The current result therefore records
`ready_for_stochastic_trials=false`.

## Saved artifacts

- `deterministic_summary.json`: combined machine-readable gate results;
- `run_configuration.json`: complete fixed configuration;
- `event_to_segment_mapping.csv`: ((g,j)\mapsto i_{g,j}\) and geom mapping;
- one case directory each for step logs, state changes, masks, trajectories,
  and summaries;
- `figures/manipulation_check_cases_B_C.png`;
- three paired MP4 demos and preview PNGs under `demos/`.
