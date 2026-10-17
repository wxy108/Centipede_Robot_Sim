# Step 1 - Zero-Erasure Stance-Event Baseline

Date: 2026-08-05

Status: **PASS**

## Scope

This step adds an observational stance-event layer around the canonical
continuous CPG controller. The erasure variable is fixed at `Z == 0`; no leg,
shoulder, or spine command is modified. Stochastic erasure is not implemented
in this step.

## Canonical configuration

- Flat ground
- Yaw half-amplitude: 15 degrees, locked across models
- Angular frequency: 4 rad/s
- Inter-module phase: -pi
- Body-leg phase: 0 rad
- Stance duty factor: 0.5
- Gait period: 1.5707963268 s
- Expected stance duration: 0.7853981634 s
- MuJoCo timestep: 0.002 s
- Validation horizon: 4 gait cycles

## Phase classes

For N=6:

- L=2, R=3
- C0={1,3,5}
- C1={2,4,6}

For N=8:

- L=2, R=4
- C0={1,3,5,7}
- C1={2,4,6,8}

## Acceptance results

| Check | N=6 | N=8 |
|---|---:|---:|
| Overall status | PASS | PASS |
| Complete stance events | 42 | 56 |
| Partial terminal events | 6 | 8 |
| Maximum stance-duration error | 0.001398 s | 0.001398 s |
| Maximum same-group onset spread | 0 s | 0 s |
| Maximum control-vector difference | 0 | 0 |
| Maximum paired qpos/qvel difference | 0 | 0 |
| Complete events with direct contact | 42/42 | 56/56 |
| Direct normal-impulse range | 2.878-4.061 N s | 2.987-4.093 N s |
| Existing touch-site signal present | No | No |

The unequal per-leg count of three versus four complete events is expected:
the observation window starts while half of the legs are already in stance.
That initial truncated stance has no observed onset and is excluded. Events that
begin before the terminal time but end afterward are retained as partial rows.

## Contact measurement finding

All existing `touch_*` site sensors remained exactly zero even though MuJoCo
reported persistent contacts between the world and every `Leg{i}_{R/L}` body.
The touch sites therefore cannot be used as contact truth in the current model.

The validated event log instead computes

```text
J = integral of summed normal mj_contactForce over each nominal stance window.
```

Only contacts between a leg body and a world-body geometry are included, so
leg-leg self-collisions are excluded. Raw touch-site values remain in separate
diagnostic columns and are explicitly marked unreliable.

No threshold `J_min` has been selected. Defining effective contact `Y` is a
later preregistration step based on the clean impulse distributions.

## Files

Implementation:

- `centipede_sim/reliability/gait_events.py`
- `centipede_sim/reliability/channels.py`
- `centipede_sim/reliability/event_logger.py`
- `centipede_sim/reliability/run_baseline.py`
- `centipede_sim/reliability/tests/test_gait_events.py`

Raw results:

- `exp_data/reliability/step1_event_baseline/baseline_summary.json`
- `exp_data/reliability/step1_event_baseline/centipede_6seg_summary.json`
- `exp_data/reliability/step1_event_baseline/centipede_6seg_stance_events.csv`
- `exp_data/reliability/step1_event_baseline/centipede_8seg_summary.json`
- `exp_data/reliability/step1_event_baseline/centipede_8seg_stance_events.csv`

## Next step not yet implemented

Add a latched controller-level stance-omission gate. At each phase-group stance
onset, sample `Z` once; if `Z=1`, hold only the selected leg-elevation command at
the safe lifted target until nominal stance ends. Independent and perfectly
correlated channels remain separate implementations and must preserve the
zero-erasure pass-through test.
