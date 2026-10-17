# Step 40 cleanup manifest

Status: **Step 40 completed and validated on 2026-09-07**. Superseded
benchmark/pilot outputs and Step-40 worker checkpoints are stored in dated,
reversible frozen archives. Reproducible Python bytecode caches are removed
only after the final data and media audits pass.

## Keep: active scientific record

- `step25*` through `step32*`: root-cause and symmetric-null evidence.
- `step33_physical_robot_pitch_body_yaw_iid_wave_sweep`: completed physical-robot evidence.
- `step34*` through `step39*`: active gait, amplitude, IID, duty-factor and phase-offset evidence.
- `step40_high_frequency_iid_contact_erasure_sweep`: completed canonical experiment (42 conditions, 21,504 trials, and 14 videos).
- `centipede_sim/reliability/_archive` and repository-level `_archive`: frozen provenance and causal-history record.
- All center-rooted physical models, source scripts, reports, manifests, raw trial tables and final media.

## Archived: confirmed non-active generated outputs

The following six directories are intermediate Step-33 timing/pilot copies.
No references to their names were found outside the candidate directories in
the live reliability code, documentation, top-level README, or active
reliability Markdown files.

| Directory | Files | Bytes | Reason |
|---|---:|---:|---|
| `exp_data/reliability/step33_benchmark` | 12 | 270,021 | superseded timing output |
| `exp_data/reliability/step33_pilot` | 13 | 358,661 | superseded pilot output |
| `exp_data/reliability/step33_rollout_benchmark` | 12 | 272,709 | superseded timing output |
| `exp_data/reliability/step33_rollout_benchmark4` | 12 | 300,253 | superseded worker-count timing output |
| `exp_data/reliability/step33_rollout_benchmark8` | 12 | 304,288 | superseded worker-count timing output |
| `exp_data/reliability/step33_rollout_benchmark16` | 12 | 314,178 | superseded worker-count timing output |

Subtotal: 73 files, 1,820,110 bytes.  All six directories were moved to
`exp_data/reliability/_archive/frozen_2026-09-04/nonactive_step33_benchmarks/`.
The move is reversible and removes them from the active evidence level.

## Delete: reproducible bytecode caches

Only directories named exactly `__pycache__` are included.  These contain no
source or scientific data and are regenerated automatically.

| Directory | Files | Bytes |
|---|---:|---:|
| `centipede_sim/reliability/__pycache__` | 26 | 1,355,754 |
| `centipede_sim/reliability/_archive/frozen_2026-08-31/__pycache__` | 16 | 740,101 |
| `centipede_sim/scripts/__pycache__` | 1 | 29,091 |
| `_archive/frozen_2026-08-31/generated_caches/centipede_sim/scripts/__pycache__` | 6 | 122,033 |

Final pre-deletion subtotal: 49 files, 2,246,979 bytes. All four exact
directories were rechecked immediately before removal.

## Step-40 completion archive

The complete per-condition `parts/` tree and scheduler logs are moved to
`exp_data/reliability/_archive/frozen_2026-09-04/step40_worker_checkpoints/`
only after the consolidated CSVs, global validation, figures, 14 demos, and
all-frame media audit pass. Partial canonical files replaced by authoritative
staging continuations remain recoverable from the staging merge backup and
SHA-256 receipt; canonical final files are hash-verified after merging.

## Guardrails

1. Every deletion target must resolve beneath
   `C:/Users/wxy22/Documents/Centipede_robot_MUJOCO`.
2. Each target is removed by its explicit literal path; no wildcard or broad
   parent-directory deletion is permitted.
3. Step-40 worker processes must finish before live bytecode caches are
   removed.
4. After deletion, all targets are checked for absence and active Step-40
   imports are compiled again.

Archived before Step 40: 73 generated files, 1,820,110 bytes.

Final Step-40 archive inventory: 126 per-condition worker files (16,614,857
bytes), the scheduler-log directory, and two orchestration logs (3,588 bytes),
for 128 files and 16,618,445 bytes in total. Final cache deletion: 49 files and
2,246,979 bytes across the four literal `__pycache__` directories listed
above. The post-operation audit confirmed that the active Step-40 directory no
longer contains worker checkpoints or orchestration logs, all four cache
directories are absent, and the consolidated scientific record remains intact.
