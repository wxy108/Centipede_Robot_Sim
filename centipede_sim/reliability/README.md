# Current reliability code

Only the center-root null/physical pipeline is active here. Historical
erasure, terrain, random-contact-development, and causal-diagnostic runners
are frozen under `_archive/frozen_2026-08-31/` and are not part of the live
API.

## Production entry points

- `run_high_frequency_iid_contact_erasure_sweep.py` — Step-40 physical-robot
  sweep over N=5–10 and the seven locked reference-10 phase offsets.  Every
  leg stays pitch-down; each physical leg collision channel is independently
  erased with Bernoulli(0.5) once per `T_g/50` window.  Produces 512 trials per
  condition, matched clean references, direct signed-COM performance tables,
  independence diagnostics, and resumable per-condition checkpoints.
- `render_high_frequency_iid_contact_erasure_demos.py` — exact stochastic-mask
  replay for the median-representative N=5 and N=10 Step-40 trial at every
  phase offset.  Red denotes an erased physical-leg collision channel, not a
  pitch-up command.
- `run_phase_offset_duty_factor_sweep.py` — Step-39 deterministic N=10
  phase-offset x commanded-duty grid. Uses the seven reference-10 phase labels
  from Step 37 and the nine peak-centred duty values from Step 38, keeps leg yaw
  fixed, and records all 63 conditions plus 21 same-rollout representative
  videos. Displacement, actual contact, tracking, and tilt remain separate.
- `run_centered_duty_factor_sweep.py` — Step-38 deterministic duty-factor
  demos at N=10 and reference wave number 2. Preserves the corrected
  segment-centered sine carrier and centers each binary down window on its
  original stance peak; changes the width only. At d=0.5 it recovers the
  corrected canonical pitch rule. Leg yaw is fixed; body yaw remains 19 deg
  with Step-37 moderate PD. No IID or contact erasure.
- `run_locked_phase_segment_iid_sweep.py` — Step-37 physical-robot sweep.
  Defines each phase offset on the N=10 reference morphology, locks that offset
  while N varies from 5 through 10, runs 128 independent trials per condition,
  pilots a stronger runtime body-yaw PD setting, and produces separate
  histograms plus predeclared IQR trend tests.
- `render_locked_phase_segment_iid_demos.py` — replays the median-representative
  trial for every Step-37 reference wave number. Use `--segments 5 6` for the
  small-robot set. Media/stability gates are separate from cross-runtime
  displacement-difference diagnostics.
- `run_locked_phase_all_down_reference.py` — deterministic N=10 reference with
  every leg always commanded pitch down, the same locked phase offsets, fixed
  leg yaw, 19-degree body-yaw amplitude, and moderate body-yaw PD. Records actual
  contact separately and renders every reference condition.
- `select_low_wobble_body_amplitudes.py` — Step-36 low-wobble body-yaw
  amplitude selection. It audits five diagnostics independently, refines each
  feasible boundary at 1-degree resolution, and renders the eight selected
  original-contact demos without modifying the robot model.
- `optimize_original_contact_body_amplitude.py` — Step-35 bounded body-yaw
  amplitude search with corrected original contact and fixed leg yaw; matched
  18.85-second COM-displacement objective, phase and long-duration checks.
- `render_physical_robot_original_contact_no_leg_yaw.py` — Step-34 original
  cave-grouped contact videos, matched to Step 33's model, amplitude and fixed
  leg-yaw targets; no randomization.
- `run_physical_robot_pitch_body_yaw_iid_wave_sweep.py` — Step-33 fixed-N=10
  physical-robot IID pitch sweep over body wave number; constant leg-yaw targets.
- `validate_physical_robot_iid_wave_controls.py` — independent Step-33 actuator
  mapping and vectorized-versus-scalar command-array audit.
- `finalize_physical_robot_iid_wave_outputs.py` — regenerate Step-33 plots and
  short demos from completed trial CSVs; verify unique trials and readable media
  without changing the raw trial table.
- `run_center_rooted_dynamic_null_iid.py` — symmetric-null IID engine.
- `run_center_rooted_physical_robot_uniform512.py` — final physical-robot
  512-trial sweep matched to the null.
- `run_center_rooted_dynamic_null_wave_flip.py` — current absolute-wave-number
  and canonical/flipped contact-pattern experiment.
- `render_center_rooted_dynamic_null_wave_flip.py` — matched Step-30 demos.
- `render_center_rooted_physical_robot_wave_demos.py` — Step-31 measured-STL
  physical-robot wave-number demos and geometric/dynamic pitch audit.
- `generate_center_rooted_dynamic_null_models.py` — null model generator.
- `generate_center_rooted_physical_robot_models.py` — audited physical model
  generator.
- `validate_center_rooted_dynamic_null.py` and
  `validate_center_rooted_physical_robot.py` — structural and stability gates.

The remaining visible Python files are dependency modules imported by these
entry points. They are retained even when their historical filenames are
broader than the current use.

One external compatibility dependency remains at `../scripts/test_models.py`;
the active pipeline imports only its actuator-index and control-command
helpers. All other former `centipede_sim/scripts` utilities are archived.

## Run location

Run from `centipede_sim/reliability` so the deliberately local dependency
imports resolve consistently. Use `--help` before any production run. The
active scripts must write only to the active experiment locations
listed in `../../exp_data/reliability/README.md`.

## Locked parameters for the uniform IID baseline

- body-yaw half-amplitude: 15 deg;
- spatial phase increment: `-pi`;
- leg yaw: locked;
- leg pitch: independent Bernoulli(0.5) per physical leg and half-cycle;
- four warm-up cycles and twelve evaluation cycles;
- strict random keys include seed, segment count, window, segment, and side.

Step 30 has its own frozen coupling contract in
`../../exp_data/reliability/step30_symmetric_null_fixed_body_leg_phase_coupling/COUPLING_AND_WAVE_NUMBER_DEFINITION.md`.
Step 31 reuses that controller unchanged and records the physical pitch
tracking limitation in
`../../exp_data/reliability/step31_physical_robot_wave_number_demos/README.md`.
