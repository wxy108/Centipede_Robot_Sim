# Centipede Robot MuJoCo — canonical workspace

This directory now contains one active research line. Historical experiments
are frozen under `_archive/` or `exp_data/reliability/_archive/` and must not
be used as inputs to new results.

## Start here

1. **Current code:** `centipede_sim/reliability/README.md`
2. **Current evidence:** `exp_data/reliability/README.md`
3. **Causal diagnosis:**
   `exp_data/reliability/step27_directional_bias_diagnosis/REPORT.md`
4. **Publication-facing packages:** `deliverables/`
5. **Theory and notation:** `docs/reliability_framework/`
6. **Frozen active-file inventory:** `MAINLINE_MANIFEST_2026-08-31.csv`

## Active scientific line

The active robot is the measured Centipede STL model with the audited
center-root dynamics/contact representation. Its controlled null is the
center-rooted fore-aft mirror-symmetric dynamic model.

The active evidence chain is intentionally short:

| Stage | Purpose |
|---|---|
| Step 25 | Canonical center-root symmetric-null model source and compact validation |
| Step 25b | Final symmetric-null IID baseline, 512 trials per segment count |
| Step 26 | Matched head/center/tail physical-root audit and physical model source |
| Step 26b | Final physical-robot IID baseline, 512 trials per segment count |
| Step 27 | Frozen directional-bias diagnosis and formal report |
| Step 30 | Historical wave-number definition; pitch/cave mapping superseded |
| Step 31 | Invalid per-segment pitch parameterization retained for diagnosis |
| Step 32 | Current physical-robot cave-grouped wave-number demos |
| Step 33 | Physical robot, fixed N=10: body-yaw wave-number sweep with IID leg pitch and constant leg-yaw targets |
| Step 34 | Matched original-contact demos: corrected cave-grouped pitch, no IID and no leg-yaw oscillation |
| Step 35 | Original-contact body-yaw amplitude optimization, N=10: bounded 0–40 deg search per wave number, fixed leg yaw; see its validation/results files |
| Step 36 | Low-wobble amplitude selection from 80 audited conditions: separate sway/heave/tilt/heading/joint-amplitude gates, 1-deg boundary refinement, and one verified demo per wave number |
| Step 37 | Locked reference-10 phase-offset by segment-count IID sweep: 42 conditions, 5,376 trials, stronger audited body-yaw PD, separate histograms, and predeclared IQR trend tests |

Step 32 corrects the visualization-level physical transfer: body yaw is sampled
at inter-segment joints, leg commands at segment centers, and binary pitch is
constant within each body half-wave/cave. Step 35 is the separately authorized
original-contact amplitude sweep and preserves the binary 45-deg pitch step.
It changes body-yaw target amplitude only, not model parameters or the shared
15-degree baseline defaults. Smoothed/rate-limited pitch remains a separate
controller choice. Do not revive older Step 1--24 runners to implement it.

Step 36 does not overwrite the displacement-only Step-35 optima. It treats the
15-degree and 40-degree cases as low/high-amplitude references, constrains five
wobble diagnostics separately, and maximizes displacement only among candidates
that pass every diagnostic gate. The physical model, original contact logic,
fixed leg-yaw targets, timing, and binary leg-pitch commands remain unchanged.

Step 33 is the separately authorized IID experiment. It deliberately removes
the canonical cave-to-pitch coupling: every leg independently selects down/up
once per half cycle. It retains the measured center-rooted model, the binary
pitch targets, and the audited dynamics settings. Results and completion gates
are under `exp_data/reliability/step33_physical_robot_pitch_body_yaw_iid_wave_sweep/`.
This 128-trial-per-wave sweep does not replace the 512-trial-per-segment
Step-25b/26b baselines.

Step 37 extends that IID physical-robot line without redefining wave number for
each morphology. The wave-number label is defined at N=10, its adjacent-body-
yaw phase increment is held fixed for N=5..10, and only the represented
effective wave count changes with segment count. The experiment uses a common
19-degree body-yaw amplitude and a runtime-only stronger PD setting; the source
robot XML files remain unchanged. The main finding is phase-dependent
distribution concentration rather than a universal monotone mean-displacement
law. See `exp_data/reliability/step37_locked_phase_segment_iid_sweep/FINDINGS.md`.

## Locked interpretation

- The large historical displacement-versus-segment trend was a mixture of
  physical fore-aft polarity and nonphysical representation sensitivity.
- Balanced explicit contacts, a tightened solver, and controlled
  initialization reduce the historical numerical amplifier.
- Center-rooting removes the leading endpoint-root component; the physical
  robot retains a real model-level directional offset relative to the null.
- Use the uniform-512 Step-25b/26b datasets for quantitative claims.

## Archive rule

Archive directories are read-only evidence. They may be inspected to audit a
claim, but active scripts must not import from them and new outputs must never
be written into them. See `_archive/README.md` and
`_archive/ARCHIVE_MANIFEST_2026-08-31.csv`.

`MAINLINE_MANIFEST_2026-08-31.csv` records the path, size, and SHA-256 hash of
every active code, evidence, documentation, and unpacked deliverable file.
