# Active reliability evidence

The directories listed below form the active evidence chain. Historical material is frozen under `_archive/`
and must not be used to generate or tune new results.

| Directory | Status and role |
|---|---|
| `step40_high_frequency_iid_contact_erasure_sweep` | **Completed.** Center-rooted measured-STL physical robot, N=5–10 and seven locked reference-10 phase offsets. All legs remain pitch-down; individual physical-leg collision channels undergo IID Bernoulli(0.5) erasure in `T_g/50` windows. Contains 42 conditions × 512 trials, 42 matched clean references, 13 figures, 14 exact-mask videos, global validation, and a final report. |
| `step39_phase_offset_duty_factor_sweep` | Deterministic physical N=10 phase-offset x commanded-duty grid: seven reference-10 phase labels by nine centered duty values, 63 conditions total. Includes absolute signed displacement, observed-contact and tilt maps, controller/provenance checks, and 21 same-rollout representative videos. No IID or erasure. |
| `step38_centered_duty_factor_sweep` | Deterministic physical N=10, reference wave 2, centered stance-window duty-factor sweep. Fixed sine carrier, leg yaw, 19-degree body-yaw amplitude and moderate PD; no IID or erasure. Includes command/actual-contact diagnostics and one full-evaluation video per duty factor. |
| `step25_center_rooted_dynamic_null` | Canonical symmetric-null XML source, compact validation, summaries, figures, and selected demos. Historical raw trajectories are archived. |
| `step25b_center_rooted_dynamic_null_uniform512` | Final uniform-512 null baseline. CSV tables and figures are authoritative; raw trajectories are archived. |
| `step26_center_rooted_physical_robot` | Audited physical head/center/tail models and matched root-bias summaries. Raw trajectories are archived. |
| `step26b_center_rooted_physical_robot_uniform512` | Final uniform-512 physical baseline, matched by seed to Step 25b. CSV tables and figures are authoritative. |
| `step27_directional_bias_diagnosis` | Frozen causal report and compact evidence tables. |
| `step30_symmetric_null_fixed_body_leg_phase_coupling` | Historical wave-number experiment; wave-number definition retained, pitch/cave mapping superseded. |
| `step31_physical_robot_wave_number_demos` | Invalid per-segment pitch parameterization; retained only as diagnostic evidence. |
| `step32_physical_robot_cave_grouped_wave_demos` | Current physical-robot wave-number demos with joint/segment collocation and one binary contact state per body half-wave/cave. |
| `step33_physical_robot_pitch_body_yaw_iid_wave_sweep` | Fixed N=10 measured-STL robot, varying body wave number, IID pitch per leg/half-cycle, constant leg-yaw targets. 128 trials per wave; consult validation.json for completion. |
| `step34_physical_robot_original_contact_no_leg_yaw_demos` | Deterministic original-contact demos matched to Step 33, with corrected Step-32 cave parity, 15-degree body-yaw half-amplitude and constant leg-yaw targets. |
| `step35_original_contact_amplitude_optimization` | Original-contact N=10 amplitude search (0–40 deg per wave number), no IID, fixed leg yaw. `trials.csv` records measured displacement; `results.json` and `validation.json` define completion and bounded optima. |
| `step36_original_contact_low_wobble_amplitudes` | Low-wobble continuation of Step 35. Eighty audited conditions, separate diagnostic limits, 1-degree boundary refinement, and eight verified final demos. `selected_amplitudes.csv`, `validation.json`, and `demo_validation.json` are the primary records. |
| `step37_locked_phase_segment_iid_sweep` | Measured center-rooted physical robot, reference-N=10 phase labels 1.0–4.0, locked phase offsets, N=5–10, and 128 IID trials per condition. Contains raw/summary tables, one histogram panel per offset, stronger-PD pilot evidence, validation, and predeclared IQR trend tests. |

## Quantitative authority

- Null IID trials: `step25b_center_rooted_dynamic_null_uniform512/iid_trials.csv`
- Physical IID trials: `step26b_center_rooted_physical_robot_uniform512/iid_trials.csv`
- Matched interpretation:
  `step26b_center_rooted_physical_robot_uniform512/FINDINGS.md`
- Root-bias diagnosis: `step27_directional_bias_diagnosis/REPORT.md`
- Current physical cave-grouping contract and demos:
  `step32_physical_robot_cave_grouped_wave_demos/README.md`
- Current physical IID body-wave sweep:
  `step33_physical_robot_pitch_body_yaw_iid_wave_sweep/README.md`
- Current low-wobble original-contact amplitudes and demos:
  `step36_original_contact_low_wobble_amplitudes/README.md`
- Current locked-phase-by-segment IID findings:
  `step37_locked_phase_segment_iid_sweep/FINDINGS.md`
- Current phase-offset-by-commanded-duty map:
  `step39_phase_offset_duty_factor_sweep/README.md`
- Current high-frequency IID contact-erasure contract and results:
  `step40_high_frequency_iid_contact_erasure_sweep/FINAL_REPORT.md`
  and `step40_high_frequency_iid_contact_erasure_sweep/EXPERIMENT_CONTRACT.md`

## Raw-output policy

Per-trial NPZ trajectories and duplicated JSON trial records are not active
analysis inputs. They are frozen under
`_archive/frozen_2026-08-31/raw_outputs/`. The compact CSV tables, validation
files, figures, selected demos, and canonical model XML files remain visible.

Do not copy an archived stage back into the active level. If a historical
claim must be checked, read it in place and cite the dated archive manifest.
