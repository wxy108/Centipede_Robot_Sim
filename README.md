# Centipede Robot — reliability experiments

Current local canonical workspace synchronized on 2026-10-17. This supersedes the old April simulation tree; the old layout and full history remain available through Git and the pre-sync tag.

## Start here

- [Current code and entry points](centipede_sim/reliability/README.md).
- [Evidence index](exp_data/reliability/README.md).
- [Step40 final report](exp_data/reliability/step40_high_frequency_iid_contact_erasure_sweep/FINAL_REPORT.md) and [validation receipt](exp_data/reliability/step40_high_frequency_iid_contact_erasure_sweep/validation.json).
- [Step40 trial table](exp_data/reliability/step40_high_frequency_iid_contact_erasure_sweep/iid_contact_erasure_trials.csv).
- [Directional-bias diagnosis](exp_data/reliability/step27_directional_bias_diagnosis/REPORT.md).

The main evidence is a center-rooted measured-STL physical robot and its controlled symmetric null. The current Step40 dataset has **42 conditions × 512 trials = 21,504 trials** at q=0.5, with all individual physical legs commanded down and IID erasure applied to collision channels. Its existing receipt reports all gates passed and zero erased-contact violations; the copied raw CSV row/condition counts were independently checked during synchronization.

Step38/39 duty-factor studies, Step25b/26b matched null/physical baselines and current code are retained with their definitions. Step30 has a superseded pitch mapping; Step31 is explicitly invalid diagnostic evidence. Older Step1–24 material remains in the local frozen archive rather than being revived here. Strong historical displacement trends were partly affected by representation/root bias; the existing causal report must accompany interpretation.

The latest histogram plotting source contains the corrected shared-axis/bin logic. Large time-series tables, raw NPZ trajectories and videos remain local, while compact tables, contracts and receipts are preserved. See `LOCAL_SOURCE_MANIFEST.json` for source paths, hashes and omitted large files. This update did not rerun trials, rewrite raw observations or change source model/controller parameters.

Run locations and dependencies are documented in the current code README. Completed sweeps must not be rerun just because an old runner is present. The original local workspace README is preserved under `provenance/LOCAL_WORKSPACE_README.md`.
