# Step 27 provenance re-audit — 2026-08-31

The Step 27 source-artifact manifest was rechecked after the workspace cleanup.
Archived Step 21--24 paths were updated to their frozen locations without
changing the recorded report-era hashes.

## Result

- 23 of 24 recorded artifacts remain byte-identical to the Step 27 manifest.
- The only mismatch is
  `centipede_sim/reliability/run_center_rooted_dynamic_null_iid.py`.
- Step 27 recorded SHA-256:
  `34650ca7cd15303b15b658f89087a7b635a8789ffbbbe77d7e3a89f327cc9492`.
- Current active-file SHA-256:
  `cabb0f9d1bd3977c5ed3a8fd6d7e929c70c2cf51f7318f9ab9494745e131da50`.

The current runner was modified after the report was frozen. The historical
report, numerical tables, and archived evidence remain unchanged, but this
single live runner is not a byte-exact reconstruction source for Step 27.
Do not overwrite the report-era hash or claim full source-level
reproducibility unless the exact historical runner is recovered.

