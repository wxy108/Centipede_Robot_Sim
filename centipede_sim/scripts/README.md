# Active compatibility helper

Only `test_models.py` remains here because the current reliability pipeline
imports its actuator-index and control-command helpers. It is a library
dependency despite its historical filename.

The former model, terrain, plotting, and sweep scripts are frozen under
`../../_archive/frozen_2026-08-31/legacy_sim_shell/centipede_sim/scripts/`.
Do not restore or extend those scripts for new reliability experiments.

