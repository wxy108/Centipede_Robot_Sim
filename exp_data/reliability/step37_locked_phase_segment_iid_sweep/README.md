# Step 37: locked phase-offset × segment-count IID sweep

`N_wave_ref10` labels a phase increment defined on the 10-segment reference robot:
`Delta_phi=-2*pi*N_wave_ref10/9`. The same adjacent-joint phase increment is
held fixed for N=5..10, so `N_wave_eff=N_wave_ref10*(N-1)/9`.

Each of the 42 conditions contains 128 independent trials.
The shared body-yaw half-amplitude is 19 deg. Leg yaw is fixed.
Body-yaw PD uses the audited `moderate` runtime setting; source XML files are unchanged.

IID means independent Bernoulli(0.5) pitch-down/up commands per physical leg and
half-cycle. Actual contact is measured and is not directly switched off.

Primary outputs:

- `DEMO_INDEX.md`: one compact index for all videos, SD figures, and reference data.
- `iid_trials.csv`: raw trial outcomes.
- `iid_summary.csv`: center and spread per phase/segment condition.
- `dispersion_trends.csv`: predeclared IQR/SD trend tests.
- `phase_endpoint_comparison.csv`: direct N=5 versus N=10 comparison.
- `figures/iid_histograms_locked_phase_*.png`: one histogram panel per locked phase.
- `demos/*.mp4`: median-representative N=10 replay for every locked phase.
- `demo_manifest.csv` and `demo_validation.json`: demo provenance and replay/media checks.
- `small_N_demo_manifest.csv` and `small_N_demo_validation.json`: N=5 and N=6 IID demos.
- `sd_by_phase_and_segment.csv`: sample SD and bootstrap intervals for each condition.
- `sd_dispersion_trends.csv`: SD-versus-N slope and bootstrap interval for each locked phase.
- `figures/iid_SD_vs_segment_*.png`: one SD plot per phase offset.
- `all_pitch_down_reference/`: separate deterministic N=10 all-down reference, figures, and demos.
- `validation.json`: completion gates.
