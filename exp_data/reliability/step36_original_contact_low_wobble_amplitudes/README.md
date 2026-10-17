# Low-wobble original-contact amplitudes (Step 36)

This stage keeps the Step-35 physical N=10 model, original cave-grouped pitch, fixed leg-yaw targets, omega=4 rad/s, initialization, flat ground and 18.85-second COM-displacement objective. Only body-yaw half-amplitude is selected. Step 35 remains unchanged.

Wobble is not a scalar reward. At 50 Hz we separately retain: detrended 5–95% lateral COM sway, detrended 5–95% vertical COM heave, RMS material-frame chassis tilt relative to the upright reference, detrended whole-body heading-rate RMS, and actual body-yaw-joint RMS. Cubic detrending removes slow turning/drift while preserving the 12 gait-cycle oscillation.

Candidate amplitudes begin at 15, 20, 25, 30, 35 and 40 deg; the interval immediately above each coarse winner is then refined at 1-deg spacing. For each diagnostic and wave, the limit is W15 + 0.60*max(0,W40-W15), removing at least 40% of positive excess wobble. Among candidates passing every limit, the largest forward displacement is selected. This is an explicit engineering trade-off, not a universal perceptual threshold or continuous optimum.

| N_wave | Selected A (deg) | D (m) | Performance retained vs 40 deg (%) |
|---|---|---|---|
| 0.5 | 31 | 1.2607 | 55.5 |
| 1 | 19 | 4.0651 | 54.6 |
| 1.5 | 29 | 4.8821 | 77.3 |
| 2 | 30 | 4.0181 | 79.0 |
| 2.5 | 28 | 3.2529 | 70.8 |
| 3 | 19 | 2.0405 | 50.0 |
| 3.5 | 21 | 1.8963 | 51.7 |
| 4 | 30 | 2.5743 | 74.8 |

All selected runs must also pass Step-35 numerical/upright gates. Model SHA-256: `765343c6b830bfaaf920021244c99023086db4102ee5d7e7b16325930ea81e93`. Commands marked down/up are not direct contact-state measurements. Self-collision remains disabled in this unchanged model.

Files: wobble_trials.csv/json; selected_amplitudes.csv; wobble_limits.csv; results.json; validation.json; figures/; demos/.
