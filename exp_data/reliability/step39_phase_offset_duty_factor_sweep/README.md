# Step 39: phase offset x commanded duty factor

Physical N=10 centipede robot. This is a 7 x 9 deterministic grid, not an ensemble estimate and not an IID/contact-erasure experiment.

## Controlled definition

The ten-segment reference label defines $\Delta\phi=-2\pi N_{wave}^{(10)}/9$. The sweep uses $N_{wave}^{(10)}=1,1.5,\ldots,4$, hence $\Delta\phi=-40^\circ,-60^\circ,\ldots,-160^\circ$.

For segment $i$, $\theta_i(t)=\omega t+(i-1/2)\Delta\phi$. Right and left carriers are $\sin\theta_i$ and $-\sin\theta_i$. A down command is selected when $u_{is}\geq\cos(\pi d)$, so each window remains centered at its original peak/cave and has duration $dT_g$.

At every phase offset, d=0.5 is checked against the corresponding corrected canonical contact controller. Body-yaw motion uses the same phase increment as the leg-pitch carrier. Leg yaw remains fixed.

Fixed quantities: body-yaw half-amplitude 19 deg, omega=4 rad/s, moderate PD, 4 warm-up cycles, 12 evaluated cycles, unchanged source XML, no randomization and no contact erasure.

Commanded down duty and detected ground contact are different quantities. Contact is an observed foot/leg-to-world collision-pair fraction; it is neither forced by the command nor thresholded by normal force.

## Primary evidence

- `trials.csv`: all 63 absolute signed-displacement outcomes and diagnostics.
- `time_series.csv`: all evaluated COM trajectories.
- `per_leg_contact_summary.csv`: per-leg command, contact, and tracking summaries.
- `figures/displacement_phase_offset_x_duty_heatmap.png`: primary two-dimensional result.
- `figures/displacement_vs_duty_all_phase_offsets.png`: one duty curve per phase offset.
- `figures/detected_contact_phase_offset_x_duty_heatmap.png`: command/contact interpretation.
- `figures/body_tilt_phase_offset_x_duty_heatmap.png`: stability diagnostic, not an objective.
- `controller_validation.json`, `validation.json`, and `run_configuration.json`: audit and provenance.
- `media_header_validation.csv`: all 21 corrected phase labels and readable-frame checks.

## Same-rollout demonstrations

Videos are saved at d=0.2, 0.5, 0.8 for every phase offset. Each video and its reported metrics come from the same rollout.

| Reference wave label | Phase increment | Duty | Displacement (m) | Max tilt (deg) | Status | Video |
|---:|---:|---:|---:|---:|---|---|
| 1 | -40 deg | 0.2 | +1.8695 | 2.84 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_100_duty_020.mp4) |
| 1 | -40 deg | 0.5 | +4.3820 | 15.04 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_100_duty_050.mp4) |
| 1 | -40 deg | 0.8 | +3.1522 | 1.47 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_100_duty_080.mp4) |
| 1.5 | -60 deg | 0.2 | +2.0584 | 1.96 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_150_duty_020.mp4) |
| 1.5 | -60 deg | 0.5 | +3.3123 | 5.80 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_150_duty_050.mp4) |
| 1.5 | -60 deg | 0.8 | +2.7233 | 1.41 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_150_duty_080.mp4) |
| 2 | -80 deg | 0.2 | +1.5527 | 4.21 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_200_duty_020.mp4) |
| 2 | -80 deg | 0.5 | +2.7262 | 3.61 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_200_duty_050.mp4) |
| 2 | -80 deg | 0.8 | +2.0518 | 1.04 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_200_duty_080.mp4) |
| 2.5 | -100 deg | 0.2 | +1.2749 | 3.58 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_250_duty_020.mp4) |
| 2.5 | -100 deg | 0.5 | +2.2555 | 2.81 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_250_duty_050.mp4) |
| 2.5 | -100 deg | 0.8 | +1.6955 | 0.89 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_250_duty_080.mp4) |
| 3 | -120 deg | 0.2 | +1.7920 | 9.68 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_300_duty_020.mp4) |
| 3 | -120 deg | 0.5 | +2.0983 | 4.75 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_300_duty_050.mp4) |
| 3 | -120 deg | 0.8 | +1.4437 | 0.69 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_300_duty_080.mp4) |
| 3.5 | -140 deg | 0.2 | +0.6913 | 3.17 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_350_duty_020.mp4) |
| 3.5 | -140 deg | 0.5 | +1.7304 | 2.14 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_350_duty_050.mp4) |
| 3.5 | -140 deg | 0.8 | +1.3602 | 0.75 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_350_duty_080.mp4) |
| 4 | -160 deg | 0.2 | +0.6591 | 4.35 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_400_duty_020.mp4) |
| 4 | -160 deg | 0.5 | +1.6844 | 4.25 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_400_duty_050.mp4) |
| 4 | -160 deg | 0.8 | +1.4517 | 1.03 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step39_phase_offset_duty_factor_sweep/demos/N10_NwaveRef10_400_duty_080.mp4) |

## Direct observations

All 63 deterministic conditions passed the numerical and media gates. The largest scanned displacement was 5.0092 m at reference wave label 1 and duty 0.4, but its maximum sampled tilt was 10.11 deg, so this cell is not declared a robust optimum.

For reference wave labels 1 through 3.5, the largest displacement among the nine sampled duty values occurs at d=0.4. At wave label 4, d=0.3 is only about 0.006 m above d=0.4. This is a grid observation, not a continuous optimum.

The reference-wave-2 row exactly reproduces the Step-38 displacement values. High displacement and low wobble occupy different parts of the map.

## Full grid

The table below is diagnostic. The largest displacement in a row is not automatically a robust or optimal gait; consult tilt, tracking, contact, and repeated-trial evidence before making that claim.

| Wave label | Phase | d=0.1 | d=0.2 | d=0.3 | d=0.4 | d=0.5 | d=0.6 | d=0.7 | d=0.8 | d=0.9 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | -40 deg | +0.891 | +1.870 | +3.665 | +5.009 | +4.382 | +3.652 | +3.273 | +3.152 | +2.860 |
| 1.5 | -60 deg | +0.789 | +2.058 | +3.588 | +3.819 | +3.312 | +3.195 | +2.966 | +2.723 | +2.438 |
| 2 | -80 deg | +0.462 | +1.553 | +2.828 | +2.964 | +2.726 | +2.436 | +2.221 | +2.052 | +1.835 |
| 2.5 | -100 deg | +0.232 | +1.275 | +2.483 | +2.644 | +2.255 | +2.068 | +1.876 | +1.696 | +1.444 |
| 3 | -120 deg | +0.772 | +1.792 | +2.150 | +2.186 | +2.098 | +1.925 | +1.744 | +1.444 | +1.139 |
| 3.5 | -140 deg | +0.008 | +0.691 | +1.923 | +2.043 | +1.730 | +1.660 | +1.544 | +1.360 | +1.157 |
| 4 | -160 deg | +0.057 | +0.659 | +1.837 | +1.831 | +1.684 | +1.541 | +1.502 | +1.452 | +1.297 |
