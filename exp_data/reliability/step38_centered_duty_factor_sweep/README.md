# Step 38: peak-centered duty factor, physical centipede robot

N=10; reference wave number 2; spatial increment -80 deg. No IID, no contact erasure, no leg-yaw oscillation. The source XML is unchanged.

## Phase and duty contract

For segment index $i=0,\ldots,9$, $\theta_i(t)=\omega t+(i-1/2)\Delta\phi$, with $\Delta\phi=-4\pi/9$ and $\omega=4\,\mathrm{rad/s}$. Body-yaw joint $j$ has phase $\omega t+j\Delta\phi$.

The unchanged stance carriers are $u_{iR}=\sin\theta_i$ and $u_{iL}=-\sin\theta_i$. Their peaks are separated by half a period. A stance/down command is selected when $u_{is}\geq\cos(\pi d)$, using deterministic half-open opening/closing ties in code. Each window has phase width $2\pi d$ and time duration $dT_g$, where $T_g=2\pi/\omega$.

The numerical pitch targets remain +5 deg (down) and -40 deg (up). The sine is a phase carrier, not a replacement continuous motor command. No time warping or movement of the stance-window centers is introduced. At d=0.5 the corrected canonical pitch commands are recovered, including zero-crossing ties.

A blanket leg-pitch/body-yaw offset of -pi is not the current convention: at the collocated segment center, right-side down follows positive body curvature; left-side down follows negative curvature. Choosing lift rather than down as the positive carrier would reverse that sign. Segment-center and inter-segment-joint phases differ by the required half spatial increment.

For d<0.5, each pair has a both-up gap of fraction 1-2d per cycle. For d>0.5, both-down overlap occupies fraction 2d-1. Both sides still have the same duty and retain their original, half-cycle-separated centers.

## Controlled simulation

Body-yaw half-amplitude 19 deg, Step-37 moderate PD, fixed leg yaw (R=-5 deg, L=+5 deg), fixed body pitch. Common settling ramp 1.5 s, cosine bridge 0.4 s into each condition's initial targets, 4 warm-up cycles, 12 evaluated cycles. Each condition is one deterministic run, not an ensemble mean.

Command duty is measured at every simulation step. Detected ground contact is sampled at 50 Hz and uses the existing foot/leg-to-world contact-pair definition; it is not an enforced support fraction or a force-threshold criterion. MuJoCo contact geometry, actuation lag, and slip can make it differ from commanded duty. Stability checks are numerical/sample-based gates, not a proof of physical accuracy.

MuJoCo version: 3.3.7. Videos and metrics use the very same newly simulated trajectory; they are not cross-runtime reruns of an older trial.

## Results and videos

| Duty | Down time per cycle (s) | Displacement (m) | Detected contact fraction | Status | Video |
|---:|---:|---:|---:|---|---|
| 0.1 | 0.157 | +0.4617 | 0.070 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_010.mp4) |
| 0.2 | 0.314 | +1.5527 | 0.161 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_020.mp4) |
| 0.3 | 0.471 | +2.8276 | 0.262 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_030.mp4) |
| 0.4 | 0.628 | +2.9644 | 0.364 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_040.mp4) |
| 0.5 | 0.785 | +2.7262 | 0.337 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_050.mp4) |
| 0.6 | 0.942 | +2.4359 | 0.370 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_060.mp4) |
| 0.7 | 1.100 | +2.2213 | 0.408 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_070.mp4) |
| 0.8 | 1.257 | +2.0518 | 0.442 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_080.mp4) |
| 0.9 | 1.414 | +1.8347 | 0.469 | PASS | [MP4](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step38_centered_duty_factor_sweep/demos/N10_Nwave2_duty_090.mp4) |

## Files

- `demos/`: full-evaluation MP4s and previews.
- `figures/centered_stance_windows.png`: carrier and centered R/L command windows.
- `figures/displacement_vs_duty_factor.png`: signed, unnormalized forward COM displacement.
- `figures/commanded_vs_detected_contact.png`: command/contact distinction.
- `figures/displacement_time_series.png`: trajectories.
- `trials.csv`, `time_series.csv`, `per_leg_contact_summary.csv`: compact numerical evidence.
- `controller_validation.json`, `validation.json`, `run_configuration.json`: definition, provenance, and checks.
