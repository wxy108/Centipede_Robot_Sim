# Step 30 controller definition (superseded for cave-grouped pitch)

> Historical result only. Its wave-number definition remains useful, but its
> per-segment pitch threshold is not spatially collocated with the body-joint
> samples and does not guarantee a uniform stance/swing assignment within one
> body half-wave/cave. Do not transfer its pitch semantics to the physical
> robot. See Step 32 for the corrected controller.

## 1. Spatial wave number

For ten segment-indexed phase locations, define

```text
theta_i(t) = omega*t + phi_0 + i*Delta_phi,    i=0,...,9
Delta_phi  = -2*pi*N_wave/(10-1).
```

Therefore `theta_9-theta_0=-2*pi*N_wave`: `N_wave` is the absolute
number of complete spatial periods over the first-to-last segment span. The
minus sign selects propagation direction. This is the convention migrated from
the live Centipede_IsaacLab_RL controller; it is not the invalid Step 28
inverse-wavelength definition `-2*pi/N_wave`.

## 2. Fixed body-leg phase coupling

The normalized body-yaw wave at segment phase `theta_i` is

```text
b_i(t) = sin(theta_i).
```

The oscillatory part of leg yaw is fixed at `+pi/2` relative to body yaw:

```text
y_i(t) = sin(theta_i + pi/2).
```

This offset is independent of `N_wave`. No half-spatial-increment correction
or shared `bodyleg_coupling` factor is used.

The right leg-pitch continuous precursor is fixed at `+3*pi/2`:

```text
p_i,R(t) = sin(theta_i + 3*pi/2).
```

The left side is in antiphase:

```text
p_i,L(t) = sin(theta_i + 3*pi/2 + pi) = -p_i,R(t).
```

## 3. Zero-crossing conversion to robot pitch steps

The physical robot controller does not send the continuous pitch precursor to
the pitch joint. It applies the established `leg_angle` sign convention:

```text
q_pitch(i,s) = q_up    if p_i,s > 0
               q_down  if p_i,s <= 0.
```

Here `q_up=-40 deg` and `q_down=+5 deg`. At an exact zero crossing the target
is defined as down, matching the existing robot helper. Exact simultaneous
left/right zeros are isolated transition instants; away from them the two
sides are complementary.

The canonical condition uses these targets directly. The flipped condition
exchanges only the final right/left binary pitch targets. Body yaw and leg yaw
remain bitwise unchanged.

## 4. Fixed amplitudes and tested grid

```text
body-yaw half-amplitude = 0.4 rad = 22.918 deg
N_wave = 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0
```

The production experiment contains eight global initial phases for each wave
number and each contact pattern: `8*8*2=128` trials. All 128 passed the
stability gates. The control contract independently checked 584 canonical/flip
time pairs and 5,840 segment-level coupling samples.

## 5. Output convention

The primary comparison figure uses `abs(D_y)` so forward and reverse movement
are compared on one positive axis. Raw signed displacement is retained in
`wave_contact_flip_trials.csv` and must be used for direction claims.

The lowest tested wave number is a direction-reversal boundary: at
`N_wave=0.5`, canonical motion is negative and the pitch-flipped condition is
positive. It is retained as a real result, not relabeled or omitted.
