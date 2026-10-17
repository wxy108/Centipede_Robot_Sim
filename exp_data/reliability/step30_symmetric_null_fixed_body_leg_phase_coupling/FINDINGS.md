# Step 30: fixed body-leg phase coupling and contact-phase flip

## Definition

The robot length is fixed. `N_wave` is the absolute number of complete 
spatial waves over the head-to-tail segment span. This is migrated from 
the live Centipede_IsaacLab_RL definition. For N segment-phase locations, 
`phi_body=-2*pi*N_wave/(N-1)`. The negative sign retains the established 
MuJoCo propagation direction. 
For each segment phase `theta_i`, the leg-yaw waveform is 
`sin(theta_i+pi/2)`. The leg-pitch precursor is 
`sin(theta_i+3*pi/2)` and is converted to the robot's binary target 
at zero using the robot convention: positive is up and nonpositive 
is down. The left precursor 
has an additional `pi` offset. In the flipped condition only the 
left/right leg-pitch commands are exchanged; body and leg yaw are 
bitwise unchanged.

## Protocol

- Symmetric center-rooted null with N=10 segments.
- N_wave values: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0.
- Body-yaw half-amplitude: 0.400 rad (22.92 deg), migrated from the live RL YAML.
- Leg-yaw/body-yaw phase offset: pi/2 (fixed; independent of N_wave).
- Leg-pitch precursor/body-yaw phase offset: 3*pi/2 (fixed), then zero-crossing quantization.
- Global CPG phases: 0, 45, 90, 135, 180, 225, 270, 315 deg.
- 4 unmeasured warm-up cycles and 12 evaluated cycles.
- Whiskers in the displacement figure are phase-ensemble minima/maxima, not IID confidence intervals.

## Results

| N_wave | Canonical mean (mm) | Canonical phase range (mm) | Flipped mean (mm) | Flipped phase range (mm) | Reverse/forward magnitude | Canonical slip (mm/s) | Flipped slip (mm/s) |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | -2062.20 | [-2517.93, -1786.97] | +1135.44 | [+1115.69, +1156.84] | 0.551 | 183.86 | 124.36 |
| 1.0 | +396.48 | [+390.62, +401.87] | -253.56 | [-278.69, -229.17] | 0.640 | 172.39 | 93.25 |
| 1.5 | +2618.52 | [+2614.77, +2620.83] | -1075.87 | [-1081.62, -1064.29] | 0.411 | 131.07 | 172.43 |
| 2.0 | +2118.12 | [+2116.61, +2120.05] | -1773.97 | [-1777.00, -1769.72] | 0.838 | 91.65 | 129.23 |
| 2.5 | +2016.12 | [+2013.77, +2021.37] | -1890.09 | [-1892.40, -1888.18] | 0.937 | 73.39 | 89.08 |
| 3.0 | +2012.26 | [+2009.48, +2014.66] | -1774.72 | [-1778.31, -1772.80] | 0.882 | 74.83 | 67.88 |
| 3.5 | +1921.68 | [+1919.30, +1922.69] | -1659.90 | [-1660.93, -1657.81] | 0.864 | 62.67 | 51.19 |
| 4.0 | +2005.90 | [+2004.66, +2007.25] | -1508.24 | [-1509.10, -1506.67] | 0.752 | 59.30 | 51.86 |

## Interpretation

The canonical mean is positive at 7/8 tested absolute wave numbers. 
The flipped mean is negative at 7/8 wave numbers.

The largest canonical forward mean occurs at N_wave=1.5 (+2618.52 mm). The strongest flipped reverse mean occurs at N_wave=2.5 (-1890.09 mm).

Across matched phase/wave-number pairs, flipping changes mean persistent-contact slip speed by -8.74 mm/s on average.

A pitch-phase flip is not a full mechanical time reversal: shoulder yaw, body shape, impact history, 
and frictional state are not reversed. Therefore unequal forward/reverse magnitudes are expected and 
must be interpreted together with measured slip rather than as a violation of symmetry.

Wave numbers where the flipped reverse magnitude is smaller and flipped slip is larger: 1.5, 2, 2.5. This criterion is evaluated from the data rather than assumed.

## Key artifacts

- `wave_contact_flip_summary.csv`
- `wave_contact_flip_trials.csv`
- `paired_phase_comparison.csv`
- `directionality_comparison.csv`
- `control_contract.json`
- `COUPLING_AND_WAVE_NUMBER_DEFINITION.md`
- `figures/`
- `demos/`
