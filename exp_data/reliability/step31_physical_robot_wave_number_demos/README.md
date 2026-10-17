# Step 31: INVALID spatial pitch parameterization (superseded)

> Do not use these videos as the final physical-robot wave-number demos. The
> per-segment `sin(theta+3*pi/2)` threshold can split a single visible body
> half-wave/cave across different pitch states. The corrected cave-collocated
> controller and replacement videos are in
> `../step32_physical_robot_cave_grouped_wave_demos/`.

This directory contains visualization diagnostics, not a performance sweep.
The controller is bitwise the Step-30 coupling law; only the model is replaced
by the audited center-rooted measured-STL robot.

## Pitch audit

- Up target: -40 deg; down target: 5 deg.
- All 20 physical legs passed the geometry-direction check.
- Minimum up-minus-down leg-mesh clearance: 110.85 mm.
- Mean dynamic contact selectivity `P(contact|down)-P(contact|up)`:
  +0.513.
- Mean pitch tracking error across tested wave numbers: 6.90 deg.
- Mean 95th-percentile pitch tracking error: 39.65 deg.
- Largest pitch-up contact fraction: 0.151
  at `N_wave=0.5`.

The physical XML mirrors the left and right link frames through body
orientations. Identical textual joint-axis entries therefore do not imply
identical world-frame rotations. Direct MuJoCo kinematics confirms that
`-40 deg` raises and `+5 deg` lowers both sides.

The remaining pitch issue is temporal: the binary target changes by 45 deg at
each zero crossing, while the torque-limited physical joint cannot change
angle instantaneously. The large transition-tail tracking error is therefore
real. It must not be confused with a reversed pitch convention. Smoothing or
rate-limiting this transition would define a different controller and is not
applied in these demos.

## Demos

Each video compares canonical pitch timing with an exact left/right pitch
exchange at one fixed `N_wave`. Green denotes commanded pitch-down, red
commanded pitch-up, and yellow measured ground contact. The videos use global phase
0 deg, four warm-up cycles, and the same 0.4-rad body-yaw half-amplitude as
Step 30.

See `demos/demo_manifest.csv`, `pitch_kinematic_audit.json`,
`pitch_dynamic_audit.csv`, and `figures/pitch_geometry_and_tracking_audit.png`.
