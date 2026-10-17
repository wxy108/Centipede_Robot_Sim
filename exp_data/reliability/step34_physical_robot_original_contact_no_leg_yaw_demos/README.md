# Step 34: original contact pattern, no leg-yaw oscillation

These eight deterministic videos complement Step 33's IID videos. The same
center-rooted, measured-STL, ten-segment physical robot and the same 15-degree
body-yaw half-amplitude are retained. Leg yaw has fixed neutral servo targets
(R=-5 deg, L=+5 deg); pitch is +5 deg down / -40 deg up. No model, gain,
armature, solver, collision or friction parameters are changed.

Only the pitch law changes: restore the corrected Step-32 cave grouping.
The body-joint phase is omega*t+j*Delta_phi; the segment-center phase is
omega*t+(i-1/2)*Delta_phi. Each half-wave/cave receives one identical side-down
choice; the other side is complementary. The pattern is periodic, not random,
and is not artificially turning ground contacts off. Step 32's 0.4-rad body
amplitude and oscillating leg yaw are NOT reintroduced.

Initialization uses the same stages as Step 33: straight all-down settling,
cosine bridge, then four warm-up cycles, now with the deterministic controller.
Each video shows 6 seconds after warm-up at 30 fps. Green = commanded down;
red = commanded up. Commands follow the target body-wave phase; actuator lag
means actual contact and observed instantaneous body curvature need not agree
perfectly with those target states. Measured tracking errors are retained.

`demo_manifest.csv` records each six-second deterministic displacement and
tracking/stability diagnostics. These are not ensemble means and must not be
compared directly with the 18.85-second IID ensemble statistics. `demos/`
contains MP4s and previews; `controller_validation.json` checks grouping and
periodicity, including zero-crossing boundaries; `validation.json` checks
numerical stability and video decoding. Existing Step-32/33 files are untouched.
