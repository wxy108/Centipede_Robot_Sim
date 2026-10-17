# Frozen clean locomotion references

**Date:** 7 August 2026  
**Models:** `centipede_4seg.xml`, `centipede_6seg.xml`, `centipede_8seg.xml`, `centipede_10seg.xml`  
**Output root:** `exp_data/reliability/step4_clean_reference/`  
**Terrain:** flat  
**Yaw half-amplitude:** \(15^\circ\)  
**Channel:** \(Z\equiv0\) through the exact-contact latch

## Fixed protocol

The post-ramp evaluation duration is four gait periods:

\[
T_{\mathrm{eval}}=4T_{\mathrm{gait}}.
\]

With MuJoCo timestep \(\Delta t=0.002\,\mathrm s\), the requested duration is
\(6.283185\,\mathrm s\) and the realized fixed-step duration is
\(6.284\,\mathrm s\). The forward direction is fixed model \(+y\).

The canonical position is the whole-robot center of mass returned by the
`robot_root` MuJoCo subtree. Therefore

\[
D_0(N)
=
\left[
\mathbf p_{\mathrm{COM}}(T_{\mathrm{eval}})
-
\mathbf p_{\mathrm{COM}}(0)
\right]\cdot\widehat{\mathbf e}_{\mathrm f}.
\]

The free-joint root displacement is retained only as a compatibility
diagnostic; it is not the canonical \(D_0\).

## Frozen values

| Morphology | \(G\) | \(m\) | \(D_0\), COM | Root displacement | \(\varepsilon_D=0.1|D_0|\) | Accepted interval |
|---|---:|---:|---:|---:|---:|---:|
| \(N=4\) | 2 | 2 | 1.4024879 m | 1.3970537 m | 0.1402488 m | [1.2622391, 1.5427367] m |
| \(N=6\) | 2 | 3 | 1.3302456 m | 1.3272484 m | 0.1330246 m | [1.1972210, 1.4632702] m |
| \(N=8\) | 2 | 4 | 1.3140344 m | 1.3116910 m | 0.1314034 m | [1.1826310, 1.4454378] m |
| \(N=10\) | 2 | 5 | 1.3300460 m | 1.3283882 m | 0.1330046 m | [1.1970414, 1.4630506] m |

The first-pilot displacement reliability is preregistered as

\[
\mathcal R_D(q,N)
=
\Pr\!\left(
|\widehat D-D_0(N)|
\le 0.10|D_0(N)|
\right).
\]

## Verification

Each morphology was run three times. All three full-state trajectories overlap
exactly:

\[
\max_{r,r'}\max_t
\left\|
\mathbf x_r(t)-\mathbf x_{r'}(t)
\right\|_\infty=0.
\]

For every replay:

- all sampled masks were zero;
- the latch produced no collision-state change;
- all collision bits remained at their original values;
- the applied and nominal control vectors were identical;
- the exact-latch and bare-reference model states were identical;
- all states remained finite and MuJoCo emitted no warning;
- neither morphology collapsed.

The mean post-ramp absolute actuator-space mechanical work was
\(61.7649\), \(89.0223\), \(116.6919\), and \(144.1498\,\mathrm J\) for
\(N=4,6,8,10\), respectively.
These are servo mechanical-work diagnostics, not battery-energy estimates and
not part of the current displacement reliability gate.

## Channel-policy decision

Immediate exact collision restoration is included in the base binary
erasure-channel dynamics. The registered recontact impulse is therefore part
of the subsequent task response. Guarded restoration is excluded from the
base experiment and may be introduced only as a separately named sensitivity
model.

The clean-reference package passed and records
`ready_for_stochastic_mask_validation=true`.

## Saved artifacts

- `clean_reference_summary.json` and `clean_reference_table.csv`;
- `run_configuration.json`;
- three summaries and compressed trajectories for each morphology;
- `figures/clean_reference_N4_N6_N8_N10.png`;
- paired bare-versus-exact-latch MP4 demos for \(N=4,6,8,10\).
