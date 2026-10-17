# Step 2 Historical Performance Audit, Figures, and Demo Videos

**Status:** diagnostic analysis complete  
**Date:** 2026-08-06  
**Controlled conditions:** flat terrain, \(q=0.10\), four gait cycles, yaw half-amplitude \(15^\circ\)  
**Sample:** 8 seeds per channel/model condition, 32 MuJoCo trials total

> Canonical notation update: \(G\) is the phase-group count, \(m\) is copy multiplicity, and \(\mathrm L/\mathrm R\) mean left/right only. This report describes the historical controller-level stance-omission mechanism, not the new exact contact-erasure experiment.

## 1. Measured diagnostics

For observed planar displacement \(\mathbf d\) and paired clean displacement \(\mathbf d_0\), define the historical axial-progress ratio

\[
\eta_D=\frac{\mathbf d^{\mathsf T}\mathbf d_0}{{\lVert\mathbf d_0\rVert}^2}.
\]

The perpendicular and endpoint errors are

\[
e_\perp
=
\frac{|d_xd_{0,y}-d_yd_{0,x}|}{\lVert\mathbf d_0\rVert},
\qquad
e_{\mathrm{end}}=\lVert\mathbf d-\mathbf d_0\rVert.
\]

The height diagnostic is

\[
\eta_h=\frac{\min_t z(t)}{z(0)},
\qquad
C_{\mathrm{collapse}}=\mathbf 1\{\eta_h<0.5\}.
\]

The historical actuator-space work diagnostics are

\[
W_{\mathrm{abs}}
=
\int_0^{T_{\mathrm{eval}}}\sum_a|f_a(t)v_a(t)|\,dt,
\]

and

\[
W_+
=
\int_0^{T_{\mathrm{eval}}}\sum_a[f_a(t)v_a(t)]_+\,dt.
\]

The interval excludes the 1.5-second startup ramp. These are mechanical work proxies, not an electrical battery-loss model.

Task reliability was not computed because no displacement tolerance or sample-size rule was preregistered.

## 2. Descriptive results

| Channel | \(N\) | \(m\) | Axial progress \(\eta_D\), mean \(\pm\) SD | Mean lateral error (m) | Mean endpoint error (m) | Collapses | Mean absolute-work ratio | Effective omission conversion |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Independent | 6 | 3 | \(0.9833\pm0.0247\) | 0.0639 | 0.0813 | 4/8 | 0.9447 | 0.8929 |
| Independent | 8 | 4 | \(1.0275\pm0.0220\) | 0.0414 | 0.0649 | 4/8 | 0.9367 | 0.9474 |
| Perfectly correlated | 6 | 3 | \(0.9343\pm0.0898\) | 0.0594 | 0.1088 | 4/8 | 0.9556 | 1.0000 |
| Perfectly correlated | 8 | 4 | \(0.9520\pm0.0613\) | 0.0370 | 0.0778 | 4/8 | 0.9565 | 1.0000 |

Evidence-supported descriptive observations:

- In this finite batch, independent masks preserved axial progress more closely and with less seed-to-seed variation than perfectly correlated masks for both morphologies.
- Increasing \(N=6\) to \(N=8\), equivalently \(m=3\) to \(m=4\) at fixed \(G=2\), reduced mean endpoint error in both channel modes.
- The collapse diagnostic was not discriminative: every condition had 4 collapses in 8 trials.
- Work below the clean reference is expected when stance commands are omitted or the robot collapses. It is not evidence of improved efficiency.

These are not a statistical ranking. Eight seeds per condition are insufficient for a reliability-law estimate.

## 3. Relation to the channel law

With

\[
F_e=\prod_{j=1}^{m}Z_{e,j},
\]

the high-sample mask test established

\[
\Pr(F_e=1)=q^m
\quad\text{for independent copies},
\qquad
\Pr(F_e=1)=q
\quad\text{for perfectly correlated copies}.
\]

The physical batch showed a qualitatively consistent signature, but the link remains provisional because it contained only 128 group events per condition and no preregistered task-success set.

## 4. Figures and numerical outputs

Outputs are stored in:

`exp_data/reliability/step2_channels/performance/`

- `fig01_group_erasure_scaling.png`: theoretical and empirical \(q^m\) versus \(q\);
- `fig02_axial_progress_ratio.png`: all historical \(\eta_D\) values and condition means;
- `fig03_min_height_ratio.png`: all historical \(\eta_h\) values and the 0.5 diagnostic threshold;
- `fig04_work_and_contact_conversion.png`: work response and physical omission conversion;
- `fig05_erasure_vs_endpoint_error.png`: exploratory omission/error relationship;
- `performance_summary.json` and `performance_trials.csv`: complete numerical tables.

The figures and derived numerical tables were regenerated with \(m\). Immutable per-trial raw files may retain the legacy key `R`; current readers map it explicitly to \(m\).

## 5. Demo videos

Each MP4 displays the clean reference on the left and the historical stance-omission trajectory on the right. A red border marks an active commanded omission.

Eight videos are stored under:

`exp_data/reliability/step2_channels/performance/videos/`

| Video | Role |
|---|---|
| `N6_independent_seed000101.mp4` | Independent, \(m=3\), high-omission collapsed example |
| `N6_correlated_seed000101.mp4` | Perfectly correlated, \(m=3\), complete-group omission example |
| `N8_independent_seed000101.mp4` | Independent, \(m=4\), collapsed example |
| `N8_correlated_seed000101.mp4` | Perfectly correlated, \(m=4\), complete-group omission example |
| `N6_independent_seed000202.mp4` | Independent stable example |
| `N8_independent_seed000202.mp4` | Independent stable example |
| `N6_correlated_seed000404.mp4` | Low-load correlated stable example |
| `N8_correlated_seed000606.mp4` | Correlated multi-omission collapsed example |

Every video has 190 frames at 30 fps and lasts approximately 6.33 seconds. The videos are illustrative only; the table above uses all 32 trials.

## 6. Superseding experimental gate

The earlier multi-metric success-set proposal is superseded by a simpler fixed-duration definition. For the new exact contact-erasure experiment,

\[
\widehat D=
[\mathbf p_{\mathrm{COM}}(T_{\mathrm{eval}})-\mathbf p_{\mathrm{COM}}(0)]
\cdot\widehat{\mathbf e}_{\mathrm f},
\]

and

\[
\mathcal R_D(q,N)
=
\Pr\!\left(|\widehat D-D_0(N)|\leq\varepsilon_D\right).
\]

For \(n_{\mathrm{trial}}\) held-out trials,

\[
\widehat{\mathcal R}_D
=
\frac{1}{n_{\mathrm{trial}}}
\sum_{a=1}^{n_{\mathrm{trial}}}
\mathbf 1\!\left\{
|\widehat D_a-D_0(N)|\leq\varepsilon_D
\right\}.
\]

The tolerance \(\varepsilon_D\), uncertainty interval, and stopping rule must be fixed before the final stochastic batch. Collapse, lateral motion, body contact, and work remain reported diagnostics but do not replace the displacement criterion.
