# Step 8: repaired clean-reference freeze

**Date:** 8 August 2026  
**Status:** PASS  
**Morphologies:** $N\in\{4,6,8,10\}$ bilateral segments  
**Terrain:** flat, body--ground collision enabled  
**Yaw half-amplitude:** $15^\circ$  
**Evaluation horizon:** four gait periods

## 1. Registered reference

For morphology $N$, the clean task output is the displacement of the
`robot_root` subtree center of mass along fixed model-forward $+y$:

\[
D_0(N)=
\left[
\mathbf p_{\mathrm{COM}}(T_{\mathrm{eval}})
-\mathbf p_{\mathrm{COM}}(0)
\right]\cdot\widehat{\mathbf e}_{\mathrm f}.
\]

The first reliability band remains symmetric:

\[
\varepsilon_D(N)=0.10|D_0(N)|,
\qquad
|\widehat D-D_0(N)|\le\varepsilon_D(N).
\]

## 2. Frozen values

| $N$ | Legs | $m=N/2$ | $D_0(N)$ [m] | $\varepsilon_D$ [m] | Frozen interval [m] |
|---:|---:|---:|---:|---:|---:|
| 4 | 8 | 2 | 1.400237616 | 0.140023762 | [1.260213855, 1.540261378] |
| 6 | 12 | 3 | 1.330245604 | 0.133024560 | [1.197221044, 1.463270164] |
| 8 | 16 | 4 | 1.314034401 | 0.131403440 | [1.182630961, 1.445437841] |
| 10 | 20 | 5 | 1.330045987 | 0.133004599 | [1.197041388, 1.463050585] |

Three independent executions per morphology reproduced the complete terminal
state exactly; both replay standard deviation and maximum state delta were
zero. All 12 runs passed finite-state, positive-forward-displacement, body
collision, and warning gates.

Only $N=4$ changed materially from the historical clean freeze
($1.4024879\,\mathrm m\rightarrow1.4002376\,\mathrm m$); the longer models
retained their previous values to the saved precision. The corrected values,
not the historical Step 4 table, are the reference for every later task trial.

## 3. Saved evidence

The run configuration, all numeric trajectories, aggregate JSON/CSV, figure,
and four decoded 189-frame MP4 demos are under
`exp_data/reliability/step8_body_contact_clean_reference/`.
