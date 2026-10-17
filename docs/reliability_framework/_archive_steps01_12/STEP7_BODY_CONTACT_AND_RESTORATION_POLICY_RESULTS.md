# Step 7: body--ground collision repair and restoration-policy audit

**Date:** 8 August 2026  
**Status:** PASS  
**Scope:** $N=4$, historical worst-event diagnostic  
**Event:** `N4_g2_j2_i4_R_c0_h1`  
**Terrain:** flat  
**Yaw half-amplitude:** $15^\circ$  
**Burst length:** $K=1$

## 1. Audit finding

The historical floor admitted leg collisions but rejected every non-leg body
collision. Its filter was

\[
(c_{\mathrm{floor}},a_{\mathrm{floor}})=(1,4),
\]

whereas non-leg body geoms used collision type $2$. Because MuJoCo accepts a
pair only when

\[
(c_1\mathbin{\&}a_2)\ne0
\quad\text{or}\quad
(c_2\mathbin{\&}a_1)\ne0,
\]

body--floor collision was impossible. This explains why the short $N=4$
model could sink through the plane and rotate unrealistically after loss of
leg support.

The repaired floor uses

\[
(c_{\mathrm{floor}},a_{\mathrm{floor}})=(1,6),
\qquad 6=2\lor4,
\]

so it accepts both body type $2$ and leg type $4$. The same definition is
now generated for $N\in\{4,6,8,10\}$ and used by flat and height-field
terrain.

## 2. Restoration policies compared

All policies erased both physical sides of the selected segment throughout the
registered support window and produced exactly zero bilateral contact impulse.
They differed only at restoration.

1. **Hard:** restore both collision channels at the nominal event offset.
2. **Clearance-gated:** restore the opposite side at the nominal offset; keep
   the formerly supporting side disabled until its collision mesh clears the
   ground by $h_{\min}=2\,\mathrm{mm}$.
3. **Retracted:** override the selected elevation actuator to hold the stance
   leg up, then restore collision at the nominal offset.

The retracted policy is an actuation fault / stance-omission model because

\[
\mathbf u(t)\ne\mathbf u^0(t).
\]

It is therefore not interchangeable with an environmental contact-erasure
channel.

## 3. Quantitative result

| Policy | Loss relative to paired clean | Peak restoration-step impulse | Ratio to clean module peak | Trigger-side delay | Command identity |
|---|---:|---:|---:|---:|---|
| Hard | $1.652\%$ | $0.169995\,\mathrm{N\,s}$ | $2.534$ | $0\,\mathrm s$ | exact |
| Clearance-gated | $2.730\%$ | $0.016629\,\mathrm{N\,s}$ | $0.248$ | $0.432\,\mathrm s$ | exact |
| Retracted | $2.857\%$ | $0\,\mathrm{N\,s}$ | $0$ | $0\,\mathrm s$ | violated by design |

The paired clean module's largest single-step impulse was
$0.067091\,\mathrm{N\,s}$. All three trials remained finite, emitted no
MuJoCo warnings, restored every collision bit, and avoided deep body
penetration.

## 4. Registered realization

The canonical artificial environmental channel is now **clearance-gated
bilateral contact erasure**:

\[
Z_{e,j}=1
\Longrightarrow
\begin{cases}
\text{both sides of }\mathcal M_{i_{g,j}}\text{ are collision-off on }\mathcal W_e,\\
\mathbf u(t)=\mathbf u^0(t),\\
\text{restoration is permitted only after the corresponding geom is clear.}
\end{cases}
\]

The nominal binary erasure duration remains $K=1$. Clearance gating is a
numerical/physical restoration guard, not an extension of the sampled channel
symbol. Task metrics are frozen at $T_{\mathrm{eval}}$; any post-horizon
steps used solely to verify final restoration do not change \(\widehat D\).

Hard restoration is retained as a sensitivity condition. Retraction is kept as
a separately named actuator intervention.

## 5. Consequence for historical evidence

The historical Step 6 single-erasure map combined disabled body--ground
collision with hard restoration. It remains useful as a failure-discovery
record, but its $N=4$ task losses are not valid evidence for the repaired
physical model. Clean references and the entire 196-event map must be
recomputed; those corrected results are reported in Steps 8 and 9.

## 6. Saved evidence

All machine-readable values, the three MP4 videos, previews, and the comparison
figure are under
`exp_data/reliability/step7_collision_repair_policy_diagnostic/`.
