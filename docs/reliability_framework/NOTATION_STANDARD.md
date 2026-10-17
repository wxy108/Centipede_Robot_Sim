# Canonical notation for the Centipede reliability program

**Status:** Canonical from 7 August 2026 onward.  Earlier Step 1/Step 2 files are historical records; when they use `L` for the phase-group count or `R` for the repeat count, those symbols must be read through the legacy map at the end of this note.

## 1. Non-negotiable symbol rules

1. \(\mathrm L\) and \(\mathrm R\) mean **left** and **right** only. They are labels, never scalar variables.
2. \(G\) is the spatial phase alphabet size: the number of distinct phase values assigned across the body. Lowercase \(g\) indexes the corresponding phase-equivalence classes.
3. \(m_g\) is the phase-class multiplicity: the number of physical segment copies having phase \(g\). In a balanced phase partition, \(m_g=m=N/G\), but \(m\) is not generally the spatial repetition count.
4. Lowercase \(j\) is the copy index within a phase group; \(i_{g,j}\) is the corresponding physical segment index.
5. Lowercase \(c\) is the gait-cycle index. It is never a copy index or a correlation parameter.
6. Lowercase \(e\) denotes one support event; uppercase \(E_{\mathrm{act}}\) denotes actuator work. Erasure is always \(Z\), never \(E\).
7. The first controlled experiment has temporal burst length \(K=1\). \(K\) is not folded into the spatial redundancy count.
8. \(P_\phi\) is the primitive spatial period of the ordered phase sequence, and \(\nu=N/P_\phi\) is the spatial repetition count when \(P_\phi\mid N\). A paper-specific wave count \(\xi\) may be identified with \(\nu\), not with \(m_g\) in general.

## 2. Morphology and grouping

| Symbol | Definition |
|---|---|
| \(N\) | Number of bilateral body segments/modules. The robot has \(2N\) individual legs. |
| \(\mathcal S=\{\mathrm L,\mathrm R\}\) | Set of physical sides. |
| \(\sigma\in\mathcal S\) | Side label. |
| \(\mathcal M_i=\{(i,\mathrm L),(i,\mathrm R)\}\) | Bilateral leg pair belonging to segment \(i\); this is the basic spatial failure object. |
| \(\phi_i\) | Nominal spatial phase assigned to segment \(i\), reduced modulo \(2\pi\). |
| \(\mathcal A_\phi=\{\phi_i\bmod 2\pi:i=1,\ldots,N\}\) | Spatial phase alphabet: the set of distinct phase values present across the body. |
| \(G=|\mathcal A_\phi|\) | Spatial phase alphabet size, equivalently the number of distinct phase-equivalence classes. |
| \(\bar\phi_g\) | The \(g\)-th distinct phase value in \(\mathcal A_\phi\). |
| \(g\in\{1,\ldots,G\}\) | Phase-class index. |
| \(\mathcal G_g=\{i:\phi_i\equiv\bar\phi_g\pmod{2\pi}\}\) | Homologous phase class: physical segments sharing phase \(\bar\phi_g\). |
| \(m_g=|\mathcal G_g|\) | Phase-class or homologous-copy multiplicity. |
| \(m\) | Common class multiplicity for a balanced partition: \(m_g=m\) and \(N=Gm\). |
| \(j\in\{1,\ldots,m_g\}\) | Copy index within group \(g\). |
| \(i_{g,j}\) | Physical segment occupied by copy \(j\) of group \(g\). |
| \(P_\phi\) | Primitive spatial period: the smallest number of segments after which the ordered phase sequence repeats exactly. |
| \(\nu=N/P_\phi\) | Spatial repetition count when \(P_\phi\) divides \(N\). |
| \(\xi\) | Paper-specific spatial wave count. Set \(\xi=\nu\) only when one primitive period represents one complete wave under the paper's convention. |

For the present antiphase inter-segment gait,

\[
(\phi_1,\ldots,\phi_N)
=
(0,\pi,0,\pi,\ldots),
\qquad
\mathcal A_\phi=\{0,\pi\},
\qquad
\boxed{G=|\mathcal A_\phi|=2}.
\]

The two phase classes are

\[
\mathcal G_1=\{1,3,5,\ldots\},
\qquad
\mathcal G_2=\{2,4,6,\ldots\}.
\]

The primitive ordered motif is \((0,\pi)\), hence

\[
P_\phi=2,
\qquad
\nu=\frac{N}{2}.
\]

Each phase occurs exactly once per primitive motif, so this particular gait has the special equality

\[
m_1=m_2=m=\nu=\frac{N}{2}.
\]

Thus

\[
N=6:\quad (G,m,P_\phi,\nu)=(2,3,2,3),
\qquad
N=8:\quad (G,m,P_\phi,\nu)=(2,4,2,4).
\]

The equality \(m=\nu\) is not general. For example, the repeated motif \((0,0,\pi,\pi)\) has \(G=2\) and \(P_\phi=4\); if repeated \(\nu\) times, each phase-class multiplicity is \(m_g=2\nu\). Therefore \(m_g\) counts homologous physical channels, whereas \(\nu\) counts complete ordered-motif repetitions.

In the balanced, regularly repeated case,

\[
i_{g,j}=g+(j-1)G.
\]

## 3. Time and support events

| Symbol | Definition |
|---|---|
| \(t\) | Continuous simulation time. |
| \(\omega\) | CPG angular frequency. |
| \(T_{\mathrm{gait}}=2\pi/\omega\) | One complete gait period. |
| \(d\in(0,1)\) | Nominal stance duty factor. |
| \(\tau_{\mathrm{st}}=dT_{\mathrm{gait}}\) | Duration of one nominal support window. Currently \(d=0.5\), hence \(\tau_{\mathrm{st}}=T_{\mathrm{gait}}/2\). |
| \(T_{\mathrm{eval}}\) | Fixed duration of a simulation trial. |
| \(c\in\mathbb N_0\) | Gait-cycle index. |
| \(e=(c,g,\sigma)\) | One group-level support event: cycle \(c\), phase group \(g\), and nominal stance side \(\sigma\). |
| \(t_e\) | Onset time of event \(e\). |
| \(\mathcal W_e=[t_e,t_e+\tau_{\mathrm{st}})\) | Support window associated with event \(e\). |
| \(K\) | Number of consecutive support windows in a temporal erasure burst. The baseline is \(K=1\). |

The side \(\sigma\) in \(e=(c,g,\sigma)\) identifies the **nominal stance that triggers the event**. It does not mean that only side \(\sigma\) is disabled by an erasure.

## 4. Exact module-window contact-erasure channel

For event \(e=(c,g,\sigma)\) and copy \(j\), define

\[
Z_{e,j}=Z_{c,g,\sigma,j}\in\{0,1\},
\]

where \(Z_{e,j}=1\) means that the entire bilateral pair \(\mathcal M_{i_{g,j}}\) is erased during \(\mathcal W_e\). Let \(A_{i,\sigma'}(t)\in\{0,1\}\) be leg-ground collision availability for physical side \(\sigma'\in\mathcal S\). Then

\[
Z_{e,j}=1
\Longrightarrow
A_{i_{g,j},\sigma'}(t)=0
\quad
\forall\,t\in\mathcal W_e,
\quad
\forall\,\sigma'\in\mathcal S.
\]

Outside the erased window, \(A_{i,\sigma'}(t)=1\). All actuator commands remain nominal:

\[
\mathbf u(t)=\mathbf u^{0}(t).
\]

This is an **exact segment-pair leg-ground contact erasure**, not a leg-lifting command. Because the current MuJoCo model has one collision mesh per leg and no separate foot collision geom, the implementation disables the complete leg-mesh-to-ground collision channel for both sides of the selected segment.

## 5. Noise law and group failure

The marginal erasure probability is

\[
q=\Pr(Z_{e,j}=1).
\]

Independent copies:

\[
Z_{e,1},\ldots,Z_{e,m_g}
\overset{\mathrm{iid}}{\sim}
\operatorname{Bernoulli}(q).
\]

Perfectly correlated copies:

\[
U_e\sim\operatorname{Bernoulli}(q),
\qquad
Z_{e,j}=U_e
\quad\forall j.
\]

Define the complete phase-group failure indicator

\[
F_e=\prod_{j=1}^{m_g}Z_{e,j}.
\]

Therefore

\[
\Pr(F_e=1)=
\begin{cases}
q^{m_g}, & \text{independent copies},\\
q, & \text{perfectly correlated copies}.
\end{cases}
\]

Here \(F_e=1\) means that all redundant segment-pair copies in **one phase group and one support event** are erased. It does not mean whole-robot failure. The task-level consequence must be measured through the full dynamics.

For descriptive purposes,

\[
\rho_Z=\operatorname{Corr}(Z_{e,j},Z_{e,j'}),\qquad j\neq j',
\]

with \(\rho_Z=0\) and \(\rho_Z=1\) at the two first-stage endpoints. Pairwise \(\rho_Z\) alone is not sufficient to specify an intermediate joint tail law.

## 6. Physical and task-level outputs

| Symbol | Definition |
|---|---|
| \(J^{\perp}_{e,j,\sigma'}\) | Normal leg-ground impulse of side \(\sigma'\) for copy \(j\) during \(\mathcal W_e\). |
| \(\widehat D\) | Measured signed forward displacement over \([0,T_{\mathrm{eval}}]\). |
| \(D_0(N)\) | Mean or preregistered clean-reference displacement for the same morphology \(N\), controller, and \(T_{\mathrm{eval}}\). |
| \(\varepsilon_D\) | Allowed absolute displacement deviation from the reference. |
| \(E_{\mathrm{act}}\) | Positive actuator mechanical work. |
| \(D^\star\) | Prescribed target distance in a later goal-reaching task. |
| \(\widehat T_{D^\star}\) | First-passage time to \(D^\star\); omitted from the fixed-duration first experiment. |
| \(\mathcal R_D\) | Fixed-duration displacement reliability. |
| \(\mathcal R_{\mathrm{task}}\) | Full distance-time-energy task reliability for a goal-reaching experiment. |

Using a fixed forward unit vector \(\widehat{\mathbf e}_{\mathrm f}\),

\[
\widehat D=
\bigl[\mathbf p_{\mathrm{COM}}(T_{\mathrm{eval}})
-\mathbf p_{\mathrm{COM}}(0)\bigr]
\cdot\widehat{\mathbf e}_{\mathrm f}.
\]

The first controlled experiment uses

\[
\boxed{
\mathcal R_D(q,N)
=
\Pr\!\left(
|\widehat D-D_0(N)|\le\varepsilon_D
\right).
}
\]

Because every trial has the same \(T_{\mathrm{eval}}\), no random arrival-time variable appears here. If the experiment later changes to a target-distance task, use

\[
\boxed{
\mathcal R_{\mathrm{task}}
=
\Pr\!\left(
|\widehat D-D^\star|\le\varepsilon_D,
\widehat T_{D^\star}\le T_{\max},
E_{\mathrm{act}}\le E_{\max}
\right).
}
\]

## 7. Information-theoretic statement

Let \(X_e\) be a logical source contact symbol and \(\mathbf Y_e=(Y_{e,1},\ldots,Y_{e,m_g})\) its repeated channel outputs. Under the binary-erasure idealization,

\[
\frac{I(X_e;\mathbf Y_e)}{H(X_e)}
=1-\Pr(F_e=1)
=
\begin{cases}
1-q^{m_g}, & \text{independent copies},\\
1-q, & \text{perfectly correlated copies}.
\end{cases}
\]

This is a channel-level result. It predicts whether at least one copy of a logical contact symbol survives; it does not by itself predict \(\mathcal R_D\). Whole-body dynamics supplies the experimentally measured map from \(F_e\) and the full erasure pattern to \(\widehat D\).

## 8. Legacy-to-canonical map

| Legacy notation | Canonical notation | Reason |
|---|---|---|
| \(L\): number of phase classes | \(G\) | Reserves \(\mathrm L\) for left. |
| \(R\): repeated copies per class | \(m\) or \(m_g\) | Reserves \(\mathrm R\) for right. |
| \(j\): phase-class index | \(g\) | Makes “group” visually explicit. |
| \(r\): copy index | \(j\) | Standard generic within-group index; avoids visual collision with \(\mathrm R\). |
| \(k\): stance-event count | \(c\): gait-cycle index | Makes the temporal meaning explicit. |
| \(\mathcal C_j\): phase class | \(\mathcal G_g\): phase group | Avoids collision with contact/channel notation. |
| \(A_e\): all-copy erasure | \(F_e\): complete group failure | \(A\) is reserved for collision availability. |
| \(B_e\): shared Bernoulli draw | \(U_e\) | Reserves \(K\) for burst length without adding another “B/burst” ambiguity. |
| \(T_g\) | \(T_{\mathrm{gait}}\) | Avoids collision with group index \(g\). |
| \(\tau_s\) | \(\tau_{\mathrm{st}}\) | Avoids collision with side index and states “stance” explicitly. |
| \(M\): number of events/trials | \(n_{\mathrm{evt}}\), \(n_{\mathrm{trial}}\) | Reserves \(\mathcal M_i\) for a bilateral module pair. |
| \(\widehat T\) | \(\widehat T_{D^\star}\) | States which target defines the first-passage time. |
| \(\mathcal E\) | \(E_{\mathrm{act}}\) | Makes actuator work explicit and remains distinct from event \(e\). |

Raw files from the completed controller-level stance-omission pilot may still contain legacy keys `L` and `R`. They are immutable historical data fields. New code, plots, tables, and manuscripts must emit `G` and `m` and may read the old keys only through an explicit compatibility layer.
