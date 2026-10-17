# Step 40 experiment contract

## Morphology and spatial phase

Let (N\in\{5,6,7,8,9,10\}) be the number of physical robot segments.  The
phase label (N_{\mathrm{wave}}^{(10)}) is defined on the ten-segment reference
robot and takes values

\[
N_{\mathrm{wave}}^{(10)}\in\{1,1.5,2,2.5,3,3.5,4\}.
\]

Its adjacent body-yaw phase increment is

\[
\Delta\phi=-\frac{2\pi N_{\mathrm{wave}}^{(10)}}{9}.
\]

This same (\Delta\phi), rather than the effective number of waves, is held
fixed when (N) changes.  The effective number of complete waves present in
an (N)-segment body is therefore

\[
N_{\mathrm{wave,eff}}
=N_{\mathrm{wave}}^{(10)}\frac{N-1}{9}.
\]

## Deterministic controller

The body-yaw target for joint (j\in\{0,\ldots,N-2\}) is

\[
\theta_j(t)=A\sin(\omega t+j\Delta\phi),
\qquad A=19^\circ,\quad \omega=4\;\mathrm{rad\,s^{-1}}.
\]

The gait period is (T_g=2\pi/\omega\).  Leg yaw is fixed.  Every physical
leg-pitch target remains at the same down command,

\[
\theta^{\mathrm{pitch}}_{i,s}(t)=+5^\circ,
\qquad s\in\{L,R\}.
\]

Thus this experiment contains no random swing/stance command and no random
leg posture.  The body-yaw target is zero-order held within each channel
window at 50 samples per gait cycle; the matched clean reference uses exactly
the same discretization.

## IID contact-erasure channel

One gait cycle is divided into (H=50) equal target windows,

\[
\tau_c=\frac{T_g}{H}\approx 0.031416\;\mathrm{s}.
\]

For trial (r), window (h), segment (i\in\{1,\ldots,N\}), and physical
side (s\in\{L,R\}), define

\[
E_{r,h,i,s}\sim\operatorname{Bernoulli}(q),\qquad q=0.5.
\]

All (E_{r,h,i,s}) are mutually independent across trial, time window,
segment, and side.  The collision channel is

\[
C_{r,h,i,s}=1-E_{r,h,i,s}.
\]

When (E_{r,h,i,s}=1), all collisions involving that physical leg are
disabled for the complete window.  The leg is not lifted and its pitch command
does not change.  Consecutive erased windows remain continuously disabled;
there is no instantaneous restore event between them.

Each trial contains 4 clean warm-up cycles followed by 12 evaluated cycles,
or 600 temporal channel windows.  It therefore contains (1200N) independent
leg-channel decisions.  Every morphology-phase condition contains 512 trials,
for 42 conditions and 21,504 trials in total.

The random key is constructed independently for each trial from

\[
(\text{master seed},N,N_{\mathrm{wave}}^{(10)},r,\text{experiment tag}),
\]

and every complete binary mask is hashed.  Duplicate masks are a validation
failure.

## Performance and validation

The primary performance variable is the direct, unnormalized signed forward
COM displacement

\[
\widehat D_{r}=
y_{\mathrm{COM}}(t_{\mathrm{end}})-y_{\mathrm{COM}}(t_{\mathrm{start}})
\quad[\mathrm{m}].
\]

Condition summaries report its mean, median, standard deviation, quartiles,
and interquartile range.  A matched (q=0), all-pitch-down reference is also
computed for every morphology-phase condition.  No success threshold is
introduced at this stage, so the experiment measures a performance
distribution rather than imposing a reliability definition after the fact.

Numerical validity (finite state, no MuJoCo warning, exact channel enforcement,
and collision-state restoration) is kept separate from locomotion outcomes.
Falling or inversion is retained as a physical performance failure and is not
discarded as an invalid trial.
