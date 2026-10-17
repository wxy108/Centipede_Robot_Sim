# Step 40: high-frequency IID contact erasure

This experiment uses the center-rooted physical measured-STL robot.  It spans
N=5..10 and the seven phase increments labelled
`N_wave_ref10=1,1.5,2,2.5,3,3.5,4` on the ten-segment
reference.  Each of the 42 conditions has 512
independent trials (21504 total).

Every leg remains commanded pitch-down (5 deg), and leg yaw is
fixed.  At each of 50 channel windows per gait cycle, each physical leg draws
an independent Bernoulli(0.5) erasure.  An erased leg's collision channel is
disabled for the complete window; the leg is not lifted.  Evaluation lasts
12 cycles, or 600
channel windows.  Body-yaw commands are zero-order held at that same 50-sample
per-cycle resolution.  Source XML files remain unchanged.

The primary performance measure is direct signed COM forward displacement in
meters.  `iid_contact_erasure_trials.csv` contains every raw trial;
`iid_contact_erasure_summary.csv` contains condition-level mean, median, SD and
IQR; `matched_clean_references.csv` contains q=0 comparisons produced with the
same numerical discretization.  `validation.json` records completion,
independence and contact-erasure gates.
