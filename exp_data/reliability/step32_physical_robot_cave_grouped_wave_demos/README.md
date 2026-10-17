# Step 32: cave-grouped physical robot wave demos

This is the corrected replacement for the Step-31 spatial pitch pattern.
Step 31 evaluated an independent continuous pitch precursor at every segment;
that can split one visible body-curvature half-wave into different pitch
states.  Those videos must not be used as the final wave-number demos.

Here body-yaw actuators are treated as samples between adjacent segment
centers.  If body joint `j` has phase `omega*t+j*Delta_phi`, segment `i` has
the collocated phase `omega*t+(i-1/2)*Delta_phi`.  The integer half-wave index
is `floor(theta_segment/pi)`.  Every segment with the same index receives one
identical right/left stance choice; the opposite side is always complementary.

Validated wave numbers: 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4.
For the originally flagged cases, the legacy Step-31 command was uniform in
only 33.3% of
true cave groups at `N_wave=1` and
25.0% at
`N_wave=1.5`; the corrected controller is 100%.
See `cave_grouping_validation.json`, `figures/`, and `demos/`.
