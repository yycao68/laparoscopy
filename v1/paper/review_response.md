# Review audit

## Confirmed and corrected

- Restored the Kalman/oscillator development and the \(H_\infty/H_2\)
  section in the main manuscript. The issue was not that these ideas were
  invalid; the issue was the evidence level. They are now written as
  implemented/design-level mechanisms with explicit simulation and LMI checks,
  not as unqualified clinical guarantees.
- Added a common-P bounded-real LMI script, `simulation/lap_commonP_sdp.py`,
  for the linear post-feedforward residual plant. It certifies a single storage
  function over the tested inertia-scale polytope.
- Restored the respiration-oscillator stress test in `lap_experiments.py`.
  The current slow 0.25 Hz task shows negligible measured difference between
  random-walk and oscillator models, so the paper keeps the oscillator as an
  implemented internal-model option for faster periodic motion or longer
  horizons.
- Rewrote the abstract around the realistic 15 mm benchmark and randomized
  simulation results. The 6 cm command is now explicitly a nonclinical
  feasibility stress test.
- Clarified the RCM projection in Equation (2), QP dimensions, RLS indicator,
  and the implemented forgetting factor \(\alpha=0.995\).
- Clarified that nonlinear RCM satisfaction is empirical because the curvature
  bound is not computed online.
- Distinguished the targeted 3.522 mm soft-null-space ablation from the
  8.13 mm position-impedance baseline in Table I.
- Added a PID-plus-force-governor baseline.
- Added eight matched randomized episodes with mean and sample standard
  deviation.
- Added sensitivity of the empirical force-tightening factor. At
  \(k_e=1200\), \(\rho_F=0.25,0.33,0.40\) gives 2.39, 2.90, and 3.28 N.
- Corrected Equation (12) and the implementation so actuator limits constrain
  the total command \(\tau_{\rm ff}+J_{\rm tip}^\top F_{\rm mpc}+\tau_a\),
  not only the auxiliary torque. The code uses the FR3 MuJoCo joint-force
  ranges.
- Investigating the randomized failure exposed an unsafe zero-feedback
  infeasibility response. The controller now falls back to stable tip MPC with
  soft RCM regulation for that update.
- Re-ran the latency sweep after the total-command actuator bound. The 5 ms
  case remains stable under constrained joint/task control, but now has larger
  RMSE and RCM deviation than the 0 ms case; the paper reports this directly.

## Not present in the current source/PDF

- No `equation[[...]]` placeholders occur in the LaTeX source or built PDF.
- Equation (8) already contains both position and velocity input rows.
- Penetration uses \(\delta\); no conflicting \(\hat\delta\) notation occurs.
- The RLS dead-zone indicator is correctly typeset.
- All referenced figures are embedded and numbered consistently.
- “ROBOT-ASSISTED” and the opening sentence are correct in the built PDF.
- Discussion before Conclusion is the standard order and was retained.

## Remaining limitations

- No ex vivo or hardware validation.
- No formal nonlinear RCM/force invariant. The paper now has a formal linear
  common-P disturbance-to-force certificate and measured nonlinear simulation
  evidence, but hardware/ex vivo safety still requires quantified nonlinear
  residual margins.
- Tight RCM bands can trigger fallback under the total actuator box.
- The high-delay fallback trades RCM accuracy for stability.
- A constraint-matched MPVIC baseline remains future work.
