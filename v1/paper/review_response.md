# Review audit

Updated 2026-09-11 after the reproducibility and claim-alignment pass.

## Follow-up evidence audit --- 2026-09-12

All stored JSON, figures, and PDFs predate the corrected default free-response
path. The maintained manuscript therefore excludes the archived numerical
results by default and presents the corrected evaluation protocol pending a
complete rerun. The simulator-free regression suite now passes 6/6. A newly
identified numerical issue was also corrected: `hinf_riccati` now reports
iteration-budget exhaustion as infeasible instead of returning an unconverged
matrix with a successful status.

## Confirmed and corrected

- Connected the Kalman/oscillator free response to the deployed joint/task QP,
  including the force-row offset and prediction diagnostics, and added focused
  regression coverage. The old detuning table was removed pending regeneration.
- Limited the common-P bounded-real result to its actual six-state weighted
  state/control output. It is no longer presented as a tissue-force certificate
  or as the unconstrained realization of the deployed 20-state QP.
- Removed the unsupported horizon theorem and force-invariance proposition;
  the paper now gives the valid finite-horizon norm conversion and lists the
  missing terminal-set and contact-output requirements.
- Rewrote the abstract around the realistic 15 mm benchmark and randomized
  simulation results. The 6 cm command is now explicitly a nonclinical
  feasibility stress test.
- Clarified the RCM projection in Equation (2), QP dimensions, and the
  implemented forgetting factor \(\alpha=0.995\). Both RLS parameter and
  covariance equations now freeze when excitation or residual gates are off.
- Clarified that nonlinear RCM satisfaction is empirical because the curvature
  bound is not computed online.
- Distinguished the targeted 3.522 mm soft-null-space ablation from the
  8.13 mm position-impedance baseline in Table I.
- Added a PID-plus-force-governor baseline.
- Extended the randomized harness to retain raw per-seed metrics, solver
  statuses, fallback and saturation counts, force exceedances, maxima, and 95\%
  confidence intervals; the corrected artifact still requires a simulator run.
- Added sensitivity of the empirical force-tightening factor. At
  \(k_e=1200\), \(\rho_F=0.25,0.33,0.40\) gives 2.39, 2.90, and 3.28 N.
- Corrected Equation (12) and the implementation so actuator limits constrain
  the total command \(\tau_{\rm ff}+J_{\rm tip}^\top F_{\rm mpc}+\tau_a\),
  not only the auxiliary torque. The code uses the FR3 MuJoCo joint-force
  ranges.
- Investigating the randomized failure exposed an unsafe zero-feedback
  infeasibility response. The controller now falls back to stable tip MPC with
  soft RCM regulation for that update.
- Relabeled the delay sweep as configured actuator delay rather than measured
  latency, and instrumented solver-only and complete controller-call wall time
  separately.

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
- No formal nonlinear RCM/force invariant. The common-P calculation certifies
  weighted state/control performance for a separate six-state linear model.
- Full simulation reproduction still requires the external `pHRI/simulation`
  source. Public Python packages are pinned and missing-source detection is now
  explicit through `PHRI_SIM_DIR`.
- Corrected oscillator, randomized-tail, and end-to-end timing artifacts still
  require regeneration before submission.
- Tight RCM bands can trigger fallback under the total actuator box.
- The high-delay fallback trades RCM accuracy for stability.
- A constraint-matched MPVIC baseline remains future work.
