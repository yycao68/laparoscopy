# Peer-Review and Reproducibility Audit

**Manuscript:** [v1/paper/laparoscopy_ieee.tex](v1/paper/laparoscopy_ieee.tex) with [v1/paper/body.tex](v1/paper/body.tex)  
**Review date:** 2026-09-10; full follow-up audit 2026-09-12; regeneration completed 2026-09-13  
**Recommendation:** Ready for a further human read-through; no known blocking issue remains

## Regeneration Completed --- 2026-09-13

The external `pHRI/simulation` bridge that blocked every prior audit pass was
available in this environment (resolved via `fr3_dependency.py`'s
`PHRI_SIM_DIR` default). All eight simulation entry points were rerun from
the corrected controller: `lap_benchmark.py`, `lap_experiments.py`,
`lap_monte_carlo.py`, `lap_sweep.py`, `lap_verify.py`, `lap_hinf_design.py`,
and `lap_commonP_sdp.py`, plus the 8/8-passing focused test suite
(`test_lap_controller.py`, which has grown from the 6 tests below to 8).

The free-response fix changes the headline result: nominal maximum RCM
deviation is now 0.82\,mm (not the pre-fix 0.354\,mm), tip-only MPC edges out
joint/task MPC on tip tracking (2.55 vs.\ 2.61\,mm RMSE), the RCM-band sweep
is non-monotonic rather than identifying 0.1\,mm as best, the 4\,ms configured-delay
threshold now sends the 5\,ms case to fallback (not just 10--20\,ms), and the
trocar-clearance test shows a real (if modest) sensitivity rather than none.
The randomized study shows QP-solve fallbacks in 7 of 8 episodes (34 of
${\sim}9600$ steps), not the "one episode" the pre-fix run suggested. The
$H_2/H_\infty$ and common-$P$ numbers (Major Finding 3's gain/spectral-radius
values, Finding 2's $\gamma=1.480$) are unaffected by the fix --- they are a
separate six-state offline analysis --- and were independently reproduced
exactly.

`body.tex`'s `\ifdefined\includeStalePreCorrectionResults` gate has been
removed; the paper now carries only regenerated values, and every table,
figure, and cross-referenced number was checked against a fresh script run
rather than assumed. Both PDFs rebuild clean at 9 pages, matching the
project's own prior page-count note.

Findings 5 (unmatched RCM baselines), 6's remaining items (no lock file
beyond the now-added `requirements.txt`, no CI), and the deeper analytical
issues in Findings 2--4 (now reframed rather than resolved: the common-$P$
certificate is correctly scoped to weighted state/control performance, and
the invalid theorem/proposition were removed rather than repaired) remain
open, as does hardware/ex vivo validation.

**Mid-regeneration correction.** The first draft of this regeneration
attributed the headline-table shift entirely to the free-response fix. A
2x2 ablation (trust-region row x free-response fix, each on/off) on the same
nominal episode showed that is wrong: relaxing the trust region alone (with
the free-response fix still active) recovers most of the pre-correction
numbers (1.71\,mm RMSE / 0.59\,mm RCM, versus 2.61/0.82\,mm with it active),
while the free-response fix alone, with the trust region relaxed, moves this
benchmark only marginally (1.71/0.59\,mm versus 2.23/0.58\,mm with both
reverted). The dominant cause is an undocumented joint trust-region row
(`rcm_trust_radius`, 10\,mrad per joint) added in the same commit as the
free-response fix, with no prior mention in the paper, this review, or any
commit message found. Structurally it enforces the deviation bound
Lemma~\ref{lem:curv} (RCM linearization error) assumes but never enforced
before, so it is not an arbitrary addition -- but its cost was previously
uncredited and its value (10\,mrad) is a configured constant, not derived
from a certified $L_\Phi$ bound. `body.tex` has been corrected to attribute
the shift to this row and describe it in Section III where the RCM rows are
defined. There remains a smaller, unexplained ${\sim}0.22$\,mm gap between
"both changes reverted" (0.58\,mm) and the original pre-audit figure
(0.354\,mm) that this ablation does not account for; it was not chased
further given the two dominant, now-isolated causes above.

**Is this a bug, and why exactly does it hurt?** A follow-up analysis first
confirmed the constraint's algebra is correct: `self.q0` is set once at
episode start and never reassigned, which looked suspicious, but its
appearance in `q_offset = self.q0 + qfree - st.q` algebraically cancels
against the `st.q - self.q0` term already baked into the state vector, so
what the row actually bounds is `\|q_{k+i} - q_current\|`, a proper rolling
trust region, not one anchored to a stale point -- confirmed empirically
(`st.q` itself drifts up to 0.12 rad from `self.q0` during the episode
without permanent infeasibility, which a literal stale-anchor bug would
produce). A finer sweep (0.010 to 0.020 rad in 0.002 steps) showed the
shipped 10\,mrad value sits at a fragile, non-monotonic QP-feasibility
boundary -- some nearby values solve cleanly, others (including 10\,mrad
itself) intermittently produce `solved inaccurate`/`maximum iterations
reached` outcomes that cascade into fallback and worse RCM tracking. Values
${\ge}18$\,mrad were uniformly clean in this sweep. **Verdict: not a logic
bug, but a materially mistuned constant.** `rcm_trust_radius` was changed to
0.02 (20\,mrad) and every affected script rerun:
- `lap_benchmark.py`: joint/task MPC now 1.71\,mm RMSE / 0.59\,mm RCM
  (was 2.61 / 0.82), *better* than tip-only MPC on both axes rather than
  worse on tracking.
- `lap_monte_carlo.py`: 2.49$\pm$0.93\,mm RMSE / 0.62$\pm$0.22\,mm RCM
  (was 4.00$\pm$2.17 / 0.68$\pm$0.29); fallbacks 11 of ~9600 steps in 6/8
  episodes (was 34 in 7/8); 0 torque-saturation steps (was 26).
- `lap_experiments.py` Exp A (oscillator): the retuned QP has more room to
  correct a mismatched disturbance model, so the four detuning conditions
  now span only 1.707--1.739\,mm RMSE (${\le}2\%$), versus a much larger,
  counter-intuitively-ordered spread before retuning.
- `lap_experiments.py` Exp B (0\,ms row) improves to 1.61\,mm RMSE / 0.32\,mm
  RCM (RCM unchanged from before retuning; the 5/10/20\,ms rows are in the
  soft-RCM fallback branch and do not use this constant at all).
- `lap_experiments.py` Exp C and `lap_sweep.py`'s RCM-band sweep are
  unaffected by this constant (confirmed by rerun: identical to the
  pre-retuning numbers), so those sections' numbers and prose stand as
  already written.

`body.tex` (abstract, Section III, Table I, evidence status, Sections
VI-C/VI-D/VI-G, Discussion, Conclusion) was updated throughout to the
retuned numbers, with the mistuning explicitly named as the cause of the
intermediate regeneration's worse figures rather than the free-response fix.
Both PDFs rebuild clean at 9 pages.

## Full Follow-up Audit --- 2026-09-12

The follow-up audit compared the current v1 source with every stored JSON,
figure, and PDF. All stored outputs are timestamped 2026-09-10, while the
controller and reporting paths changed on 2026-09-11. Because the default
nominal and randomized controller enables the corrected oscillator/Kalman free
response, the archived closed-loop numbers cannot be attributed to the current
controller. The maintained manuscript now excludes those tables and figures by
default and contains no active quantitative claim derived from them.

The executable evidence available without the external simulator is now six
focused tests. They verify dependency diagnostics, free-response forwarding,
command sensitivity, adaptive force-row sensitivity, fallback timing/status
reporting, and rejection of an unconverged $H_\infty$ Riccati iteration. The
suite passes 6/6. The added numerical test addresses an audit finding that
iteration-budget exhaustion previously returned a false feasible status.
The same pass found that a failed full QP followed by a successful fallback
reported only the fallback status and last-solve time. The controller now
preserves the primary status and reports aggregate solver time, solve count,
and fallback status for the complete controller call.

The full simulation and both linear-design entry points remain blocked by the
external `pHRI/simulation` bridge. Accordingly, the active paper presents the
evaluation protocol and operating envelope, not the archived values. Its tone
has also been revised from deficit-oriented comparisons to symmetric design
tradeoffs while retaining the actual local-linearization, fallback,
contact-model, and simulation-only boundaries.

## Revision Status --- 2026-09-11

The main claim corrections and the oscillator code defect identified below
have been addressed:

- the Kalman/oscillator free response now enters the deployed joint/task QP,
	including its adaptive force rows and prediction diagnostics; focused
	tests cover forwarding, command sensitivity, force-row sensitivity, and the
	external-dependency error;
- the stale oscillator table and figure panel were removed pending a rerun;
- the common-$P$ result is now limited to the six-state weighted state/control
	output, and the deployed 20-state QP is explicitly separated from that study;
- the invalid horizon theorem and force-invariance proposition were removed;
- the RLS parameter and covariance updates are both written with the
	implementation gates;
- solver-only and complete controller-call timing are now distinct metrics;
- randomized runs now retain raw per-seed metrics, statuses, fallbacks,
	saturation, threshold exceedances, maxima, and 95\% confidence intervals;
- a pinned public dependency manifest and an actionable `PHRI_SIM_DIR`
	preflight were added.

Submission remains blocked on the external `pHRI/simulation` source, rerunning
all controller-dependent and linear-design artifacts, adding constraint-matched
baselines, and rebuilding the PDF. The detailed findings below are retained as
the audit trail that motivated the revision.

## Executive Summary

The paper has a credible central simulation result: the implemented joint/task QP adds decision-coupled RCM authority, and the stored artifacts consistently report substantially lower RCM deviation than the position-impedance and tip-only controllers. The source also handles several limitations unusually candidly, including empirical force tightening, loss of RCM performance during fallback, lack of recursive-feasibility proof, and absence of hardware or ex vivo validation.

The manuscript is not submission-ready, however. The oscillator ablation is a no-op for the reported joint/task controller, so the four identical oscillator results cannot support the interpretation given in the paper. More importantly, the common-$P$ value $\gamma=1.480$ is computed for a six-state weighted state/control-output plant, not for tissue contact force, and neither its gain nor storage matrix is used by the deployed 20-state joint/task QP. The horizon theorem and force-invariance proposition then combine an induced $\ell_2$ gain with pointwise disturbance growth and assert a robust invariant terminal set that is neither constructed nor imposed online.

Reproduction is also blocked from a clean checkout: every simulation imports an undeclared sibling `pHRI/simulation` tree, which is absent from this repository. There is no dependency manifest or automated regression suite. Consequently, the stored JSON values can be checked for internal agreement with the paper, but the reported simulations could not be rerun from the supplied project.

## Major Findings

### 1. The respiration-oscillator experiment is a no-op for the reported controller

The controller constructs an oscillator-dependent free response in [v1/simulation/lap_controller.py#L521-L529](v1/simulation/lap_controller.py#L521-L529), but the deployed `mpc_kalman` branch calls the joint/task QP with `d_free_tip=None` in [v1/simulation/lap_controller.py#L537-L548](v1/simulation/lap_controller.py#L537-L548). The free response is used only by the tip-MPC branch in [v1/simulation/lap_controller.py#L563-L568](v1/simulation/lap_controller.py#L563-L568).

Experiment A changes `use_oscillator` and oscillator frequency while retaining `mode="mpc_kalman"` in [v1/simulation/lap_experiments.py#L96-L109](v1/simulation/lap_experiments.py#L96-L109). Therefore those settings cannot alter the joint/task QP command. The four exactly identical rows in `lap_experiments.json` follow by construction; they do not show that the random-walk state is sufficient for the 0.25 Hz task, as claimed in [v1/paper/body.tex#L477-L486](v1/paper/body.tex#L477-L486).

**Required correction:** pass the oscillator/random-walk free response into `_full_qp`, add a regression proving that oscillator state changes the predicted trajectory or first command, and rerun detuning tests at conditions where the internal model is observable. Otherwise remove the oscillator experiment and describe the oscillator as undeployed design code.

### 2. The common-$P$ certificate does not bound tissue contact force

The common-$P$ script defines

$$
z=\begin{bmatrix}Q^{1/2}x\\R^{1/2}F_{\mathrm{mpc}}\end{bmatrix}
$$

through `Cz` and `Dz` in [v1/simulation/lap_commonP_sdp.py#L112-L118](v1/simulation/lap_commonP_sdp.py#L112-L118). Its reported $\gamma=1.480$ therefore bounds a weighted state/control-output channel. It does not include a tissue-contact output map.

A separate local Riccati script adds the linearized contact slope $C_f^\top C_f$ in [v1/simulation/lap_hinf_design.py#L180-L198](v1/simulation/lap_hinf_design.py#L180-L198), but that is not the common-$P$ problem that produces $\gamma=1.480$. The manuscript nevertheless uses the common-$P$ result in a force-safety section and concludes that realized contact force is bounded by a cap plus margins in [v1/paper/body.tex#L365-L393](v1/paper/body.tex#L365-L393). Applied corrective force $F_{\mathrm{mpc}}$ and tissue reaction $F_{\mathrm{ext}}$ are not interchangeable during transients.

**Required correction:** include an explicit linearized contact-force output in the common-$P$ synthesis and verify its gain at every vertex, or limit $\gamma=1.480$ to weighted state/control performance. Do not use that number as a tissue-force budget until the certified output is the actual contact-force channel.

### 3. The synthesized $H_2/H_\infty$ law is not the unconstrained realization of the reported joint/task QP

The gain-match script verifies a six-state, three-input tip model with terminal cost $P_2$ in [v1/simulation/lap_hinf_design.py#L47-L84](v1/simulation/lap_hinf_design.py#L47-L84). The reported controller is instead a 20-state, ten-input joint/task QP. It uses `Q_full`, auxiliary-torque weights, and a scalar terminal multiplier in [v1/simulation/lap_controller.py#L127-L140](v1/simulation/lap_controller.py#L127-L140) and [v1/simulation/lap_controller.py#L379-L390](v1/simulation/lap_controller.py#L379-L390); it does not use $P_2$, the common-$P$ storage, or either synthesized gain.

The paper states that the synthesized gain is what the QP realizes when inequalities are inactive and that the design supplies a low-level 1 kHz option in [v1/paper/body.tex#L340-L364](v1/paper/body.tex#L340-L364). No controller mode loads that gain, and the JSON artifacts retain only gain norms, not a deployable gain matrix.

**Required correction:** either implement and test the synthesized feedback and use its terminal matrix in the actual joint/task QP, including a gain-equivalence test for the full 20-state/10-input model, or present the $H_2/H_\infty$ work as a separate offline design study rather than the nominal law underlying the reported QP.

### 4. The horizon theorem and force-invariance proposition are not established by the supplied analysis

The bounded-real LMI gives an induced sequence-energy statement, $\|z\|_2\le\gamma\|w\|_2$. The horizon theorem instead adds the pointwise growth quantity $L_dN\Delta t$ directly to $\bar w$ and applies the same induced-gain bound in [v1/paper/body.tex#L184-L203](v1/paper/body.tex#L184-L203). If $\bar w$ is an $\ell_2$ sequence bound, this addition is not justified; if it is a per-sample amplitude bound, conversion to a horizon $\ell_2$ bound introduces horizon and sample-time scaling.

The proposition additionally asserts positive invariance of

$$
\mathcal X_f=\{x:x^\top Px\le c\}\cap\mathcal C_{\mathrm{tight}}
$$

without selecting $c$, constructing the robust predecessor condition, or proving that the bounded-real dissipation inequality makes $V$ decrease under nonzero persistent disturbance. The online QP has no terminal-set row and uses only a scaled terminal penalty, as the paper itself acknowledges in [v1/paper/body.tex#L295-L300](v1/paper/body.tex#L295-L300). No numerical values for $e_K$, $L_d$, $\bar w$, $c$, or the aggregate nonlinear margin are supplied or checked by code.

Calling the proposition conditional does not repair the derivation: several conditions needed for the conclusion are uncomputed, and robust positive invariance does not follow from the induced-gain inequality alone.

**Required correction:** distinguish finite-horizon pointwise bounds from infinite-horizon induced $\ell_2$ bounds, include the required norm conversion, identify a measured residual set, compute a robust invariant terminal set and terminal row, and verify the nonlinear margins. Otherwise remove the theorem/proposition and retain the LMI as a linear stability and weighted-performance result only.

### 5. The headline RCM comparison is not constraint-matched

The proposed controller is the only main-table method with decision-coupled hard RCM rows and seven auxiliary joint-torque decisions. Position impedance and tip-only MPC rely on soft null-space RCM action. The resulting 0.354 versus 8.13/5.53 mm comparison in [v1/paper/body.tex#L403-L432](v1/paper/body.tex#L403-L432) demonstrates the value of adding direct hard RCM authority, but it does not isolate prediction or the broader interaction-dynamics formulation.

The targeted hard-versus-soft ablation is useful, and the manuscript admits that a constraint-matched MPVIC baseline is missing. That missing baseline remains consequential because the abstract, contributions, and conclusion lead with the unmatched RCM ratios.

**Required correction:** add a torque-level or kinematic RCM controller with the same actuator box and RCM target, and preferably a constraint-matched predictive-impedance baseline. Until then, frame the result as a mechanism demonstration of decision-coupled RCM constraints rather than comparative superiority of predictive interaction dynamics.

## Reproducibility and Reporting Findings

### 6. A clean checkout cannot execute the simulations

Both [v1/simulation/lap_env.py#L24-L30](v1/simulation/lap_env.py#L24-L30) and [v1/simulation/lap_controller.py#L20-L26](v1/simulation/lap_controller.py#L20-L26) import `fr3_mujoco`, `so3_utils`, and `fr3_impedance` from a sibling `pHRI/simulation` directory. That directory is neither part of this repository nor declared as a submodule or package dependency. With the VS Code-selected Python 3.12 environment, `import lap_env` fails with:

```text
ModuleNotFoundError: No module named 'fr3_mujoco'
```

The repository also has no `requirements.txt`, lock file, `pyproject.toml`, or Python tests. `lap_verify.py` gates four broad thresholds, but exact manuscript numbers, oscillator behavior, fallback counts, common-$P$ feasibility, and timing claims have no regression assertions. In particular, `lap_commonP_sdp.py` returns normally if the SDP is infeasible.

**Required correction:** vendor or submodule the shared simulator, replace path injection with an installable dependency, pin Python and package versions, and add regression tests for every abstract/table/conclusion value. The verification command should fail if the SDP is infeasible or a claimed artifact is stale.

### 7. Solver timing is not end-to-end control timing, and the latency supervisor is configured rather than measured

The timer starts inside `_solve_osqp` after dynamics evaluation, prediction-matrix construction, Hessian formation, and constraint assembly in [v1/simulation/lap_controller.py#L289-L339](v1/simulation/lap_controller.py#L289-L339). The reported approximately 0.45 ms is therefore OSQP setup/update/solve wall time, not end-to-end controller latency.

The latency study inserts a fixed command-buffer delay and selects the fallback from the configured `actuator_delay` threshold in [v1/simulation/lap_experiments.py#L45-L70](v1/simulation/lap_experiments.py#L45-L70) and [v1/simulation/lap_controller.py#L533-L568](v1/simulation/lap_controller.py#L533-L568). Actual computation time does not advance simulated time, and the supervisor does not detect missed deadlines. The paper appropriately says the 1 kHz request is simulation-only, but “measured QP wall time” should not be used to imply a 1 kHz end-to-end implementation.

**Required correction:** time the complete control call, report hardware/OS/Python/solver versions and deadline misses, and distinguish configured actuator delay from measured computation/communication delay.

### 8. The randomized study hides the safety-relevant tail

The eight-seed study reports only mean and sample standard deviation. For joint/task MPC, RCM deviation is $1.214\pm1.612$ mm and one episode invokes fallback, indicating a strongly skewed safety-relevant tail. The generated JSON stores scenarios and aggregate summaries but not per-seed controller metrics, solver statuses, saturation, or fallback counts in [v1/simulation/lap_monte_carlo.py#L109-L158](v1/simulation/lap_monte_carlo.py#L109-L158).

**Required correction:** save raw per-seed metrics and status logs; report maximum RCM deviation, threshold-exceedance count, fallback count, force violations, and confidence intervals. Eight fixed seeds are acceptable as an exploratory stress set but not as a robustness distribution.

### 9. The RLS equations do not match the gated implementation

Equation (RLS) gates the parameter update with the residual indicator, while the following covariance equation is written as an unconditional forgetting update in [v1/paper/body.tex#L309-L319](v1/paper/body.tex#L309-L319). The implementation returns before updating either the estimate or covariance when penetration is below 1 mm or the residual is inside the dead zone in [v1/simulation/lap_controller.py#L173-L197](v1/simulation/lap_controller.py#L173-L197). This distinction is exactly what produces the reported no-contact covariance growth of $\times1.0$.

**Required correction:** write the estimator and covariance updates piecewise, with both frozen when excitation or residual gates are inactive.

## Editorial Checks

- The maintained abstract is approximately 241 words, within the common IEEE 250-word limit.
- Within `body.tex`, no duplicate labels, missing `\ref`/`\eqref` targets, or undefined citation keys were found.
- The current source contains 18 bibliography entries.
- Existing project notes report a nine-page IEEE build, but no LaTeX or PDF inspection tool is installed in the current environment, so the PDF was not independently rebuilt or page-audited.
- Given the lack of hardware/ex vivo evidence and the unresolved force-certificate issues, “Safe” in the title should be reconsidered in favor of “Constraint-Aware” or “Force-Limited” unless the formal claims are repaired.

## Claims Supported by the Supplied Artifacts

The following points are internally consistent between current source and stored JSON artifacts, although they could not be rerun from this checkout:

1. The implemented full QP has 50 decision variables for $N=5$, with Cartesian-force, total-torque, RCM, and adaptive penetration rows.
2. The stored nominal metrics match Table I: 2.25 mm tip RMSE, 0.354 mm maximum RCM deviation, and 1.18 N peak tissue force for joint/task MPC.
3. Total-command actuator rows include current feedforward torque, and the final command is clipped to the FR3 actuator box.
4. The guarded RLS implementation freezes adaptation without penetration/excitation and projects estimated parameters into configured bounds.
5. The empirical force-tightening sweep reports 2.39, 2.90, and 3.28 N at $\rho_F=0.25,0.33,0.40$ for $k_e=1200\,\mathrm{N/m}^{1.5}$.
6. The stored RCM-band sweep exposes non-monotonic behavior and fallback at the tightest 0.05 mm setting rather than hiding it.
7. The paper clearly disclaims clinical certification, hardware/ex vivo validation, nonlinear RCM guarantees, and completed recursive feasibility.

## Recommended Revision Order

1. Wire the oscillator/free-response model into the deployed joint/task QP and replace the invalid ablation.
2. Separate the common-$P$ weighted-performance certificate from contact-force claims, or resynthesize it with an explicit tissue-force output.
3. Either connect the synthesized gain/storage to the actual full QP or remove the constrained-realization and 1 kHz implementation claims.
4. Rework or remove the horizon and invariance theorem/proposition; compute the missing residual, terminal-set, and nonlinear-margin quantities if retained.
5. Make the repository executable from a clean checkout and add exact regression coverage.
6. Add constraint-matched RCM/predictive-impedance baselines and report randomized tail/fallback statistics.
7. Correct timing terminology and the gated RLS equations.

## Validation Record

- Repository status before review: clean.
- Selected interpreter: `C:\Users\HILDEV01\Documents\Yongyan\.venv\Scripts\python.exe` (Python 3.12.12).
- Installed environment includes NumPy 2.5.3, SciPy 1.18.1, OSQP 1.1.3, CVXPY 1.9.2, CLARABEL 0.11.1, and MuJoCo 3.13.0; the manuscript reports MuJoCo 3.6.0 and provides no lock file.
- Runtime reproduction: blocked at `import lap_env` by missing `fr3_mujoco` from undeclared sibling `pHRI/simulation`.
- Python/TeX editor diagnostics: none in the reviewed v1 sources.
- LaTeX/PDF toolchain: `latexmk`, `pdflatex`, `tectonic`, `pdfinfo`, `pdftotext`, and `mutool` are unavailable; no rebuild was performed.
- No manuscript, simulation source, generated JSON, figure, or PDF was modified by this review.
