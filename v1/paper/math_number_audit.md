# Math and number audit for laparoscopy v1

Date: 2026-06-25

Scope: `laparoscopy/v1/paper/body.tex` checked against the v1 simulation and
design scripts in `laparoscopy/v1/simulation/`.

## Scripts rerun

- `python3 lap_benchmark.py`
- `python3 lap_verify.py`
- `python3 lap_sweep.py`
- `python3 lap_monte_carlo.py`
- `python3 lap_experiments.py`
- `python3 lap_hinf_design.py`
- `python3 lap_commonP_sdp.py`

All scripts completed successfully. The common-P SDP emitted CLARABEL's
standard "solution may be inaccurate" warning, but the script independently
checked the strict LMI residual and accepted only the strictly negative result.

## Confirmed paper numbers

### Nominal benchmark

Matches `lap_metrics.json`.

- Position impedance: RMSE 8.64 mm, SS error 3.94 mm, RCM 8.13 mm,
  respiratory residual 2.29 mm, peak force 0.85 N.
- PID + force governor: RMSE 8.69 mm, SS error 4.91 mm, RCM 8.13 mm,
  respiratory residual 2.08 mm, peak force 0.78 N.
- Tip-only MPC: RMSE 2.55 mm, SS error 0.64 mm, RCM 5.53 mm,
  respiratory residual 0.56 mm, peak force 1.19 N.
- Joint/task MPC: RMSE 2.25 mm, SS error 1.78 mm, RCM 0.354 mm,
  respiratory residual 0.59 mm, peak force 1.18 N.
- QP timing rerun: joint/task median 0.457 ms, p95 0.541 ms. The paper's
  "about 0.45 ms" statement remains consistent.

### Targeted verification

Matches `lap_verify.py` and `lap_sweep.json`.

- RLS nominal estimate: 476.3 N/m^1.5, 4.7 percent error.
- No-contact RLS drift: zero; covariance growth: x1.0.
- Deep-reference constrained peak: 1.46 N.
- Force limit disabled peak: 7.69 N.
- Decision-coupled RCM: 0.362 mm.
- Soft-null-space RCM: 3.522 mm.
- Sensor/stick-slip stress: peak 1.43 N, estimated stiffness 518.5 N/m^1.5.
- Tissue sweep final errors: 5.1, 4.7, and 7.1 percent.
- RCM-band sweep: 2.595, 0.362, 0.559, and 0.856 mm.
- Tightening sweep at k_e = 1200: rho 0.25, 0.33, 0.40 gives 2.39, 2.90,
  and 3.28 N.

### Randomized repeated trials

Matches `lap_monte_carlo.json`.

- Joint/task MPC: 3.08 +/- 0.93 mm RMSE, 1.214 +/- 1.612 mm RCM,
  1.34 +/- 0.25 N peak force.
- PID + force governor: 9.57 +/- 1.17 mm RMSE, 8.112 +/- 1.589 mm RCM.
- Tip-only MPC: 2.60 +/- 0.57 mm RMSE, 5.557 +/- 0.582 mm RCM.

### Robustness experiments

Matches `lap_experiments.json`.

- Oscillator/no-oscillator cases: all 0.745 mm respiratory residual and
  2.310 mm tip RMSE in the current slow-respiration task.
- Latency RMSE: 2.41, 2.52, 2.79, 3.25 mm for 0, 5, 10, 20 ms.
- Latency RCM: 0.347, 0.519, 5.673, 6.035 mm.
- Trocar clearance RCM: 0.351, 0.349, 0.349 mm.
- Trocar clearance SS error: 0.227, 0.221, 0.221 mm.

### H2/H-infinity design

Matches `lap_hinf_design.json` and `lap_commonP_sdp.json`.

- H2/LQR spectral radius: 0.8182.
- MPC/LQR gain-match error: 9.78e-10.
- Local H-infinity gamma minimum: 1.0081.
- Deploy gamma: 1.5122.
- Deploy spectral radius: 0.9728.
- Common-P gamma: 1.480.
- Common-P strict LMI max eigenvalue: -4.4e-08.
- Common-P max spectral radius: 0.9936.
- Frequency-sweep norm max: 1.1083.

## Math/text corrections made

- Corrected the constant-A_d computational statement: powers of A_d and weights
  are fixed, but B_d, Gamma, and the numerical Hessian H are updated for the
  frozen configuration.
- Corrected the RCM linearization lemma to match the implemented one-point
  current-configuration linearization, not a previous-trajectory successive
  convexification.
- Clarified the LMI sign convention: the LMI uses signed feedback
  F = Ktilde x, while the Riccati convention writes F = -K x.
- Clarified the Kalman/oscillator evidence level: the estimator exists and is
  used for diagnostic/free-response branches, but the reported joint/task QP
  benchmark is mainly supported by RLS penetration rows and RCM rows; oscillator
  tests show no measured benefit in the current slow-respiration case.

## Remaining evidence-level caveats

- The common-P result is a linear post-feedforward certificate, not a nonlinear
  clinical safety guarantee.
- The explicit robust terminal set is described as a design/future-work object,
  not as a completed online recursive-feasibility proof.
- The current RCM rows use one-point linearization and empirical sweep evidence;
  a certified curvature back-off is still future work.
