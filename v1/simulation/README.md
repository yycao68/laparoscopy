# Laparoscopy v1 simulation

This directory is the implementation used by the v1 paper.

Python 3.12 is used for the current scripts. Install the pinned public
dependencies from the v1 directory:

```bash
python -m pip install -r requirements.txt
```

The FR3 bridge is an external source dependency, not a PyPI package. Set
`PHRI_SIM_DIR` to the `pHRI/simulation` directory containing
`fr3_mujoco.py`, `so3_utils.py`, and `fr3_impedance.py`:

```bash
set PHRI_SIM_DIR=C:\path\to\pHRI\simulation
```

Without those sources, simulator entry points fail immediately with an
actionable dependency message. The focused controller tests remain runnable:

```bash
python -m pytest test_lap_controller.py
```

Main commands:

```bash
python3 lap_benchmark.py
python3 lap_verify.py
python3 lap_experiments.py
python3 lap_monte_carlo.py
python3 lap_hinf_design.py
python3 lap_commonP_sdp.py
```

`lap_controller.py` implements the exact-ZOH joint/task QP. The complete
`mpc_kalman` mode optimizes Cartesian tip force and seven joint correction
torques in one problem. The actuator box is imposed on the total command
`tau_ff + J_tip.T @ F_mpc + tau_aux`, using the FR3 joint force ranges from
the MuJoCo model, rather than on the auxiliary correction alone. RCM and
adaptive tissue-penetration limits are OSQP constraint rows. `mpc_kalman_soft`
retains the old null-space RCM realization for ablation.

`pid_force` provides the simple PID-plus-force-governor baseline used in the
revised comparison. `lap_monte_carlo.py` runs eight matched randomized
episodes and stores raw per-seed metrics, sample statistics, 95% confidence
intervals, extrema, solver statuses, fallback counts, saturation counts, and
force-threshold exceedances.

For latency testing, `actuator_delay` enables a supervisory stability
backstop. The constrained joint/task branch is retained through the verified
5 ms case. Above the configured 6 ms threshold, the controller switches to
tip MPC with soft null-space RCM regulation. This avoids divergence through
20 ms in the current sweep, but it deliberately trades sub-millimetre RCM
accuracy for stability.

`lap_verify.py` exits nonzero if RLS, force limitation, RCM regulation, or
adverse-sensing checks fail. It reports OSQP-only and complete controller-call
wall times separately. `test_lap_controller.py` contains six simulator-free
regression tests, including checks for fallback timing/status reporting and for
rejecting exhausted H-infinity Riccati iteration rather than reporting it as
feasible.

`lap_monte_carlo.py` writes raw per-seed metrics, QP status counts, fallback
counts, torque-saturation counts, force-threshold exceedances, maxima, and 95%
confidence intervals in addition to mean and sample standard deviation.

The offline H-infinity scripts form a separate six-state linear design study.
`lap_hinf_design.py` computes the per-vertex H2/H-infinity design and verifies
an unconstrained tip-MPC/LQR gain match. `lap_commonP_sdp.py` solves a common-P
bounded-real LMI over the inertia-scale polytope. Its output channel weights
tip state and corrective command; it does not certify tissue force or the
deployed 20-state joint/task QP. Both entry points obtain the nominal plant from
`LapEnv` and therefore require the same external FR3 bridge as the simulations.
