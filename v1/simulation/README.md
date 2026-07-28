# Laparoscopy v1 simulation

This directory is the implementation used by the v1 paper.

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
episodes and reports mean plus sample standard deviation.

For latency testing, `actuator_delay` enables a supervisory stability
backstop. The constrained joint/task branch is retained through the verified
5 ms case. Above the configured 6 ms threshold, the controller switches to
tip MPC with soft null-space RCM regulation. This avoids divergence through
20 ms in the current sweep, but it deliberately trades sub-millimetre RCM
accuracy for stability.

`lap_verify.py` exits nonzero if RLS, force limitation, RCM regulation, or
adverse-sensing checks fail. It also reports median, 95th-percentile, and
maximum QP wall time.

The offline H-infinity scripts are part of the restored safety/invariance
track. `lap_hinf_design.py` computes the per-vertex H2/H-infinity design and
the unconstrained MPC/LQR gain match. `lap_commonP_sdp.py` solves a common-P
bounded-real LMI over the inertia-scale polytope using the same exact-ZOH input
matrix as the controller. The resulting certificate is a linear post-feedforward
disturbance-to-force bound and a 1 kHz state-feedback option; it is not a
global nonlinear clinical safety certificate.
