# Laparoscopy Impedance-MPC simulation

Reference implementation of the simulation benchmark in
`../laparoscopy.md` (Section 7). It reuses the FR3 MuJoCo
infrastructure from `../../pHRI/simulation/` and adds the
laparoscopy-specific physics in Python (instrument, compliant trocar,
Hunt–Crossley tissue, RCM). Every script runs on the MuJoCo `FR3MuJoCoEnv`
(7-DoF Franka FR3, MuJoCo Menagerie model) via the `LapEnv` wrapper.

## Files

| file | role |
|---|---|
| `lap_env.py` | FR3 + rigid instrument; tool-tip kinematics; RCM error `e_rcm` and analytic Jacobian `J_rcm`; compliant-trocar force (lateral wall spring + axial friction); Hunt–Crossley tissue with sinusoidal respiration. |
| `lap_controller.py` | Tip-space two-layer Impedance MPC. Modes: `impedance` (static baseline), `mpc` (constant-`A_d` QP), `mpc_kalman` (QP + augmented Kalman with the **respiration oscillator**, §3.4.1). Now also includes the **adaptive Hunt–Crossley RLS** (dead-zone + excitation gate + covariance bound, §4) and the **hard force-safety governor** (caps commanded penetration so contact force ≤ `F_tissue_max`, eq. 10c). Null-space RCM regulation shared by all. |
| `lap_benchmark.py` | Palpation task through the trocar onto respiring tissue with a lateral RCM-stress load; runs the three controllers, prints the metric table, writes `lap_benchmark.png` and `lap_metrics.json`. |
| `lap_experiments.py` | Robustness stress tests (reviewer responses): respiration frequency mismatch, control latency, trocar deadzone+friction. Writes `lap_experiments.png` and `lap_experiments.json`. |
| `lap_hinf_design.py` | Offline H∞/H₂ control design (§5) for the post-feedforward tip plant: H₂/LQR via DARE, **numerical Theorem-1 check** (unconstrained-MPC gain = LQR gain when terminal cost = DARE `P`), and the H∞ gain via γ-iteration → reports γ and `F_max=γ·w̄`. Writes `lap_hinf_design.json`. No SDP solver required (γ-iteration on the H∞ Riccati). |
| `lap_verify.py` | **Verification suite** for the implemented mechanisms: (T1) adaptive RLS — `k_e` converges to truth and does **not** drift with no contact (excitation gate + covariance bound); (T2) hard force safety — a deep reference is capped near `F_tissue_max` instead of pushing through; (T3) RCM excursion vs gain; (T4) **sensor robustness** — force-sensor noise + bias + stick-slip trocar friction leave peak force and `k_e` essentially unchanged. Prints PASS/FAIL, writes `lap_verify.png`. |
| `lap_video.py` | **MuJoCo rendering** of the palpation benchmark. Runs the rollout on the FR3 MuJoCo model and renders it off-screen with `mujoco.Renderer`; the analytical instrument shaft, trocar/RCM port, respiring tissue surface, and tool–tissue contact force are drawn as extra MuJoCo viz geoms, with a live HUD. Alongside the 3D view it shows a **synced live-curve panel** (tip error, tissue force with the 3 N cap, insertion vs. reference, RCM deviation) that grows with a moving time marker. Writes `lap_video_<mode>.mp4` (1880×720). |

## Run

```bash
cd laparoscopy/simulation
python3 lap_benchmark.py        # ~10 s, headless; writes PNG + JSON
python3 lap_video.py            # renders the MuJoCo palpation video (mp4)
```

Requires `mujoco`, `osqp`, `scipy`, `numpy`, `matplotlib` (already used by the
pHRI project); `lap_video.py` additionally needs `imageio`, `imageio-ffmpeg`,
and `Pillow`. It imports `fr3_mujoco`, `fr3_impedance`, `so3_utils` from
`../../pHRI/simulation/` via a relative path — no install needed.

## What the scaffold reproduces

Representative output (single run; see paper §7 for the design):

| Controller | Tip RMSE (mm) | SS err (mm) | Peak force (N) | max RCM (mm) | resp. resid. (mm) |
|---|---:|---:|---:|---:|---:|
| Classical Impedance | 8.6 | 3.9 | 0.9 | 8.1 | 2.3 |
| Impedance MPC | 2.6 | 0.6 | 1.2 | 5.5 | 0.6 |
| Impedance MPC + Kalman | 0.2 | 0.03 | 1.2 | 3.4 | 0.14 |

This already demonstrates the three orthogonal mechanisms of the paper:

* **Offset-free tracking** — SS error collapses 3.9 → 0.03 mm with the Kalman
  augmentation (the `K_d^{-1} F` impedance bias is removed without stiffening).
* **Prediction** — tip RMSE 8.6 → 2.6 mm from the constant-`A_d` QP alone.
* **Respiration rejection** — residual σ 2.3 → 0.14 mm from the oscillator
  internal model (§4.4.1): the periodic disturbance is *predicted*, not
  extrapolated flat.

## Implemented vs. paper (honest status)

**Implemented & verified** (`lap_verify.py`):

| mechanism | status | evidence |
|---|---|---|
| Layer-1 feedforward + constant-`A_d` tip QP (OSQP) | ✅ | benchmark |
| Augmented Kalman + respiration oscillator (§3.4.1) | ✅ | benchmark, `lap_experiments` A |
| H₂/H∞ offline design, Theorem-1 check (§5) | ✅ | `lap_hinf_design` (γ≈1.0, gain match 5e-10) |
| **Adaptive Hunt–Crossley RLS** (§4) | ✅ | T1: `k_e`→485.8 (2.8% err), no drift/windup off-contact |
| **Hard force safety / no push-through** (eq. 10c) | ✅ | T2: deep ref capped at 3.5 N vs 5.4 N push-through |
| **Sensor robustness** (noise + bias + stick-slip friction) | ✅ | T4: peak force 3.34 N vs 3.39 N ideal; `k_e` 1.7% err |
| compliant trocar (spring + Coulomb friction + deadzone) | ✅ | `lap_experiments` C (friction absorbed offset-free) |

**Still requires the joint-space reformulation:**

- **Hard RCM constraint (eq. 10b) → sub-0.5 mm band.** RCM is enforced here by a
  *null-space* regulator, and `lap_verify` T3 shows the deviation (~3.5 mm under a
  5 N push) is **insensitive to `k_rcm`** (3.573 → 3.568 mm over a 10× gain range):
  in the decoupled tip-task scheme the null space lacks authority over the pivot,
  so the sub-0.5 mm band genuinely needs the **joint-space QP** with the linearized
  RCM rows (10b) as decision-coupled constraints, not a higher gain. This is the
  one remaining architectural item.
- Full baseline suite (B1–B5 incl. PID+F and MPVIC) and the design Tables 6.3–6.5.

The force governor realises the safety intent of eq. (10c) as a *reference
governor* (caps commanded penetration via the RLS stiffness estimate); the
equivalent OSQP force-row would additionally remove the small (~0.5 N) closed-loop
overshoot seen in T2.

## Key parameters

`TrocarParams` (L=0.30 m, d_trocar=0.15 m, K_wall=200 N/m, friction),
`TissueParams` (k_e=500, n=1.5, b=0.1, respiration 3 mm @ 0.2 Hz),
`LapCtrlParams` (N=10, 100 Hz QP, F_max=8 N, k_rcm=1500). Edit these to match
the paper's §7.1 setup or to explore.
