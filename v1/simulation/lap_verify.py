"""Verification suite for the version-1 constrained controller."""

from __future__ import annotations

from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from lap_env import LapEnv
from lap_controller import LapController, LapCtrlParams

T_END = 10.0
DT_DECIM = 10
PUSH_WIN = (4.0, 6.0)
PUSH_MAG = 5.0


def _smooth(a, b, t):
    if t <= a:
        return 0.0
    if t >= b:
        return 1.0
    x = (t - a) / (b - a)
    return 3.0 * x**2 - 2.0 * x**3


def simulate(
    target_pen=0.015,
    contact=True,
    mode="mpc_kalman",
    rls=True,
    force_limit=3.0,
    force_tightening=0.33,
    true_ke=500.0,
    rcm_band=1.0e-4,
    push=True,
    sensor_sigma=0.0,
    sensor_bias=0.0,
    stick_slip=False,
    mu_s=0.0,
):
    env = LapEnv()
    env.tis.k_e = true_ke
    env.sensor_sigma = sensor_sigma
    env.sensor_bias = sensor_bias
    if stick_slip:
        env.tro.stick_slip = True
        env.tro.mu_s = mu_s
    params = LapCtrlParams(
        enable_rls=rls,
        F_tissue_max=force_limit,
        force_tightening=force_tightening,
        rcm_band=rcm_band,
    )
    ctrl = LapController(mode, env, params)
    s0 = env.s0
    reach = 0.02 + target_pen if contact else 0.0
    lateral = np.array([1.0, 0.0, 0.0])
    lateral -= (lateral @ s0) * s0
    lateral /= np.linalg.norm(lateral)
    n = int(T_END / env.dt)
    log = {
        k: []
        for k in (
            "t",
            "F_tissue",
            "ke_hat",
            "rcm",
            "Gnorm",
            "tip_s",
            "qp_ms",
            "control_ms",
            "pred_force",
            "pred_rcm",
        )
    }

    def s_ref(t):
        if t < 7.5:
            return reach * _smooth(0.0, 2.5, t)
        return reach * (1.0 - _smooth(7.5, T_END, t))

    for k in range(n):
        t = k * env.dt
        dyn, st = env.get_dynamics_and_state()
        p_d = env.p_tip0 + s_ref(t) * s0
        resolve = k % DT_DECIM == 0
        tau, info = ctrl.control(
            dyn,
            st,
            p_d,
            np.zeros(3),
            np.zeros(3),
            env.R0,
            env.p_rcm0,
            resolve=resolve,
        )
        env.apply_torque(tau)
        p_tip, _, v_tip, _ = env.tip_state(dyn, st)
        F_tis, _ = env.tissue_force(p_tip, v_tip, t)
        env.apply_point_force(st, F_tis, p_tip)
        F_tro, e_rcm = env.trocar_force(st, env.p_rcm0)
        env.apply_point_force(st, F_tro, env.p_rcm0)
        if push and PUSH_WIN[0] <= t < PUSH_WIN[1]:
            env.apply_point_force(st, PUSH_MAG * lateral, p_tip)
        log["t"].append(t)
        log["F_tissue"].append(float(np.linalg.norm(F_tis)))
        log["ke_hat"].append(info["ke_hat"])
        log["rcm"].append(float(np.linalg.norm(e_rcm)) * 1.0e3)
        log["Gnorm"].append(float(np.linalg.norm(ctrl.G_rls)))
        log["tip_s"].append(float((p_tip - env.p_tip0) @ s0) * 1.0e3)
        if resolve:
            log["qp_ms"].append(info["qp_ms"])
            log["control_ms"].append(info["control_ms"])
            log["pred_force"].append(info["predicted_peak_force"])
            log["pred_rcm"].append(info["predicted_peak_rcm"] * 1.0e3)
        env.step()
    return {key: np.asarray(value) for key, value in log.items()}


def main():
    true_ke = 500.0

    contact = simulate(target_pen=0.02, push=False)
    no_contact = simulate(target_pen=0.0, contact=False, push=False)
    ke_final = contact["ke_hat"][-1]
    ke_error = abs(ke_final - true_ke) / true_ke * 100.0
    ke_drift = abs(no_contact["ke_hat"][-1] - no_contact["ke_hat"][0])
    cov_growth = no_contact["Gnorm"][-1] / no_contact["Gnorm"][0]

    constrained = simulate(target_pen=0.06, push=False)
    unconstrained = simulate(
        target_pen=0.06,
        push=False,
        force_limit=100.0,
        force_tightening=1.0,
    )
    peak_con = float(constrained["F_tissue"].max())
    peak_unc = float(unconstrained["F_tissue"].max())

    hard = simulate(push=True, mode="mpc_kalman")
    soft = simulate(push=True, mode="mpc_kalman_soft")
    hard_rcm = float(hard["rcm"].max())
    soft_rcm = float(soft["rcm"].max())

    adverse = simulate(
        target_pen=0.04,
        push=False,
        sensor_sigma=0.10,
        sensor_bias=0.05,
        stick_slip=True,
        mu_s=4.0,
    )
    peak_adv = float(adverse["F_tissue"].max())
    ke_adv = float(adverse["ke_hat"][-1])

    qp = hard["qp_ms"]
    control_time = hard["control_ms"]
    print("\n" + "=" * 72)
    print("V1-T1  Adaptive Hunt--Crossley RLS")
    print(
        f"  k_e={ke_final:.1f} ({ke_error:.1f}% error); "
        f"no-contact drift={ke_drift:.2e}; covariance growth=x{cov_growth:.2f}"
    )
    print("V1-T2  QP tissue-force output constraint")
    print(
        f"  constrained peak={peak_con:.2f} N; disabled={peak_unc:.2f} N; "
        f"threshold=3.00 N"
    )
    print("V1-T3  Decision-coupled hard RCM rows")
    print(f"  hard={hard_rcm:.3f} mm; soft-null-space={soft_rcm:.3f} mm")
    print("V1-T4  Noise, bias, and stick-slip")
    print(f"  peak={peak_adv:.2f} N; k_e={ke_adv:.1f}")
    print("V1-T5  Solver-only and end-to-end controller wall time")
    print(
        f"  OSQP median={np.median(qp):.3f} ms; "
        f"p95={np.percentile(qp,95):.3f} ms; max={np.max(qp):.3f} ms"
    )
    print(
        f"  full control median={np.median(control_time):.3f} ms; "
        f"p95={np.percentile(control_time,95):.3f} ms; "
        f"max={np.max(control_time):.3f} ms"
    )
    checks = {
        "rls": ke_error < 10.0 and ke_drift < 1.0,
        "force": peak_con <= 3.0 and peak_unc > peak_con + 0.5,
        "rcm": hard_rcm <= 1.0 and soft_rcm > 1.0,
        "sensor": peak_adv <= 3.0 and abs(ke_adv - true_ke) / true_ke < 0.15,
    }
    print("  " + ", ".join(f"{k}={'PASS' if v else 'FAIL'}" for k, v in checks.items()))
    print("=" * 72)

    fig, axes = plt.subplots(1, 4, figsize=(18, 4.1))
    axes[0].plot(contact["t"], contact["ke_hat"], label="contact")
    axes[0].plot(no_contact["t"], no_contact["ke_hat"], "--", label="no contact")
    axes[0].axhline(true_ke, color="k", ls=":")
    axes[0].set_title("T1: RLS stiffness")
    axes[0].set_ylabel(r"$\hat k_e$ [N/m$^{1.5}$]")
    axes[0].legend(fontsize=7)

    axes[1].plot(constrained["t"], constrained["F_tissue"], label="QP constrained")
    axes[1].plot(unconstrained["t"], unconstrained["F_tissue"], label="constraint disabled")
    axes[1].axhline(3.0, color="k", ls=":")
    axes[1].set_title("T2: tissue force")
    axes[1].set_ylabel("N")
    axes[1].legend(fontsize=7)

    axes[2].plot(hard["t"], hard["rcm"], label="hard QP rows")
    axes[2].plot(soft["t"], soft["rcm"], label="soft null space")
    axes[2].axhline(0.5, color="k", ls=":")
    axes[2].set_title("T3: RCM deviation")
    axes[2].set_ylabel("mm")
    axes[2].legend(fontsize=7)

    axes[3].plot(adverse["t"], adverse["F_tissue"], label="adverse sensing")
    axes[3].axhline(3.0, color="k", ls=":")
    axes[3].set_title("T4: robustness")
    axes[3].set_ylabel("N")
    axes[3].legend(fontsize=7)
    for axis in axes:
        axis.set_xlabel("time [s]")
        axis.grid(alpha=0.3)
    fig.tight_layout()
    out = Path(__file__).parent / "lap_verify.png"
    fig.savefig(out, dpi=170)
    print(f"[plot] saved -> {out}")

    if not all(checks.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
