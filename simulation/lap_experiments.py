"""
Robustness stress tests responding to reviewer critiques (runnable against the
current reference implementation, which has the respiration oscillator, the
compliant trocar with Coulomb friction, and the constant-A_d tip QP + Kalman):

  A. Respiration frequency mismatch (reviewer I.4 / next-step #3)
     True organ motion at 0.25 Hz; the oscillator internal model is detuned by
     +/-20% (and disabled -> pure random walk). Measures the periodic tracking
     residual vs detuning.

  B. Control latency (reviewer III.5)
     The applied torque is delayed by 0..20 ms; measures tracking/stability
     degradation under loop delay.

  C. Trocar radial deadzone + friction (reviewer III.4)
     A radial port clearance (deadzone) is added to the trocar; checks that the
     Kalman still absorbs the axial Coulomb friction (SS error ~ 0) and how RCM
     deviation responds.

Run:  python3 lap_experiments.py     # headless; writes lap_experiments.png + json
"""

from __future__ import annotations
import sys, json
from collections import deque
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from lap_env import LapEnv
from lap_controller import LapController, LapCtrlParams
from lap_benchmark import (references, s_ref, lateral_axis, metrics,
                           T_END, DT_DECIM, PUSH_WIN, PUSH_MAG)


def simulate(true_resp_f=0.20, ctrl_resp_f=0.20, use_oscillator=True,
             latency_ms=0.0, trocar_clear=0.0, mode="mpc_kalman"):
    env = LapEnv()
    env.tis.resp_f = true_resp_f
    env.tro.r_clear = trocar_clear
    params = LapCtrlParams(); params.resp_f = ctrl_resp_f
    ctrl = LapController(mode, env, params, use_oscillator=use_oscillator)
    dt = env.dt
    u_lat = lateral_axis(env.s0); R_d = env.R0
    n_steps = int(T_END / dt)
    lat = int(round(latency_ms * 1e-3 / dt))
    buf = deque([np.zeros(7)] * (lat + 1), maxlen=lat + 1)

    log = {k: [] for k in ("t", "tip_err", "tip_s", "ref_s", "F_tissue",
                           "rcm", "wall_F", "d_hat")}
    for k in range(n_steps):
        t = k * dt
        dyn, st = env.get_dynamics_and_state()
        p_d, dp_d, ddp_d = references(env, t, dt)
        resolve = (k % DT_DECIM == 0)
        tau, info = ctrl.control(dyn, st, p_d, dp_d, ddp_d, R_d,
                                 env.p_rcm0, resolve=resolve)
        buf.append(tau)
        env.apply_torque(buf[0])                       # delayed application

        p_tip, s_hat, v_tip, _ = env.tip_state(dyn, st)
        F_tis, delta = env.tissue_force(p_tip, v_tip, t)
        env.apply_point_force(st, F_tis, p_tip)
        F_tro, e_rcm = env.trocar_force(st, env.p_rcm0)
        env.apply_point_force(st, F_tro, env.p_rcm0)
        if PUSH_WIN[0] <= t < PUSH_WIN[1]:
            env.apply_point_force(st, PUSH_MAG * u_lat, p_tip)

        log["t"].append(t)
        log["tip_err"].append(float(np.linalg.norm(p_d - p_tip)) * 1e3)
        log["tip_s"].append(float((p_tip - env.p_tip0) @ env.s0) * 1e3)
        log["ref_s"].append(s_ref(t) * 1e3)
        log["F_tissue"].append(float(np.linalg.norm(F_tis)))
        log["rcm"].append(float(np.linalg.norm(e_rcm)) * 1e3)
        log["wall_F"].append(float(np.linalg.norm(env.tro.K_wall * e_rcm)))
        log["d_hat"].append(float(np.linalg.norm(info["d_hat"])))
        env.step()
    return {k: np.asarray(v) for k, v in log.items()}


def exp_A():
    """Respiration frequency mismatch (true 0.25 Hz)."""
    true_f = 0.25
    cfgs = [
        ("Random walk (no osc)", dict(use_oscillator=False, ctrl_resp_f=0.25)),
        ("Osc -20% (0.20 Hz)",  dict(ctrl_resp_f=0.20)),
        ("Osc matched (0.25)",  dict(ctrl_resp_f=0.25)),
        ("Osc +20% (0.30 Hz)",  dict(ctrl_resp_f=0.30)),
    ]
    rows = {}
    for name, kw in cfgs:
        L = simulate(true_resp_f=true_f, **kw)
        m = metrics(L)
        rows[name] = dict(resp_resid_mm=m["resp_resid_mm"],
                          tip_rmse_mm=m["tip_rmse_mm"])
    return rows


def exp_B():
    """Control latency sweep (matched oscillator)."""
    rows = {}
    for lat in (0.0, 5.0, 10.0, 20.0):
        L = simulate(true_resp_f=0.25, ctrl_resp_f=0.25, latency_ms=lat)
        m = metrics(L)
        rows[f"{lat:.0f} ms"] = dict(tip_rmse_mm=m["tip_rmse_mm"],
                                     ss_err_mm=m["ss_err_mm"],
                                     peak_tip_mm=float(np.max(L["tip_err"])))
    return rows


def exp_C():
    """Trocar radial deadzone + axial friction (matched oscillator)."""
    rows = {}
    for cl in (0.0, 1.0e-3, 2.0e-3):
        L = simulate(true_resp_f=0.25, ctrl_resp_f=0.25, trocar_clear=cl)
        m = metrics(L)
        rows[f"{cl*1e3:.0f} mm"] = dict(max_rcm_mm=m["max_rcm_mm"],
                                        ss_err_mm=m["ss_err_mm"],
                                        peak_tissue_N=m["peak_tissue_N"])
    return rows


def main():
    A, B, C = exp_A(), exp_B(), exp_C()

    print("\n== Exp A: respiration frequency mismatch (true 0.25 Hz) ==")
    print(f"{'config':<24}{'resp resid [mm]':>16}{'tip RMSE [mm]':>15}")
    for k, v in A.items():
        print(f"{k:<24}{v['resp_resid_mm']:>16.3f}{v['tip_rmse_mm']:>15.3f}")

    print("\n== Exp B: control latency ==")
    print(f"{'latency':<10}{'tip RMSE':>12}{'SS err':>10}{'peak tip':>12}")
    for k, v in B.items():
        print(f"{k:<10}{v['tip_rmse_mm']:>12.3f}{v['ss_err_mm']:>10.3f}{v['peak_tip_mm']:>12.3f}")

    print("\n== Exp C: trocar radial deadzone (+ axial friction) ==")
    print(f"{'clearance':<12}{'max RCM':>10}{'SS err':>10}{'peakF[N]':>10}")
    for k, v in C.items():
        print(f"{k:<12}{v['max_rcm_mm']:>10.3f}{v['ss_err_mm']:>10.3f}{v['peak_tissue_N']:>10.3f}")

    # ---- figure ----
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    fig.suptitle("Laparoscopy robustness stress tests (reviewer responses)",
                 fontweight="bold")
    # A
    names = list(A.keys()); vals = [A[n]["resp_resid_mm"] for n in names]
    cols = ["0.5", "tab:orange", "tab:green", "tab:orange"]
    ax[0].bar(range(len(names)), vals, color=cols)
    ax[0].set_xticks(range(len(names)))
    ax[0].set_xticklabels(names, rotation=20, ha="right", fontsize=7)
    ax[0].set_ylabel("respiration residual [mm]")
    ax[0].set_title("A. Respiration freq. mismatch\n(true 0.25 Hz)")
    ax[0].grid(axis="y", alpha=0.3)
    # B
    names = list(B.keys()); vals = [B[n]["tip_rmse_mm"] for n in names]
    ax[1].bar(names, vals, color="tab:blue")
    ax[1].set_ylabel("tip RMSE [mm]"); ax[1].set_xlabel("control latency")
    ax[1].set_title("B. Control latency"); ax[1].grid(axis="y", alpha=0.3)
    # C
    names = list(C.keys()); xr = np.arange(len(names)); w = 0.38
    ax[2].bar(xr - w/2, [C[n]["max_rcm_mm"] for n in names], w,
              label="max RCM [mm]", color="tab:purple")
    ax[2].bar(xr + w/2, [C[n]["ss_err_mm"] for n in names], w,
              label="SS err [mm]", color="tab:green")
    ax[2].set_xticks(xr); ax[2].set_xticklabels(names)
    ax[2].set_xlabel("trocar radial clearance")
    ax[2].set_title("C. Trocar deadzone + friction")
    ax[2].legend(fontsize=8); ax[2].grid(axis="y", alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.92])
    out = Path(__file__).parent / "lap_experiments.png"
    fig.savefig(out, dpi=150)
    print(f"\n[plot] saved -> {out}")
    (Path(__file__).parent / "lap_experiments.json").write_text(
        json.dumps({"A": A, "B": B, "C": C}, indent=2))


if __name__ == "__main__":
    main()
