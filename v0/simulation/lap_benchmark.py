"""
Laparoscopy benchmark: tool-tip palpation through a compliant trocar onto a
respiring Hunt--Crossley tissue, with a lateral RCM-stress load.

Runs three controllers (classical impedance, Impedance MPC, Impedance MPC +
oscillator Kalman), reports the verification metrics of Section 7, and saves a
comparison figure.

Run:
    python3 lap_benchmark.py            # headless, saves PNG + JSON
"""

from __future__ import annotations
import sys, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from lap_env import LapEnv, TrocarParams, TissueParams
from lap_controller import LapController, LapCtrlParams

# ----------------------------------------------------------------------
# Task timeline
# ----------------------------------------------------------------------
T_END      = 12.0
DT_DECIM   = 10                      # QP every 10 inner steps (100 Hz @ 1 kHz)
S_APPROACH = 0.02                    # reach the tissue surface (m, along shaft)
S_PENETR   = 0.015                   # commanded penetration past the surface (m)
T_A0, T_A1 = 0.0, 2.0               # approach ramp
T_HOLD1    = 9.0                    # hold until
PUSH_WIN   = (5.0, 7.0)            # lateral RCM-stress load window
PUSH_MAG   = 5.0                    # lateral push at the tip (N)


def _smoothstep(a, b, t):
    if t <= a:
        return 0.0
    if t >= b:
        return 1.0
    x = (t - a) / (b - a)
    return 3*x**2 - 2*x**3


def s_ref(t):
    """Commanded tip displacement along the shaft (m) and its derivatives."""
    s_target = S_APPROACH + S_PENETR
    if t < T_HOLD1:
        f = _smoothstep(T_A0, T_A1, t)
        return s_target * f
    else:
        f = 1.0 - _smoothstep(T_HOLD1, T_END, t)
        return s_target * f


def references(env, t, dt):
    """World-frame tip reference (pos, vel, acc) along the shaft."""
    s0 = env.s0
    s = s_ref(t)
    sp = (s_ref(t+dt) - s_ref(t-dt)) / (2*dt)
    sa = (s_ref(t+dt) - 2*s_ref(t) + s_ref(t-dt)) / (dt*dt)
    p_d = env.p_tip0 + s * s0
    dp_d = sp * s0
    ddp_d = sa * s0
    return p_d, dp_d, ddp_d


def lateral_axis(s0):
    ref = np.array([0.0, 0.0, 1.0])
    if abs(s0 @ ref) > 0.9:
        ref = np.array([1.0, 0.0, 0.0])
    u = ref - (ref @ s0) * s0
    return u / np.linalg.norm(u)


# ----------------------------------------------------------------------
def run(mode, k_rcm=None):
    env = LapEnv()
    params = LapCtrlParams()
    if k_rcm is not None:
        params.k_rcm = k_rcm
    ctrl = LapController(mode, env, params)
    dt = env.dt
    u_lat = lateral_axis(env.s0)
    R_d = env.R0
    n_steps = int(T_END / dt)

    log = {k: [] for k in ("t", "tip_err", "tip_s", "ref_s", "F_tissue",
                           "rcm", "wall_F", "d_hat")}
    F_hold = np.zeros(3)
    for k in range(n_steps):
        t = k * dt
        dyn, st = env.get_dynamics_and_state()
        p_d, dp_d, ddp_d = references(env, t, dt)
        resolve = (k % DT_DECIM == 0)
        tau, info = ctrl.control(dyn, st, p_d, dp_d, ddp_d, R_d,
                                 env.p_rcm0, resolve=resolve)
        env.apply_torque(tau)

        # --- physics forces ---
        p_tip, s_hat, v_tip, _ = env.tip_state(dyn, st)
        F_tis, delta = env.tissue_force(p_tip, v_tip, t)
        env.apply_point_force(st, F_tis, p_tip)
        F_tro, e_rcm = env.trocar_force(st, env.p_rcm0)
        env.apply_point_force(st, F_tro, env.p_rcm0)
        if PUSH_WIN[0] <= t < PUSH_WIN[1]:
            env.apply_point_force(st, PUSH_MAG * u_lat, p_tip)

        # --- log ---
        s_actual = float((p_tip - env.p_tip0) @ env.s0)
        log["t"].append(t)
        log["tip_err"].append(float(np.linalg.norm(p_d - p_tip)) * 1e3)
        log["tip_s"].append(s_actual * 1e3)
        log["ref_s"].append(s_ref(t) * 1e3)
        log["F_tissue"].append(float(np.linalg.norm(F_tis)))
        log["rcm"].append(float(np.linalg.norm(e_rcm)) * 1e3)
        log["wall_F"].append(float(np.linalg.norm(env.tro.K_wall * e_rcm)))
        log["d_hat"].append(float(np.linalg.norm(info["d_hat"])))

        env.step()

    return {k: np.asarray(v) for k, v in log.items()}


def metrics(L):
    t = L["t"]
    hold = (t >= 2.5) & (t <= T_HOLD1)
    ss = (t >= 8.5) & (t <= T_HOLD1)
    push = (t >= PUSH_WIN[0]) & (t <= PUSH_WIN[1])
    nonpush_hold = hold & ~push
    return dict(
        tip_rmse_mm=float(np.sqrt(np.mean(L["tip_err"][hold]**2))),
        ss_err_mm=float(np.mean(L["tip_err"][ss])),
        peak_tissue_N=float(np.max(L["F_tissue"])),
        max_rcm_mm=float(np.max(L["rcm"])),
        peak_wall_N=float(np.max(L["wall_F"])),
        resp_resid_mm=float(np.std(L["tip_err"][nonpush_hold])),
    )


def main():
    runs = {
        "Classical Impedance":      run("impedance"),
        "Impedance MPC":            run("mpc"),
        "Impedance MPC + Kalman":   run("mpc_kalman"),
    }
    print("\n" + "="*78)
    hdr = f"{'Controller':<26}{'TipRMSE':>9}{'SSerr':>8}{'PeakF':>8}" \
          f"{'maxRCM':>8}{'wallF':>8}{'respσ':>8}"
    print(hdr); print("-"*78)
    table = {}
    for name, L in runs.items():
        m = metrics(L); table[name] = m
        print(f"{name:<26}{m['tip_rmse_mm']:>9.2f}{m['ss_err_mm']:>8.2f}"
              f"{m['peak_tissue_N']:>8.2f}{m['max_rcm_mm']:>8.3f}"
              f"{m['peak_wall_N']:>8.2f}{m['resp_resid_mm']:>8.3f}")
    print("="*78)
    print("Units: mm, mm, N, mm, N, mm")

    # ---- figure ----
    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("Laparoscopy benchmark: palpation through compliant trocar "
                 "(respiring Hunt–Crossley tissue + lateral RCM load)",
                 fontsize=12, fontweight="bold")
    colors = {"Classical Impedance": "tab:red", "Impedance MPC": "tab:blue",
              "Impedance MPC + Kalman": "tab:green"}
    for name, L in runs.items():
        c = colors[name]
        ax[0, 0].plot(L["t"], L["tip_err"], c, label=name, lw=1.3)
        ax[0, 1].plot(L["t"], L["tip_s"], c, lw=1.3)
        ax[0, 2].plot(L["t"], L["F_tissue"], c, lw=1.3)
        ax[1, 0].plot(L["t"], L["rcm"], c, lw=1.3)
        ax[1, 1].plot(L["t"], L["d_hat"], c, lw=1.3)
    ref = next(iter(runs.values()))
    ax[0, 1].plot(ref["t"], ref["ref_s"], "k:", lw=1, label="reference")
    ax[0, 0].set_title("Tip tracking error"); ax[0, 0].set_ylabel("mm")
    ax[0, 0].legend(fontsize=7); ax[0, 0].grid(alpha=0.3)
    ax[0, 1].set_title("Insertion depth (along shaft)"); ax[0, 1].set_ylabel("mm")
    ax[0, 1].legend(fontsize=7); ax[0, 1].grid(alpha=0.3)
    ax[0, 2].set_title("Tool–tissue force"); ax[0, 2].set_ylabel("N")
    ax[0, 2].grid(alpha=0.3)
    ax[1, 0].set_title("RCM lateral deviation"); ax[1, 0].set_ylabel("mm")
    ax[1, 0].axvspan(*PUSH_WIN, color="orange", alpha=0.12, label="lateral load")
    ax[1, 0].legend(fontsize=7); ax[1, 0].grid(alpha=0.3)
    ax[1, 1].set_title("‖disturbance estimate d̂‖"); ax[1, 1].set_ylabel("N")
    ax[1, 1].grid(alpha=0.3)
    for a in ax[:, :].ravel():
        a.set_xlabel("time (s)")
    # bar summary
    ax[1, 2].axis("off")
    names = list(table.keys())
    mets = ["tip_rmse_mm", "ss_err_mm", "max_rcm_mm"]
    labels = ["Tip RMSE\n(mm)", "SS err\n(mm)", "max RCM\n(mm)"]
    x = np.arange(len(mets)); w = 0.25
    axb = fig.add_subplot(2, 3, 6)
    for i, name in enumerate(names):
        axb.bar(x + (i-1)*w, [table[name][mm] for mm in mets], w,
                color=colors[name], label=name)
    axb.set_xticks(x); axb.set_xticklabels(labels, fontsize=8)
    axb.set_title("Summary"); axb.legend(fontsize=7); axb.grid(axis="y", alpha=0.3)

    out = Path(__file__).parent / "lap_benchmark.png"
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=150)
    print(f"\n[plot] saved -> {out}")
    (Path(__file__).parent / "lap_metrics.json").write_text(json.dumps(table, indent=2))
    print(f"[json] saved -> {Path(__file__).parent / 'lap_metrics.json'}")


if __name__ == "__main__":
    main()
