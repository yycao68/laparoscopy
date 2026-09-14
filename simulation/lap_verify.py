"""
Verification of the three surgical mechanisms now implemented in the controller:

  T1. Adaptive Hunt--Crossley RLS  — k_e estimate converges to truth in contact,
      and does NOT drift (covariance windup) during a no-contact episode
      (excitation gate + covariance bound, Section 4).

  T2. Hard force safety / no push-through  — with a reference commanded deep
      inside the tissue, the force governor caps the contact force near
      F_tissue_max; disabling it produces a large "push-through" force
      (eq. 10c via the RLS stiffness estimate, Section 5).

  T3. RCM regulation  — lateral excursion under a 5 N tip push, vs RCM gain.

Run:  python3 lap_verify.py
"""

from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from lap_env import LapEnv
from lap_controller import LapController, LapCtrlParams

T_END = 12.0
DT_DECIM = 10
PUSH_WIN = (5.0, 7.0)
PUSH_MAG = 5.0


def _smooth(a, b, t):
    if t <= a: return 0.0
    if t >= b: return 1.0
    x = (t-a)/(b-a); return 3*x**2 - 2*x**3


def simulate(target_pen=0.015, contact=True, force_gov=True, rls=True,
             k_rcm=1500.0, push=True, mode="mpc_kalman",
             sensor_sigma=0.0, sensor_bias=0.0, stick_slip=False, mu_s=0.0):
    """Descend along the shaft to `target_pen` past the surface, hold, retract."""
    env = LapEnv()
    env.sensor_sigma = sensor_sigma      # force-sensor noise / bias (sensing model)
    env.sensor_bias = sensor_bias
    if stick_slip:
        env.tro.stick_slip = True
        env.tro.mu_s = mu_s
    p = LapCtrlParams()
    p.enable_force_gov = force_gov
    p.enable_rls = rls
    p.k_rcm = k_rcm
    ctrl = LapController(mode, env, p)
    dt = env.dt
    s0 = env.s0
    # surface is 0.02 m beyond p_tip0 along s0; to penetrate by target_pen the
    # tip must travel 0.02 + target_pen. contact=False -> stay above the surface.
    reach = (0.02 + target_pen) if contact else 0.0
    u_lat = np.array([1.0, 0.0, 0.0]); u_lat = u_lat - (u_lat@s0)*s0; u_lat/=np.linalg.norm(u_lat)
    R_d = env.R0
    n = int(T_END/dt)
    log = {k: [] for k in ("t","F_tissue","ke_hat","delta_max","rcm","Gnorm","tip_s")}
    def s_ref(t):
        if t < 9.0: return reach*_smooth(0,3,t)
        return reach*(1-_smooth(9,12,t))
    for k in range(n):
        t = k*dt
        dyn, st = env.get_dynamics_and_state()
        p_d = env.p_tip0 + s_ref(t)*s0
        resolve = (k % DT_DECIM == 0)
        tau, info = ctrl.control(dyn, st, p_d, np.zeros(3), np.zeros(3), R_d,
                                 env.p_rcm0, resolve=resolve)
        env.apply_torque(tau)
        p_tip, _, v_tip, _ = env.tip_state(dyn, st)
        F_tis, _ = env.tissue_force(p_tip, v_tip, t)
        env.apply_point_force(st, F_tis, p_tip)
        F_tro, e_rcm = env.trocar_force(st, env.p_rcm0)
        env.apply_point_force(st, F_tro, env.p_rcm0)
        if push and PUSH_WIN[0] <= t < PUSH_WIN[1]:
            env.apply_point_force(st, PUSH_MAG*u_lat, p_tip)
        log["t"].append(t)
        log["F_tissue"].append(float(np.linalg.norm(F_tis)))
        log["ke_hat"].append(info["ke_hat"])
        log["delta_max"].append(info["delta_max"]*1e3)
        log["rcm"].append(float(np.linalg.norm(e_rcm))*1e3)
        log["Gnorm"].append(float(np.linalg.norm(ctrl.G_rls)))
        log["tip_s"].append(float((p_tip-env.p_tip0)@s0)*1e3)
        env.step()
    return {k: np.asarray(v) for k,v in log.items()}


def main():
    KE_TRUE = 500.0
    results = {}

    # T1: RLS convergence (in contact) and no-drift (no contact)
    Lc = simulate(target_pen=0.02, contact=True)
    Ln = simulate(target_pen=0.0, contact=False)        # never touches tissue
    ke_final = Lc["ke_hat"][-1]
    ke_err = abs(ke_final-KE_TRUE)/KE_TRUE*100
    ke_drift = abs(Ln["ke_hat"][-1]-Ln["ke_hat"][0])     # should be ~0 (gate)
    G_growth = Ln["Gnorm"][-1]/Ln["Gnorm"][0]            # covariance windup ratio

    # T2: force safety — deep reference, governor on vs off
    Lg = simulate(target_pen=0.06, force_gov=True,  push=False)
    Lo = simulate(target_pen=0.06, force_gov=False, push=False)
    peak_gov = Lg["F_tissue"].max(); peak_nogov = Lo["F_tissue"].max()

    # T3: RCM vs gain
    rcm = {kr: simulate(k_rcm=kr)["rcm"].max() for kr in (1500.0, 6000.0, 15000.0)}

    # T4: sensor robustness — deep ref + force-sensor noise/bias + stick-slip trocar
    Lcl = simulate(target_pen=0.04, push=False)
    Lad = simulate(target_pen=0.04, push=False,
                   sensor_sigma=0.10, sensor_bias=0.05, stick_slip=True, mu_s=4.0)
    peak_clean = Lcl["F_tissue"].max(); peak_adv = Lad["F_tissue"].max()
    ke_clean = Lcl["ke_hat"][-1]; ke_adv = Lad["ke_hat"][-1]

    print("\n"+"="*64)
    print("T1  Adaptive Hunt–Crossley RLS")
    print(f"    k_e estimate (true {KE_TRUE:.0f}) : {ke_final:.1f} N/m  "
          f"({ke_err:.1f}% error)  -> {'PASS' if ke_err<10 else 'FAIL'}")
    print(f"    no-contact drift / cov growth : {ke_drift:.2e} N/m, "
          f"x{G_growth:.2f}  -> {'PASS (no windup)' if ke_drift<1 and G_growth<5 else 'FAIL'}")
    print("T2  Hard force safety (deep 6 cm reference)")
    print(f"    peak force  governor ON : {peak_gov:.2f} N  (limit 3.0 N)  "
          f"-> {'PASS' if peak_gov<3.6 else 'FAIL'}")
    print(f"    peak force  governor OFF: {peak_nogov:.1f} N  (push-through)")
    print(f"    reduction : {(1-peak_gov/peak_nogov)*100:.0f}%")
    print("T3  RCM lateral excursion vs gain (5 N push)")
    for kr,v in rcm.items():
        print(f"    k_rcm={kr:>7.0f} : {v:.3f} mm"
              f"{'   (<0.5 mm)' if v<0.5 else ''}")
    print("T4  Sensor robustness (noise 0.1 N + bias 0.05 N + stick-slip trocar)")
    print(f"    peak force  ideal : {peak_clean:.2f} N   adverse : {peak_adv:.2f} N "
          f" -> {'PASS' if peak_adv<peak_clean+0.6 else 'FAIL'}")
    print(f"    k_e estimate ideal: {ke_clean:.1f}     adverse : {ke_adv:.1f} N/m "
          f" -> {'PASS' if abs(ke_adv-500)/500<0.15 else 'FAIL'}")
    print("="*64)

    # figure
    fig, ax = plt.subplots(1,4, figsize=(19,4.2))
    fig.suptitle("Laparoscopy verification: RLS, force safety, RCM, sensor robustness",
                 fontweight="bold")
    ax[0].plot(Lc["t"], Lc["ke_hat"], label="in contact")
    ax[0].plot(Ln["t"], Ln["ke_hat"], '--', label="no contact (gated)")
    ax[0].axhline(KE_TRUE, color='k', ls=':', label="true $k_e$")
    ax[0].set_title("T1  RLS $\\hat k_e$"); ax[0].set_xlabel("t (s)")
    ax[0].set_ylabel("N/m"); ax[0].legend(fontsize=7); ax[0].grid(alpha=0.3)
    ax[1].plot(Lg["t"], Lg["F_tissue"], label="governor ON")
    ax[1].plot(Lo["t"], Lo["F_tissue"], label="governor OFF")
    ax[1].axhline(3.0, color='k', ls=':', label="$F_{max}$=3 N")
    ax[1].set_title("T2  contact force, deep reference"); ax[1].set_xlabel("t (s)")
    ax[1].set_ylabel("N"); ax[1].legend(fontsize=7); ax[1].grid(alpha=0.3)
    ax[2].bar([str(int(k)) for k in rcm], list(rcm.values()), color="tab:purple")
    ax[2].axhline(0.5, color='k', ls=':', label="0.5 mm band")
    ax[2].set_title("T3  max RCM vs $k_{rcm}$"); ax[2].set_xlabel("$k_{rcm}$")
    ax[2].set_ylabel("mm"); ax[2].legend(fontsize=7); ax[2].grid(axis='y', alpha=0.3)
    ax[3].plot(Lcl["t"], Lcl["F_tissue"], label="ideal sensor")
    ax[3].plot(Lad["t"], Lad["F_tissue"], label="noise+bias+stick-slip", alpha=0.8)
    ax[3].axhline(3.0, color='k', ls=':', label="$F_{max}$=3 N")
    ax[3].set_title("T4  force under sensor noise"); ax[3].set_xlabel("t (s)")
    ax[3].set_ylabel("N"); ax[3].legend(fontsize=7); ax[3].grid(alpha=0.3)
    fig.tight_layout(rect=[0,0,1,0.93])
    out = Path(__file__).parent/"lap_verify.png"; fig.savefig(out, dpi=150)
    print(f"[plot] saved -> {out}")


if __name__ == "__main__":
    main()
