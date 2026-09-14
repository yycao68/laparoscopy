"""
Offline H-infinity / H2 control design for the post-feedforward tip plant
(Section 6 of the paper).

After Layer-1 cancellation the residual is the constant-A_d double integrator
driven by the force-form disturbance through the same input as the control:

    x_e(k+1) = A_d x_e(k) + B_d (F + w),   z = [Q^{1/2} x_e ; R^{1/2} F].

This script:
  1. builds (A_d, B_d) at a nominal configuration (FR3 home, tip frame);
  2. solves the H2 (LQR) design via the DARE  ->  gain K2, storage P2;
  3. numerically verifies Theorem 1: with the MPC terminal cost set to P2,
     the *unconstrained* receding-horizon first-move gain equals the
     infinite-horizon LQR gain for any horizon N;
  4. solves the H-infinity state-feedback design by gamma-iteration on the
     discrete H-infinity Riccati (no SDP solver needed) -> gain Kinf, gamma;
  5. reports the local induced-gain diagnostic gamma;
  6. checks robustness by re-solving across a spread of operational-space
     inertia (the genuine LPV parameter of the tip plant).

For the common-storage LMI certificate used in the paper, run
lap_commonP_sdp.py after this script.

Run:  python3 lap_hinf_design.py
"""

from __future__ import annotations
import sys, json
from pathlib import Path
import numpy as np
from numpy.linalg import inv, eigvals
from scipy.linalg import solve_discrete_are


# ----------------------------------------------------------------------
def tip_plant(env, params):
    """Constant A_d and nominal B_d (force-form) at the FR3 home pose."""
    dyn, st = env.get_dynamics_and_state()
    _, _, _, J_tip = env.tip_state(dyn, st)
    Lam_inv = J_tip @ inv(dyn.M) @ J_tip.T + 1e-6 * np.eye(3)
    dt = params.dt_mpc
    A = np.block([[np.eye(3), dt*np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
    B = np.vstack([-0.5 * dt * dt * Lam_inv, -Lam_inv * dt])
    return A, B, Lam_inv


def weights(params):
    Q = np.diag(np.concatenate([params.Q_pos*np.ones(3), params.Q_vel*np.ones(3)]))
    R = params.R_force * np.eye(3)
    return Q, R


# ----------------------------------------------------------------------
def h2_design(A, B, Q, R):
    """LQR / H2 state feedback via the DARE."""
    P = solve_discrete_are(A, B, Q, R)
    K = inv(R + B.T @ P @ B) @ (B.T @ P @ A)
    return K, P


def mpc_unconstrained_gain(A, B, Q, R, N, P_terminal):
    """First-move gain of the unconstrained finite-horizon QP with terminal cost
    P_terminal. Builds Phi, Gamma, Qbar, Rbar exactly as the MPC does."""
    Adp = [np.eye(6)]
    for _ in range(N-1):
        Adp.append(Adp[-1] @ A)
    Phi = np.vstack([Adp[k] @ A for k in range(N)])
    Gam = np.zeros((6*N, 3*N))
    for i in range(N):
        for j in range(i+1):
            Gam[6*i:6*i+6, 3*j:3*j+3] = Adp[i-j] @ B
    Qbar = np.zeros((6*N, 6*N))
    for i in range(N-1):
        Qbar[6*i:6*i+6, 6*i:6*i+6] = Q
    Qbar[6*(N-1):, 6*(N-1):] = P_terminal           # terminal cost = P
    Rbar = np.kron(np.eye(N), R)
    H = Gam.T @ Qbar @ Gam + Rbar
    # u* = -H^{-1} Gam^T Qbar Phi x_e ; first-move gain = first 3 rows
    G = inv(H) @ (Gam.T @ Qbar @ Phi)
    return G[:3, :]


# ----------------------------------------------------------------------
def hinf_riccati(A, B, E, Q, R, gamma, iters=20000, tol=1e-10):
    """Discrete H-infinity full-information Riccati by value iteration.

    Combined input [u; w] with cost blkdiag(R, -gamma^2 I). Returns (X, ok).
    ok=False if the recursion diverges or the indefiniteness/positivity
    conditions fail (=> gamma infeasible).
    """
    n = A.shape[0]
    Bc = np.hstack([B, E])
    m = B.shape[1]
    Rc = np.block([[R, np.zeros((m, E.shape[1]))],
                   [np.zeros((E.shape[1], m)), -gamma**2 * np.eye(E.shape[1])]])
    X = Q.copy()
    for _ in range(iters):
        M = Rc + Bc.T @ X @ Bc
        # required inertia: (u,u) block pos-def, (w,w) Schur neg-def
        Ruu = R + B.T @ X @ B
        Sww = (-gamma**2*np.eye(E.shape[1]) + E.T @ X @ E
               - E.T @ X @ B @ inv(Ruu) @ B.T @ X @ E)
        if np.min(eigvals(Ruu).real) <= 0 or np.max(eigvals(Sww).real) >= 0:
            return None, False
        try:
            Minv = inv(M)
        except np.linalg.LinAlgError:
            return None, False
        Xn = Q + A.T @ X @ A - (A.T @ X @ Bc) @ Minv @ (Bc.T @ X @ A)
        Xn = 0.5*(Xn + Xn.T)
        if not np.all(np.isfinite(Xn)) or np.max(np.abs(Xn)) > 1e14:
            return None, False
        if np.max(np.abs(Xn - X)) < tol:
            return Xn, True
        X = Xn
    return None, False


def _hinf_gain(A, B, E, X, R, gamma):
    Bc = np.hstack([B, E]); m = B.shape[1]
    Rc = np.block([[R, np.zeros((m, E.shape[1]))],
                   [np.zeros((E.shape[1], m)), -gamma**2*np.eye(E.shape[1])]])
    G = inv(Rc + Bc.T @ X @ Bc) @ (Bc.T @ X @ A)
    return G[:m, :]


def hinf_feasible(A, B, E, Q, R, gamma):
    """gamma is feasible iff the Riccati has a bounded solution with correct
    inertia AND the resulting state feedback is stabilizing."""
    X, ok = hinf_riccati(A, B, E, Q, R, gamma)
    if not ok:
        return False, None, None
    K = _hinf_gain(A, B, E, X, R, gamma)
    if max(abs(eigvals(A - B @ K))) < 1.0 - 1e-7:
        return True, K, X
    return False, None, None


def hinf_design(A, B, E, Q, R, gamma_hi=1e4, bisect=60):
    """Bisection for the minimal feasible gamma and the stabilizing H-inf gain."""
    ok, K_hi, X_hi = hinf_feasible(A, B, E, Q, R, gamma_hi)
    if not ok:
        return None, None, None
    lo, hi, best = 0.0, gamma_hi, (K_hi, X_hi)
    for _ in range(bisect):
        mid = 0.5*(lo + hi)
        ok, K, X = hinf_feasible(A, B, E, Q, R, mid)
        if ok:
            hi = mid; best = (K, X)
        else:
            lo = mid
    return hi, best[0], best[1]


# ----------------------------------------------------------------------
def main():
    sys.path.insert(0, str(Path(__file__).parent))
    from lap_env import LapEnv
    from lap_controller import LapCtrlParams

    env = LapEnv()
    p = LapCtrlParams()
    A, B, Lam_inv = tip_plant(env, p)
    Q, R = weights(p)
    out = {}

    # --- H2 / LQR ---
    K2, P2 = h2_design(A, B, Q, R)
    Acl2 = A - B @ K2
    out["H2"] = dict(spec_radius=float(max(abs(eigvals(Acl2)))),
                     K_norm=float(np.linalg.norm(K2)))

    # --- Theorem 1 check: MPC unconstrained gain == LQR gain (terminal cost P2) ---
    Kmpc = mpc_unconstrained_gain(A, B, Q, R, p.N, P2)
    thm1_err = float(np.max(np.abs(Kmpc - K2)))
    out["theorem1_gain_match_maxabs"] = thm1_err

    # --- H-infinity force-safety design --------------------------------
    # The disturbance is the force-form residual and shares the control
    # channel (E = B).  The contact-force output is the local Hunt--Crossley
    # linearization about the 15 mm palpation depth:
    #   df = -k*n*delta0^(n-1) s^T de_position.
    # Including C_f^T C_f in Q makes the computed norm a local
    # disturbance-to-contact-force certificate, not merely a control-effort
    # certificate.
    E = B.copy()
    delta0 = 0.015
    contact_slope = env.tis.k_e * env.tis.n_hc * delta0 ** (env.tis.n_hc - 1.0)
    C_f = np.zeros((1, 6))
    C_f[0, :3] = -contact_slope * env.s0
    Q_hinf = np.diag(np.concatenate([1.0e2*np.ones(3), 1.0e0*np.ones(3)]))
    Q_hinf += C_f.T @ C_f
    R_hinf = 1.0 * np.eye(3)                       # unit force penalty (N)
    gamma, Kinf, Xinf = hinf_design(A, B, E, Q_hinf, R_hinf)
    Acl_inf = A - B @ Kinf
    wbar = 1.0  # N, representative force-form residual bound (see Section 6.4)
    # minimal-gamma H-inf is marginally stabilising (poles on the unit circle);
    # deployment uses gamma slightly above gamma_min for damped poles.
    gamma_use = 1.5 * gamma
    _, K_use, _ = hinf_feasible(A, B, E, Q_hinf, R_hinf, gamma_use)
    sr_use = float(max(abs(eigvals(A - B @ K_use)))) if K_use is not None else None
    out["Hinf"] = dict(gamma_min=float(gamma),
                       spec_radius_at_gamma_min=float(max(abs(eigvals(Acl_inf)))),
                       gamma_deploy=float(gamma_use),
                       spec_radius_deploy=sr_use,
                       K_norm=float(np.linalg.norm(Kinf)),
                       reference_disturbance_norm=wbar,
                       local_output_bound=float(gamma*wbar))

    # --- robustness over operational-space inertia spread (true LPV param) ---
    gammas = {}
    for scale in (0.5, 1.0, 2.0):
        Bs = np.vstack([
            -0.5 * p.dt_mpc**2 * (Lam_inv * scale),
            -(Lam_inv * scale) * p.dt_mpc,
        ])
        g, _, _ = hinf_design(A, Bs, Bs, Q_hinf, R_hinf)
        gammas[f"x{scale}"] = None if g is None else float(g)
    out["Hinf_gamma_vs_inertia_scale"] = gammas
    out["contact_linearization"] = dict(
        delta0_m=delta0,
        k_e=float(env.tis.k_e),
        exponent=float(env.tis.n_hc),
        slope_N_per_m=float(contact_slope),
    )

    # ---- report ----
    print("\n== Tip plant (FR3 home) ==")
    print(f"  ||Lambda^-1|| = {np.linalg.norm(Lam_inv):.3f},  dt = {p.dt_mpc*1e3:.0f} ms")
    print("\n== H2 / LQR design ==")
    print(f"  closed-loop spectral radius = {out['H2']['spec_radius']:.4f}  (stable < 1)")
    print(f"  ||K2|| = {out['H2']['K_norm']:.3e}")
    print("\n== Theorem 1 (impedance equivalence) numerical check ==")
    print(f"  max| K_MPC(unconstrained, Qf=P2) - K_LQR | = {thm1_err:.2e}")
    print("  -> unconstrained MPC == classical impedance/LQR gain (Theorem 1).")
    print("\n== Local H-infinity contact-channel diagnostic ==")
    print(f"  minimal gamma (||T_zw||_inf) = {out['Hinf']['gamma_min']:.4f}  "
          f"(spec. radius {out['Hinf']['spec_radius_at_gamma_min']:.4f}, marginal)")
    print(f"  deploy gamma = 1.5 gamma_min = {out['Hinf']['gamma_deploy']:.4f}  "
          f"(spec. radius {out['Hinf']['spec_radius_deploy']:.4f}, damped)")
    print(f"  local weighted-output bound gamma_min*wbar = "
          f"{out['Hinf']['local_output_bound']:.3f} for reference norm {wbar:.1f}")
    print("  (This is not a global nonlinear pointwise force guarantee.)")
    print("\n== gamma vs operational-space inertia scale (LPV robustness) ==")
    for k, v in gammas.items():
        print(f"  Lambda^-1 {k:>5}:  gamma = {v:.4f}" if v else f"  {k}: infeasible")
    print("  (per-vertex storage only; run lap_commonP_sdp.py for the common-P "
          "LMI certificate reported in the paper.)")

    (Path(__file__).parent / "lap_hinf_design.json").write_text(json.dumps(out, indent=2))
    print(f"\n[json] saved -> {Path(__file__).parent / 'lap_hinf_design.json'}")


if __name__ == "__main__":
    main()
