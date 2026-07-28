"""
Genuine common-Lyapunov H-infinity synthesis over the LPV polytope (paper Eq. 14).

The per-vertex gamma reported by lap_hinf_design.py is a *surrogate*: it solves an
independent Riccati at each vertex, giving a different storage P_i per vertex, which
does NOT certify robust stability over the polytope. This script instead solves the
single SDP

    min gamma  s.t.  X >> 0,  and at every vertex v
        [ -X        (A X + B_v W)^T   0        (Cz X + Dz W)^T ]
        [ A X+B_v W  -X               B_v      0               ]  << 0
        [ 0          B_v^T            -gamma I  0               ]
        [ Cz X+Dz W  0                0        -gamma I         ]

for a COMMON (X, W) and a single gamma, with K = W X^{-1}, P = X^{-1}. Convexity in
the theta-affine data then certifies ||T_zw||_inf <= gamma for every point of the
polytope, not just the vertices (paper Section V-B).

The polytope here is the operational-space inertia spread B_d(rho) = -Lambda^{-1} dt
(the genuine LPV parameter of the tip plant), scaled over a 4x range, matching the
'gamma vs inertia scale' surrogate reported in Section VI-F.

Run:  python3 lap_commonP_sdp.py
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from numpy.linalg import inv, eigvals
from scipy.linalg import sqrtm
import cvxpy as cp

import sys
sys.path.insert(0, str(Path(__file__).parent))
# The FR3 MuJoCo bridge moved to ../../pHRI/simulation (project renamed from
# fr3_impedance); make it importable before lap_env's stale hard-coded path runs.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pHRI" / "simulation"))
from lap_env import LapEnv
from lap_controller import LapCtrlParams
from lap_hinf_design import tip_plant


def hinf_norm(Acl, Bw, Ccl, Npts=3000):
    """True discrete-time H-infinity norm ||Ccl (zI - Acl)^{-1} Bw||_inf by
    fine frequency sweep over the unit circle (D-feedthrough is zero here)."""
    n = Acl.shape[0]
    best = 0.0
    for th in np.linspace(0.0, np.pi, Npts):
        z = np.exp(1j * th)
        T = Ccl @ np.linalg.solve(z * np.eye(n) - Acl, Bw)
        best = max(best, np.linalg.svd(T, compute_uv=False)[0])
    return float(best)


def _feasible_at(A, B_vertices, Cz, Dz, gamma, eps=1e-6):
    """STRICT feasibility of the common-(X, W) bounded-real LMI at a fixed gamma.

    The min-gamma objective is numerically fragile for first-order conic solvers
    on this block LMI; a fixed-gamma *feasibility* problem solves cleanly. The
    LMI block (A X + B_v W) certifies the closed loop A + B_v K (K = W X^{-1})
    Schur. We accept a solve only if CLARABEL returns status 'optimal' AND the
    LMI is *independently* verified strictly negative-definite at every vertex
    (the gamma=1.46 boundary returns 'optimal_inaccurate' with a +4e-7 residual,
    so the strict check is what makes the certificate trustworthy).
    """
    n = A.shape[0]
    m = B_vertices[0].shape[1]
    nz = Cz.shape[0]
    nw = m
    X = cp.Variable((n, n), symmetric=True)
    W = cp.Variable((m, n))
    cons = [X >> eps * np.eye(n)]
    for B in B_vertices:
        AXBW = A @ X + B @ W
        CXDW = Cz @ X + Dz @ W
        M = cp.bmat([
            [-X,                AXBW.T,            np.zeros((n, nw)),  CXDW.T],
            [AXBW,              -X,                B,                  np.zeros((n, nz))],
            [np.zeros((nw, n)), B.T,               -gamma*np.eye(nw),  np.zeros((nw, nz))],
            [CXDW,              np.zeros((nz, n)), np.zeros((nz, nw)), -gamma*np.eye(nz)],
        ])
        cons.append(M << 0)
    prob = cp.Problem(cp.Minimize(0), cons)
    try:
        prob.solve(solver="CLARABEL")
    except cp.error.SolverError:
        return None
    if X.value is None or prob.status != "optimal":
        return None
    Xv, K = X.value, W.value @ inv(X.value)
    # independent strict LMI-residual check (max eigenvalue must be < 0)
    max_eig = -np.inf
    for B in B_vertices:
        AXBW = A @ Xv + B @ K @ Xv
        CXDW = Cz @ Xv + Dz @ K @ Xv
        M = np.block([
            [-Xv,               AXBW.T,            np.zeros((n, nw)),  CXDW.T],
            [AXBW,              -Xv,               B,                  np.zeros((n, nz))],
            [np.zeros((nw, n)), B.T,               -gamma*np.eye(nw),  np.zeros((nw, nz))],
            [CXDW,              np.zeros((nz, n)), np.zeros((nz, nw)), -gamma*np.eye(nz)],
        ])
        max_eig = max(max_eig, float(np.max(eigvals(M).real)))
    if max_eig >= -1e-9:
        return None
    rho = max(max(abs(eigvals(A + B @ K))) for B in B_vertices)
    if rho >= 1.0 - 1e-7:
        return None
    return dict(K=K, X=Xv, rho=float(rho), lmi_max_eig=max_eig)


def common_p_hinf_sdp(A, B_vertices, Q, R):
    """Smallest *strictly* feasible common-Lyapunov gamma over a descending grid."""
    n = A.shape[0]
    m = B_vertices[0].shape[1]
    Cz = np.vstack([sqrtm(Q).real, np.zeros((m, n))])      # weights tracking error
    Dz = np.vstack([np.zeros((n, m)), sqrtm(R).real])      # weights applied force

    grid = [3.0, 2.5, 2.0, 1.8, 1.7, 1.6, 1.55, 1.5, 1.49, 1.48, 1.47]
    best = None
    for gamma in grid:
        r = _feasible_at(A, B_vertices, Cz, Dz, gamma)
        if r is not None:
            best = dict(gamma=gamma, **r)      # keep the smallest strictly-feasible
    return best


def main():
    env = LapEnv()
    p = LapCtrlParams()
    A, B, Lam_inv = tip_plant(env, p)
    dt = p.dt_mpc

    # Same performance weights as the per-vertex H-inf design (lap_hinf_design.py)
    Q = np.diag(np.concatenate([1.0e2 * np.ones(3), 1.0e0 * np.ones(3)]))
    R = 1.0 * np.eye(3)

    # Polytope vertices: operational-space inertia scaled over a 4x range.
    scales = (0.5, 1.0, 2.0)
    B_vertices = [np.vstack([np.zeros((3, 3)), -(Lam_inv * s) * dt]) for s in scales]

    res = common_p_hinf_sdp(A, B_vertices, Q, R)
    out = {}
    if res is None:
        print("common-P SDP infeasible")
        return
    K, X, gamma = res["K"], res["X"], res["gamma"]
    P = inv(X)
    Cz = np.vstack([sqrtm(Q).real, np.zeros((3, 6))])
    Dz = np.vstack([np.zeros((6, 3)), sqrtm(R).real])
    # spectral radius and TRUE Hinf norm of the SINGLE common gain at every vertex
    sr, tn = {}, {}
    for s, Bv in zip(scales, B_vertices):
        Acl = A + Bv @ K
        sr[f"x{s}"] = float(max(abs(eigvals(Acl))))
        tn[f"x{s}"] = hinf_norm(Acl, Bv, Cz + Dz @ K)
    out["common_P_hinf"] = dict(
        gamma_certified=float(gamma),
        lmi_max_eig=float(res["lmi_max_eig"]),
        true_hinf_norm_per_vertex=tn,
        true_hinf_norm_max=float(max(tn.values())),
        spec_radius_per_vertex=sr,
        spec_radius_max=float(max(sr.values())),
        K_norm=float(np.linalg.norm(K)),
        P_cond=float(np.linalg.cond(P)),
        vertices_inertia_scale=list(scales),
    )

    print("\n== Common-Lyapunov H-inf synthesis over the inertia polytope (Eq. 14) ==")
    print(f"  vertices (Lambda^-1 scale): {scales}")
    print(f"  smallest STRICTLY-feasible common-P gamma = {gamma:.3f}  "
          f"(LMI max-eig {res['lmi_max_eig']:+.1e} < 0)")
    print(f"  ONE gain K, ||K|| = {np.linalg.norm(K):.2f},  cond(P) = {np.linalg.cond(P):.1f}")
    print("  per vertex:   rho(A+BK)    true ||T_zw||_inf (freq sweep)")
    for k in sr:
        print(f"     {k:>5}:     {sr[k]:.4f}        {tn[k]:.4f}")
    print(f"  max over vertices: rho = {max(sr.values()):.4f} (<1, robustly stable),"
          f"  ||T||_inf = {max(tn.values()):.4f} (<= gamma)")
    print("  -> a single storage P = X^-1 certifies ||T_zw||_inf <= gamma over the")
    print("     whole polytope; the achieved norm ~1.1 is well-damped, unlike the")
    print("     per-vertex minimal-gamma~1.0 design (marginally stabilising, rho~1).")

    (Path(__file__).parent / "lap_commonP_sdp.json").write_text(json.dumps(out, indent=2))
    print(f"\n[json] saved -> {Path(__file__).parent / 'lap_commonP_sdp.json'}")


if __name__ == "__main__":
    main()
