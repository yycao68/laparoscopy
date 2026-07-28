"""
RCM-constrained tool-tip Impedance MPC for laparoscopy.

Three controllers sharing the same Layer-1 feedforward, orientation hold,
and null-space RCM regulation, differing only in the tip-force law:

  * 'impedance'   : static Cartesian impedance  F = K_d e + D_d e_dot   (baseline)
  * 'mpc'         : constant-A_d receding-horizon QP, no disturbance estimate
  * 'mpc_kalman'  : QP + augmented Kalman with an integrating disturbance state
                    AND a respiration oscillator internal model (Section 4.4.1),
                    so the periodic disturbance is *predicted* over the horizon.

The QP is the same constant-A_d double integrator as the pHRI architecture;
only B_d depends on configuration through the tip operational-space inertia.
"""

from __future__ import annotations
import sys
from pathlib import Path
from dataclasses import dataclass, field
import numpy as np
from numpy.linalg import inv
import scipy.sparse as sp

FR3_SIM = Path(__file__).resolve().parents[2] / "fr3_impedance" / "simulation"
sys.path.insert(0, str(FR3_SIM))
from fr3_impedance import build_operational_space_model     # noqa: E402
from so3_utils import rotation_error_matrix                 # noqa: E402


@dataclass
class LapCtrlParams:
    N:        int = 10
    dt_mpc:   float = 0.01            # 100 Hz QP
    Q_pos:    float = 2.0e4
    Q_vel:    float = 50.0
    Q_f:      float = 5.0             # terminal scale
    R_u:      float = 1.0e-6
    F_max:    float = 8.0             # control-force bound (N)
    # static-impedance baseline gains
    k_imp:    float = 500.0
    d_imp:    float = 45.0
    # orientation hold
    K_rot:    float = 20.0
    D_rot:    float = 6.0
    # null-space RCM regulation + posture
    k_rcm:    float = 1500.0          # RCM stiffness (proposed; baseline lowers it)
    k_c:      float = 8.0             # posture centering
    d_null:   float = 3.0
    # Kalman
    Q_proc_d: float = 20.0
    Q_proc_r: float = 5.0e2           # oscillator process noise
    R_obs_e:  float = 1.0e-3
    R_obs_de: float = 1.0e-2
    resp_f:   float = 0.20            # respiration freq for the oscillator (Hz)
    # --- adaptive Hunt--Crossley tissue estimator (Section 4) ---
    enable_rls:   bool  = True
    n_hc:         float = 1.5         # known HC exponent
    rls_forget:   float = 0.995       # forgetting factor
    rls_dead:     float = 0.05        # dead-zone on the prediction residual (N)
    rls_pe_delta: float = 1.0e-3      # min penetration to enable adaptation (m)
    rls_cov_max:  float = 1.0e6       # covariance upper bound (anti-windup)
    ke_prior:     float = 300.0       # prior stiffness (N/m^n)
    ke_box:       tuple = (50.0, 3000.0)
    b_box:        tuple = (0.0, 1.0)
    # --- hard force-safety governor (Section 5, eq. 10c / 15a) ---
    enable_force_gov: bool = True
    F_tissue_max:     float = 3.0     # tissue-damage force threshold (N)


class LapController:
    def __init__(self, mode: str, env, params: LapCtrlParams | None = None,
                 use_oscillator: bool = True):
        assert mode in ("impedance", "mpc", "mpc_kalman")
        self.mode = mode
        self.env = env
        self.p = params or LapCtrlParams()
        self.use_osc = use_oscillator
        self.q0 = env.env.q.copy()
        dt = self.p.dt_mpc
        N = self.p.N

        # constant A_d (6x6 double integrator)
        self.A_d = np.block([[np.eye(3), dt*np.eye(3)],
                             [np.zeros((3, 3)), np.eye(3)]])
        self._Adp = [np.eye(6)]
        for _ in range(N - 1):
            self._Adp.append(self._Adp[-1] @ self.A_d)
        self.Phi = np.vstack([self._Adp[k] @ self.A_d for k in range(N)])

        Qblk = np.diag(np.concatenate([self.p.Q_pos*np.ones(3),
                                       self.p.Q_vel*np.ones(3)]))
        self.Q_bar = np.zeros((6*N, 6*N))
        for i in range(N-1):
            self.Q_bar[6*i:6*i+6, 6*i:6*i+6] = Qblk
        self.Q_bar[6*(N-1):, 6*(N-1):] = self.p.Q_f * Qblk
        self.R_bar = self.p.R_u * np.eye(3*N)

        # respiration oscillator (rotation by w*dt each step)
        w = 2*np.pi*self.p.resp_f
        self.R_osc = np.array([[np.cos(w*dt),  np.sin(w*dt)],
                               [-np.sin(w*dt), np.cos(w*dt)]])
        self.n_s = env.s0.copy()                 # respiration acts along the shaft
        # C_r maps the oscillator state to a force-form disturbance; disabled
        # (zeroed) when use_oscillator=False -> pure random-walk Kalman.
        self.C_r = (np.outer(self.n_s, np.array([1.0, 0.0]))
                    if self.use_osc else np.zeros((3, 2)))

        self._osqp = None
        self.reset()

    def reset(self):
        self.x_aug = np.zeros(11)         # [e(3), edot(3), d_hat(3), x_r(2)]
        self.P_aug = np.eye(11)
        self._first = True
        self._F_prev = np.zeros(3)
        self._osqp = None
        # adaptive Hunt--Crossley RLS state:  th = [k_e, k_e*b]  (linear-in-params)
        self.th = np.array([self.p.ke_prior, 0.0])
        self.G_rls = np.diag([1.0e3, 1.0e3])
        self.ke_hat = self.p.ke_prior
        self.b_hat = 0.0

    @property
    def theta_hat(self):
        return np.array([self.ke_hat, self.b_hat])

    # ------------------------------------------------------------------
    # Adaptive Hunt--Crossley estimator (dead-zone RLS with PE gate + cov bound)
    # ------------------------------------------------------------------
    def _rls_update(self, f_meas, delta, ddelta):
        if not self.p.enable_rls or delta <= self.p.rls_pe_delta:
            return                                   # excitation gate: only in contact
        dn = delta**self.p.n_hc
        phi = np.array([dn, dn*ddelta])              # f = k_e*dn + (k_e b)*dn*ddot
        r = f_meas - phi @ self.th
        if abs(r) <= self.p.rls_dead:                # dead-zone: ignore noise-level residual
            return
        lam = self.p.rls_forget
        denom = lam + phi @ self.G_rls @ phi
        K = self.G_rls @ phi / denom
        self.th = self.th + K * r
        self.G_rls = (self.G_rls - np.outer(K, phi) @ self.G_rls) / lam
        # covariance upper bound (anti-windup) + symmetrise
        self.G_rls = 0.5*(self.G_rls + self.G_rls.T)
        w_, V_ = np.linalg.eigh(self.G_rls)
        w_ = np.clip(w_, 1e-3, self.p.rls_cov_max)
        self.G_rls = (V_ * w_) @ V_.T
        # projection into the parameter box
        ke = float(np.clip(self.th[0], *self.p.ke_box))
        b = float(np.clip(self.th[1]/max(ke, 1e-6), *self.p.b_box))
        self.ke_hat, self.b_hat = ke, b
        self.th[0] = ke
        self.th[1] = ke*b

    def _force_governor(self, p_d, ddelta=0.0):
        """Cap commanded penetration so the predicted contact force <= F_tissue_max
        (hard force-safety, eq. 10c via the RLS stiffness estimate). The
        Hunt--Crossley velocity term is folded into the cap so the *peak* force
        (not just the quasi-static value) honours the bound during approach."""
        if not self.p.enable_force_gov:
            return p_d
        surf = self.env.surface_point(self.env.time)
        s = self.env.s0
        delta_cmd = float((p_d - surf) @ s)
        if delta_cmd <= 0.0:
            return p_d
        k_eff = max(self.ke_hat, 1e-6) * (1.0 + self.b_hat*max(ddelta, 0.0))
        # include the along-normal Kalman disturbance offset n^T d_hat (eq. 10c):
        # the budget left for the structured term is F_tissue_max minus the offset
        offset = max(float(s @ self.x_aug[6:9]), 0.0)
        budget = max(self.p.F_tissue_max - offset, 0.0)
        delta_max = (budget / k_eff)**(1.0/self.p.n_hc)
        if delta_cmd > delta_max:
            return p_d - (delta_cmd - delta_max) * s      # back off along the normal
        return p_d

    # ------------------------------------------------------------------
    def _B_d(self, Lam_inv):
        return np.vstack([np.zeros((3, 3)), -Lam_inv * self.p.dt_mpc])

    def _build_Gamma(self, Lam_inv):
        N = self.p.N
        Bd = self._B_d(Lam_inv)
        G = np.zeros((6*N, 3*N))
        for i in range(N):
            for j in range(i+1):
                G[6*i:6*i+6, 3*j:3*j+3] = self._Adp[i-j] @ Bd
        return G

    def _A_aug(self, Lam_inv):
        Bd = self._B_d(Lam_inv)
        A = np.zeros((11, 11))
        A[:6, :6] = self.A_d
        A[:6, 6:9] = Bd
        A[:6, 9:11] = Bd @ self.C_r
        A[6:9, 6:9] = np.eye(3)
        A[9:11, 9:11] = self.R_osc
        return A

    def _kalman(self, e, de, Lam_inv):
        Bd = self._B_d(Lam_inv)
        A = self._A_aug(Lam_inv)
        B = np.zeros((11, 3)); B[:6] = Bd
        C = np.zeros((6, 11)); C[:6, :6] = np.eye(6)
        Q = np.zeros((11, 11))
        Q[6:9, 6:9] = self.p.Q_proc_d*np.eye(3)
        Q[9:11, 9:11] = (self.p.Q_proc_r if self.use_osc else 0.0)*np.eye(2)
        Rm = np.diag(np.concatenate([self.p.R_obs_e*np.ones(3),
                                     self.p.R_obs_de*np.ones(3)]))
        if self._first:
            self.x_aug[:3] = e; self.x_aug[3:6] = de
            if self.use_osc:
                self.x_aug[9:11] = np.array([1e-3, 0.0])    # seed oscillator
            self._first = False
        xp = A @ self.x_aug + B @ self._F_prev
        Pp = A @ self.P_aug @ A.T + Q
        y = np.concatenate([e, de])
        S = C @ Pp @ C.T + Rm
        K = Pp @ C.T @ inv(S)
        self.x_aug = xp + K @ (y - C @ xp)
        self.P_aug = (np.eye(11) - K @ C) @ Pp
        return A

    def _solve_qp(self, H, h, ff=None):
        # Box bounds the TOTAL applied tip force  ff + F_mpc(k)  in [-F_max, F_max]
        # (eq. 10a), where ff = Lambda(q) ddot p_d is the known feedforward tip
        # force; shifting the box by -ff keeps the constraint affine in F_mpc.
        n = 3*self.p.N
        P = sp.triu(H, format='csc')
        shift = np.zeros(n) if ff is None else np.tile(ff, self.p.N)
        lb = -self.p.F_max*np.ones(n) - shift
        ub = self.p.F_max*np.ones(n) - shift
        if self._osqp is None:
            import osqp
            self._osqp = osqp.OSQP()
            A = sp.eye(n, format='csc')
            self._osqp.setup(P, h, A, lb, ub, verbose=False, warm_starting=True,
                             eps_abs=1e-6, eps_rel=1e-6, max_iter=4000, polish=False)
        else:
            self._osqp.update(Px=P.data, q=h, l=lb, u=ub)
        r = self._osqp.solve()
        if r.info.status_val not in (1, 2):
            return np.zeros(n)
        return r.x

    # ------------------------------------------------------------------
    def control(self, dyn, st, p_d, dp_d, ddp_d, R_d, p_rcm, resolve=True):
        N = self.p.N
        p_tip, s_hat, v_tip, J_tip = self.env.tip_state(dyn, st)
        J_w = dyn.J[3:, :]
        M_inv = inv(dyn.M)
        Lam_inv = J_tip @ M_inv @ J_tip.T + 1e-6*np.eye(3)
        Lam = inv(Lam_inv)

        # ---- adaptive Hunt--Crossley estimator (read-only contact sensing) ----
        f_meas, delta_now, ddelta_now = self.env.contact_probe(p_tip, v_tip, self.env.time)
        if resolve:
            self._rls_update(f_meas, delta_now, ddelta_now)

        # ---- hard force-safety governor: cap commanded penetration (eq. 10c) --
        p_d_eff = (self._force_governor(p_d, ddelta_now)
                   if self.mode != "impedance" else p_d)

        e = p_d_eff - p_tip
        de = dp_d - v_tip

        # ---- tip-force law (QP/Kalman only on MPC ticks) -----------------
        if self.mode == "impedance":
            F = self.p.k_imp*e + self.p.d_imp*de
            d_hat = np.zeros(3)
        elif not resolve:
            F = self._F_prev.copy()
            d_hat = self.x_aug[6:9]
        else:
            if self.mode == "mpc_kalman":
                A = self._kalman(e, de, Lam_inv)
                # free response = augmented free evolution projected to [e,edot]
                xi = self.x_aug.copy()
                x_free = np.zeros(6*N)
                for i in range(N):
                    xi = A @ xi
                    x_free[6*i:6*i+6] = xi[:6]
                d_hat = self.x_aug[6:9]
            else:  # 'mpc' : no estimator
                x_e = np.concatenate([e, de])
                x_free = self.Phi @ x_e
                d_hat = np.zeros(3)
            Gamma = self._build_Gamma(Lam_inv)
            H = Gamma.T @ self.Q_bar @ Gamma + self.R_bar
            H = 0.5*(H + H.T)
            h = Gamma.T @ self.Q_bar @ x_free
            ff = Lam @ ddp_d                 # feedforward tip force (eq. 10a)
            u = self._solve_qp(H, h, ff=ff)
            F = u[:3]
            self._F_prev = F.copy()

        # ---- Layer 1 feedforward + tip-force torque ----------------------
        tau_ff = dyn.Cq_dot + J_tip.T @ (Lam @ ddp_d)
        tau_mpc = J_tip.T @ F

        # ---- RCM + posture in the tip-task null space --------------------
        # No full orientation hold: in laparoscopy the tool orientation is
        # *determined* by the RCM (the shaft must pass through the trocar), so
        # the orientation DOF are left to the RCM regulator rather than pinned
        # to R_d. Only a light roll-about-shaft damping is kept (via d_null).
        os = build_operational_space_model(dyn, st.ee_vel)
        e_rcm = self.env.rcm_error(st, p_rcm)
        J_rcm = self.env.rcm_jacobian(dyn, st, p_rcm)
        tau_null = os.N_bar.T @ (-self.p.k_rcm * (J_rcm.T @ e_rcm)
                                 - self.p.k_c*(st.q - self.q0)
                                 - self.p.d_null*st.dq)

        tau = tau_ff + tau_mpc + tau_null
        info = dict(F=F, d_hat=d_hat, e_rcm=e_rcm, p_tip=p_tip,
                    ke_hat=self.ke_hat, b_hat=self.b_hat,
                    delta_max=(self.p.F_tissue_max/max(self.ke_hat, 1e-6))**(1.0/self.p.n_hc))
        return tau, info
