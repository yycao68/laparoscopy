"""Version-1 laparoscopic impedance MPC.

The full controller uses one convex, locally linear QP with stage input

    u = [F_tip (3), tau_rcm (7)].

The Cartesian force tracks the tool-tip reference while the joint-space
correction gives the optimizer direct authority over the RCM.  The prediction
state is [e, de, q-q0, dq].  Exact zero-order-hold input terms are used for both
double integrators.  Linear constraints enforce:

* Cartesian interaction-force effort;
* total commanded joint torque, including feedforward;
* a horizon-wide RCM band;
* a Hunt--Crossley penetration limit derived from the current RLS estimate.

The classical and tip-MPC modes are retained as measured baselines.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import time

import numpy as np
from numpy.linalg import inv
import scipy.sparse as sp
from scipy.linalg import solve_discrete_are

sys.path.insert(0, str(Path(__file__).parent))
from fr3_dependency import add_fr3_sim_to_path  # noqa: E402

add_fr3_sim_to_path("fr3_impedance")
from fr3_impedance import build_operational_space_model  # noqa: E402


@dataclass
class LapCtrlParams:
    N: int = 5
    dt_mpc: float = 0.01
    Q_pos: float = 2.0e4
    Q_vel: float = 50.0
    Q_joint: float = 2.0
    Q_joint_vel: float = 0.2
    R_force: float = 1.0e-6
    R_tau: float = 1.0e-6
    terminal_scale: float = 5.0
    F_max: float = 8.0
    rcm_band: float = 1.0e-4
    rcm_trust_radius: float = 2.0e-2
    force_tightening: float = 0.33
    actuator_delay: float = 0.0
    delay_fallback_threshold: float = 4.0e-3
    torque_filter_tau: float = 0.0
    torque_slew_rate: float = np.inf

    k_imp: float = 500.0
    d_imp: float = 45.0
    k_rcm_soft: float = 1500.0
    k_posture: float = 8.0
    d_null: float = 3.0
    force_pid_target: float = 0.5
    force_pid_kp: float = 5.0
    force_pid_ki: float = 10.0
    force_pid_integral_max: float = 0.5

    Q_proc_d: float = 20.0
    Q_proc_r: float = 5.0e2
    R_obs_e: float = 1.0e-3
    R_obs_de: float = 1.0e-2
    resp_f: float = 0.20

    enable_rls: bool = True
    n_hc: float = 1.5
    rls_forget: float = 0.995
    rls_dead: float = 0.05
    rls_pe_delta: float = 1.0e-3
    rls_cov_max: float = 1.0e6
    ke_prior: float = 300.0
    ke_box: tuple[float, float] = (50.0, 3000.0)
    b_box: tuple[float, float] = (0.0, 1.0)
    F_tissue_max: float = 3.0


class LapController:
    def __init__(
        self,
        mode: str,
        env,
        params: LapCtrlParams | None = None,
        use_oscillator: bool = True,
    ):
        assert mode in (
            "impedance",
            "pid_force",
            "mpc",
            "mpc_kalman",
            "mpc_kalman_soft",
        )
        self.mode = mode
        self.env = env
        self.p = params or LapCtrlParams()
        self.use_osc = use_oscillator
        self.q0 = env.env.q.copy()
        force_range = np.asarray(env.env.model.jnt_actfrcrange[:7], dtype=float)
        self.tau_min = force_range[:, 0]
        self.tau_max = force_range[:, 1]
        self.nx_tip = 6
        self.nx_full = 20
        self.nu_full = 10
        self._build_constant_models()
        self.reset()

    def _build_constant_models(self):
        dt = self.p.dt_mpc
        self.A_tip = np.block(
            [[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]]
        )
        self.A_full = np.zeros((20, 20))
        self.A_full[:6, :6] = self.A_tip
        self.A_full[6:13, 6:13] = np.eye(7)
        self.A_full[6:13, 13:20] = dt * np.eye(7)
        self.A_full[13:20, 13:20] = np.eye(7)

        self.Q_tip = np.diag(
            np.r_[self.p.Q_pos * np.ones(3), self.p.Q_vel * np.ones(3)]
        )
        self.Q_full = np.diag(
            np.r_[
                self.p.Q_pos * np.ones(3),
                self.p.Q_vel * np.ones(3),
                self.p.Q_joint * np.ones(7),
                self.p.Q_joint_vel * np.ones(7),
            ]
        )
        self.R_full = np.diag(
            np.r_[
                self.p.R_force * np.ones(3),
                self.p.R_tau * np.ones(7),
            ]
        )

        w = 2.0 * np.pi * self.p.resp_f
        self.R_osc = np.array(
            [
                [np.cos(w * dt), np.sin(w * dt)],
                [-np.sin(w * dt), np.cos(w * dt)],
            ]
        )
        self.C_r = (
            np.outer(self.env.s0, np.array([1.0, 0.0]))
            if self.use_osc
            else np.zeros((3, 2))
        )

    def reset(self):
        self.x_aug = np.zeros(11)
        self.P_aug = np.eye(11)
        self._first = True
        self._F_prev = np.zeros(3)
        self._tau_aux_prev = np.zeros(7)
        self._tau_cmd_prev = None
        self._force_pid_integral = 0.0
        self.th = np.array([self.p.ke_prior, 0.0])
        self.G_rls = np.diag([1.0e3, 1.0e3])
        self.ke_hat = self.p.ke_prior
        self.b_hat = 0.0
        self.solve_times_ms: list[float] = []
        self.last_qp_status = "not_run"
        self.last_force_prediction = 0.0
        self.last_rcm_prediction = 0.0
        self._solver_cache = {}
        self._terminal_cache = {}

    @property
    def theta_hat(self):
        return np.array([self.ke_hat, self.b_hat])

    def _rls_update(self, f_meas, delta, ddelta):
        if not self.p.enable_rls or delta <= self.p.rls_pe_delta:
            return
        dn = delta**self.p.n_hc
        phi = np.array([dn, dn * ddelta])
        residual = f_meas - phi @ self.th
        if abs(residual) <= self.p.rls_dead:
            return
        lam = self.p.rls_forget
        gain = self.G_rls @ phi / (lam + phi @ self.G_rls @ phi)
        self.th += gain * residual
        self.G_rls = (self.G_rls - np.outer(gain, phi) @ self.G_rls) / lam
        self.G_rls = 0.5 * (self.G_rls + self.G_rls.T)
        vals, vecs = np.linalg.eigh(self.G_rls)
        vals = np.clip(vals, 1.0e-3, self.p.rls_cov_max)
        self.G_rls = (vecs * vals) @ vecs.T
        ke = float(np.clip(self.th[0], *self.p.ke_box))
        b = float(np.clip(self.th[1] / max(ke, 1.0e-9), *self.p.b_box))
        self.ke_hat, self.b_hat = ke, b
        self.th[:] = (ke, ke * b)

    def _safe_reference(self, p_d, ddelta):
        """Feasibility backstop for references initialized inside tissue.

        The actual safety mechanism remains the horizon output row.  This
        prefilter only prevents an arbitrarily deep operator command from making
        the first linearized QP infeasible before the receding horizon can
        retreat.
        """
        surface = self.env.surface_point(self.env.time)
        delta_cmd = float((p_d - surface) @ self.env.s0)
        if delta_cmd <= 0.0:
            return p_d
        k_eff = max(self.ke_hat, 1.0e-6) * (1.0 + self.b_hat * max(ddelta, 0.0))
        budget = self.p.force_tightening * self.p.F_tissue_max
        delta_max = (budget / k_eff) ** (1.0 / self.p.n_hc)
        if delta_cmd <= delta_max:
            return p_d
        return p_d - (delta_cmd - delta_max) * self.env.s0

    @staticmethod
    def _prediction(A, B, N):
        nx, nu = A.shape[0], B.shape[1]
        Phi = np.zeros((nx * N, nx))
        Gamma = np.zeros((nx * N, nu * N))
        powers = [np.eye(nx)]
        for _ in range(N):
            powers.append(powers[-1] @ A)
        for i in range(N):
            Phi[nx * i : nx * (i + 1)] = powers[i + 1]
            for j in range(i + 1):
                Gamma[
                    nx * i : nx * (i + 1), nu * j : nu * (j + 1)
                ] = powers[i - j] @ B
        return Phi, Gamma

    def _B_tip_exact(self, Lam_inv):
        dt = self.p.dt_mpc
        return np.vstack(
            [-0.5 * dt * dt * Lam_inv, -dt * Lam_inv]
        )

    def _B_full_exact(self, J_tip, M_inv):
        dt = self.p.dt_mpc
        joint_map = M_inv @ np.hstack([J_tip.T, np.eye(7)])
        tip_map = J_tip @ joint_map
        B = np.zeros((20, 10))
        B[:3] = -0.5 * dt * dt * tip_map
        B[3:6] = -dt * tip_map
        B[6:13] = 0.5 * dt * dt * joint_map
        B[13:20] = dt * joint_map
        return B

    def _A_aug(self, Lam_inv):
        Bd = self._B_tip_exact(Lam_inv)
        A = np.zeros((11, 11))
        A[:6, :6] = self.A_tip
        A[:6, 6:9] = Bd
        A[:6, 9:11] = Bd @ self.C_r
        A[6:9, 6:9] = np.eye(3)
        A[9:11, 9:11] = self.R_osc
        return A

    def _kalman(self, e, de, Lam_inv):
        Bd = self._B_tip_exact(Lam_inv)
        A = self._A_aug(Lam_inv)
        B = np.zeros((11, 3))
        B[:6] = Bd
        C = np.zeros((6, 11))
        C[:6, :6] = np.eye(6)
        Q = np.zeros((11, 11))
        Q[6:9, 6:9] = self.p.Q_proc_d * np.eye(3)
        Q[9:11, 9:11] = (
            self.p.Q_proc_r if self.use_osc else 0.0
        ) * np.eye(2)
        Rm = np.diag(
            np.r_[
                self.p.R_obs_e * np.ones(3),
                self.p.R_obs_de * np.ones(3),
            ]
        )
        if self._first:
            self.x_aug[:3] = e
            self.x_aug[3:6] = de
            if self.use_osc:
                self.x_aug[9:11] = np.array([1.0e-3, 0.0])
            self._first = False
        xp = A @ self.x_aug + B @ self._F_prev
        Pp = A @ self.P_aug @ A.T + Q
        innovation = np.r_[e, de] - C @ xp
        S = C @ Pp @ C.T + Rm
        K = Pp @ C.T @ inv(S)
        self.x_aug = xp + K @ innovation
        self.P_aug = (np.eye(11) - K @ C) @ Pp
        return A

    def _solve_osqp(self, H, h, Acon, lower, upper):
        import osqp

        start = time.perf_counter()
        H = 0.5 * (H + H.T)
        key = (H.shape[0], Acon.shape[0])
        cached = self._solver_cache.get(key)
        if cached is None:
            Ppattern = sp.triu(
                sp.csc_matrix(np.ones(H.shape, dtype=float)), format="csc"
            )
            Apattern = sp.csc_matrix(np.ones(Acon.shape, dtype=float))
            solver = osqp.OSQP()
            solver.setup(
                Ppattern,
                h,
                Apattern,
                lower,
                upper,
                verbose=False,
                eps_abs=1.0e-4,
                eps_rel=1.0e-4,
                max_iter=1200,
                polishing=False,
                warm_starting=True,
            )
            cached = (solver, Ppattern)
            self._solver_cache[key] = cached
        solver, Ppattern = cached
        p_values = np.empty_like(Ppattern.data)
        for col in range(Ppattern.shape[1]):
            begin, end = Ppattern.indptr[col], Ppattern.indptr[col + 1]
            p_values[begin:end] = H[Ppattern.indices[begin:end], col]
        solver.update(
            Px=p_values,
            Ax=np.asarray(Acon, order="F").ravel(order="F"),
            q=h,
            l=lower,
            u=upper,
        )
        result = solver.solve(raise_error=False)
        elapsed = 1.0e3 * (time.perf_counter() - start)
        self.solve_times_ms.append(elapsed)
        self.last_qp_status = result.info.status
        if result.info.status_val not in (1, 2):
            return None
        return result.x

    def _tip_mpc(self, e, de, Lam_inv, d_free=None):
        N = self.p.N
        B = self._B_tip_exact(Lam_inv)
        Phi, Gamma = self._prediction(self.A_tip, B, N)
        x0 = np.r_[e, de]
        xfree = Phi @ x0 if d_free is None else d_free
        Qbar = np.kron(np.eye(N), self.Q_tip)
        key = "tip"
        Pterm = self._terminal_cache.get(key)
        if Pterm is None:
            Pterm = solve_discrete_are(
                self.A_tip, B, self.Q_tip, self.p.R_force * np.eye(3)
            )
            self._terminal_cache[key] = Pterm
        Qbar[-6:, -6:] = Pterm
        Rbar = np.kron(np.eye(N), self.p.R_force * np.eye(3))
        H = Gamma.T @ Qbar @ Gamma + Rbar
        h = Gamma.T @ Qbar @ xfree
        Acon = np.eye(3 * N)
        lb = -self.p.F_max * np.ones(3 * N)
        ub = self.p.F_max * np.ones(3 * N)
        sol = self._solve_osqp(H, h, Acon, lb, ub)
        return np.zeros(3) if sol is None else sol[:3]

    def _full_qp(
        self,
        e,
        de,
        st,
        p_d,
        p_rcm,
        J_tip,
        J_rcm,
        M_inv,
        tau_ff,
        d_free_tip,
    ):
        N, nx, nu = self.p.N, self.nx_full, self.nu_full
        B = self._B_full_exact(J_tip, M_inv)
        Phi, Gamma = self._prediction(self.A_full, B, N)
        x0 = np.r_[e, de, st.q - self.q0, st.dq]
        xfree = Phi @ x0
        if d_free_tip is not None:
            for i in range(N):
                xfree[nx * i : nx * i + 6] = d_free_tip[6 * i : 6 * (i + 1)]

        Qbar = np.kron(np.eye(N), self.Q_full)
        Qbar[-nx:, -nx:] *= self.p.terminal_scale
        Rbar = np.kron(np.eye(N), self.R_full)
        H = Gamma.T @ Qbar @ Gamma + Rbar
        h = Gamma.T @ Qbar @ xfree

        rows, lower, upper = [], [], []
        identity = np.eye(nu * N)
        for i in range(N):
            for j in range(3):
                rows.append(identity[nu * i + j])
                lower.append(-self.p.F_max)
                upper.append(self.p.F_max)
            # The plant applies tau = tau_ff + J_tip^T F_mpc + tau_a.
            # Constrain this total command, not only the auxiliary correction.
            torque_map = np.hstack([J_tip.T, np.eye(7)])
            for j in range(7):
                row = np.zeros(nu * N)
                row[nu * i : nu * (i + 1)] = torque_map[j]
                rows.append(row)
                lower.append(self.tau_min[j] - tau_ff[j])
                upper.append(self.tau_max[j] - tau_ff[j])

        e_rcm = self.env.rcm_error(st, p_rcm)
        for i in range(N):
            block = slice(nx * i, nx * (i + 1))
            G_i = Gamma[block]
            phi_i = Phi[block]
            qfree = phi_i[6:13] @ x0
            qmap = G_i[6:13]
            q_offset = self.q0 + qfree - st.q
            for joint in range(7):
                rows.append(qmap[joint])
                lower.append(-self.p.rcm_trust_radius - q_offset[joint])
                upper.append(self.p.rcm_trust_radius - q_offset[joint])
            rcm_offset = e_rcm + J_rcm @ (self.q0 + qfree - st.q)
            rcm_map = J_rcm @ qmap
            for axis in range(3):
                rows.append(rcm_map[axis])
                lower.append(-self.p.rcm_band - rcm_offset[axis])
                upper.append(self.p.rcm_band - rcm_offset[axis])

        surface = self.env.surface_point(self.env.time)
        delta_ref = float((p_d - surface) @ self.env.s0)
        ddelta = max(float(self.env.s0 @ (-de)), 0.0)
        k_eff = max(self.ke_hat, 1.0e-6) * (1.0 + self.b_hat * ddelta)
        force_budget = self.p.force_tightening * self.p.F_tissue_max
        delta_max = (force_budget / k_eff) ** (1.0 / self.p.n_hc)
        for i in range(N):
            block = slice(nx * i, nx * (i + 1))
            G_i = Gamma[block]
            efree = xfree[nx * i : nx * i + 3]
            emap = G_i[:3]
            # delta = delta_ref - s^T e <= delta_max
            row = -self.env.s0 @ emap
            rhs = delta_max - delta_ref + float(self.env.s0 @ efree)
            rows.append(row)
            lower.append(-np.inf)
            upper.append(rhs)

        sol = self._solve_osqp(
            H,
            h,
            np.asarray(rows),
            np.asarray(lower),
            np.asarray(upper),
        )
        if sol is None:
            return np.zeros(3), np.zeros(7)
        first = sol[:nu]
        predicted = xfree + Gamma @ sol
        max_rcm = 0.0
        max_force = 0.0
        for i in range(N):
            xi = predicted[nx * i : nx * (i + 1)]
            q_i = self.q0 + xi[6:13]
            rcm_i = e_rcm + J_rcm @ (q_i - st.q)
            max_rcm = max(max_rcm, float(np.linalg.norm(rcm_i)))
            delta_i = max(delta_ref - float(self.env.s0 @ xi[:3]), 0.0)
            max_force = max(max_force, k_eff * delta_i**self.p.n_hc)
        self.last_rcm_prediction = max_rcm
        self.last_force_prediction = max_force
        return first[:3], first[3:]

    def control(self, dyn, st, p_d, dp_d, ddp_d, R_d, p_rcm, resolve=True):
        control_start = time.perf_counter()
        solve_start = len(self.solve_times_ms)
        qp_fallback = False
        primary_qp_status = "not_run"
        fallback_qp_status = "not_run"
        p_tip, _, v_tip, J_tip = self.env.tip_state(dyn, st)
        M_inv = inv(dyn.M)
        Lam_inv = J_tip @ M_inv @ J_tip.T + 1.0e-6 * np.eye(3)
        Lam = inv(Lam_inv)

        f_meas, delta_now, ddelta_now = self.env.contact_probe(
            p_tip, v_tip, self.env.time
        )
        if resolve:
            self._rls_update(f_meas, delta_now, ddelta_now)

        p_d_control = (
            self._safe_reference(p_d, ddelta_now)
            if self.mode == "mpc_kalman"
            else p_d
        )
        reference_limited = np.linalg.norm(p_d_control - p_d) > 1.0e-12
        dp_control = np.zeros(3) if reference_limited else dp_d
        ddp_control = np.zeros(3) if reference_limited else ddp_d
        e = p_d_control - p_tip
        de = dp_control - v_tip
        e_rcm = self.env.rcm_error(st, p_rcm)
        J_rcm = self.env.rcm_jacobian(dyn, st, p_rcm)
        tau_ff = dyn.Cq_dot + J_tip.T @ (Lam @ ddp_control)

        if self.mode in ("impedance", "pid_force"):
            F = self.p.k_imp * e + self.p.d_imp * de
            if self.mode == "pid_force":
                force_error = max(f_meas - self.p.force_pid_target, 0.0)
                self._force_pid_integral = np.clip(
                    self._force_pid_integral + force_error * self.env.dt,
                    0.0,
                    self.p.force_pid_integral_max,
                )
                force_relief = (
                    self.p.force_pid_kp * force_error
                    + self.p.force_pid_ki * self._force_pid_integral
                )
                F -= force_relief * self.env.s0
            os_model = build_operational_space_model(dyn, st.ee_vel)
            tau_aux = os_model.N_bar.T @ (
                -self.p.k_rcm_soft * J_rcm.T @ e_rcm
                - self.p.k_posture * (st.q - self.q0)
                - self.p.d_null * st.dq
            )
            d_hat = np.zeros(3)
        elif not resolve:
            F = self._F_prev.copy()
            tau_aux = self._tau_aux_prev.copy()
            d_hat = self.x_aug[6:9]
        else:
            d_free = None
            if self.mode in ("mpc_kalman", "mpc_kalman_soft"):
                Aaug = self._kalman(e, de, Lam_inv)
                xi = self.x_aug.copy()
                d_free = np.zeros(6 * self.p.N)
                for i in range(self.p.N):
                    xi = Aaug @ xi
                    d_free[6 * i : 6 * (i + 1)] = xi[:6]
                d_hat = self.x_aug[6:9]
            else:
                d_hat = np.zeros(3)

            delay_fallback = (
                self.mode == "mpc_kalman"
                and self.p.actuator_delay > self.p.delay_fallback_threshold
            )
            if self.mode == "mpc_kalman" and not delay_fallback:
                F, tau_aux = self._full_qp(
                    e,
                    de,
                    st,
                    p_d_control,
                    p_rcm,
                    J_tip,
                    J_rcm,
                    M_inv,
                    tau_ff,
                    d_free,
                )
                primary_qp_status = self.last_qp_status
                if self.last_qp_status not in ("solved", "solved inaccurate"):
                    qp_fallback = True
                    # A zero predictive correction is not a safe fallback:
                    # it removes task/RCM feedback exactly when the constrained
                    # linearization is infeasible.  Reuse the stable tip-MPC
                    # branch and soft null-space RCM regulation for this step.
                    F = self._tip_mpc(e, de, Lam_inv, None)
                    fallback_qp_status = self.last_qp_status
                    os_model = build_operational_space_model(dyn, st.ee_vel)
                    tau_aux = os_model.N_bar.T @ (
                        -self.p.k_rcm_soft * J_rcm.T @ e_rcm
                        - self.p.k_posture * (st.q - self.q0)
                        - self.p.d_null * st.dq
                    )
            else:
                # The delay fallback intentionally omits the high-gain
                # disturbance free-response estimate.  The plain tip MPC was
                # the delay-tolerant branch in the measured sweep.
                F = self._tip_mpc(
                    e, de, Lam_inv, None if delay_fallback else d_free
                )
                primary_qp_status = self.last_qp_status
                os_model = build_operational_space_model(dyn, st.ee_vel)
                tau_aux = os_model.N_bar.T @ (
                    -self.p.k_rcm_soft * J_rcm.T @ e_rcm
                    - self.p.k_posture * (st.q - self.q0)
                    - self.p.d_null * st.dq
                )
            self._F_prev = F.copy()
            self._tau_aux_prev = tau_aux.copy()

        tau_raw = tau_ff + J_tip.T @ F + tau_aux
        if self._tau_cmd_prev is None:
            tau = tau_raw
        else:
            tau = tau_raw
            if self.p.torque_filter_tau > 0.0:
                alpha = np.exp(-self.env.dt / self.p.torque_filter_tau)
                tau = alpha * self._tau_cmd_prev + (1.0 - alpha) * tau
            if np.isfinite(self.p.torque_slew_rate):
                max_step = self.p.torque_slew_rate * self.env.dt
                tau = self._tau_cmd_prev + np.clip(
                    tau - self._tau_cmd_prev, -max_step, max_step
                )
        # Protect fallback and held-command updates as well.  For a solved full
        # QP this should be inactive except for solver tolerances/model freezing.
        tau = np.clip(tau, self.tau_min, self.tau_max)
        self._tau_cmd_prev = tau.copy()
        info = {
            "F": F,
            "tau_aux": tau_aux,
            "tau_ff": tau_ff,
            "tau_raw": tau_raw,
            "tau_saturated": bool(np.any(np.abs(tau - tau_raw) > 1.0e-8)),
            "d_hat": d_hat,
            "e_rcm": e_rcm,
            "p_tip": p_tip,
            "ke_hat": self.ke_hat,
            "b_hat": self.b_hat,
            "qp_status": primary_qp_status,
            "fallback_qp_status": fallback_qp_status,
            "qp_ms": float(sum(self.solve_times_ms[solve_start:])),
            "qp_solve_count": len(self.solve_times_ms) - solve_start,
            "qp_fallback": qp_fallback,
            "predicted_peak_force": self.last_force_prediction,
            "predicted_peak_rcm": self.last_rcm_prediction,
            "delay_fallback": (
                self.mode == "mpc_kalman"
                and self.p.actuator_delay > self.p.delay_fallback_threshold
            ),
        }
        info["control_ms"] = 1.0e3 * (time.perf_counter() - control_start)
        return tau, info
