"""
Laparoscopic environment: FR3 + rigid instrument through a compliant trocar,
contacting a Hunt--Crossley tissue surface.

This wraps the existing FR3MuJoCoEnv (reused from pHRI/simulation)
and adds the laparoscopy-specific *physics that lives in Python*:

  * a rigid instrument of length L attached along the flange z-axis;
  * the tool-tip kinematics (position p_tip, shaft direction s_hat, tip
    Jacobian J_tip);
  * the Remote-Center-of-Motion (RCM) error e_rcm and its analytic Jacobian
    J_rcm w.r.t. q;
  * a compliant trocar (lateral spring K_w * e_rcm + axial Coulomb/viscous
    friction) applied as an external wrench at the trocar point on the shaft;
  * a Hunt--Crossley tissue plane  F = k_e * delta^n * (1 + b * delta_dot)
    applied at the tool tip, with a slow sinusoidal "respiration" motion of
    the tissue surface.

No MJCF edits are required: the instrument and the contact/trocar forces are
modelled analytically and injected through env.apply_ee_wrench().
"""

from __future__ import annotations
import sys
from pathlib import Path
from dataclasses import dataclass, field
import numpy as np

# Reuse the FR3 MuJoCo bridge and helpers from the pHRI project.
sys.path.insert(0, str(Path(__file__).parent))
from fr3_dependency import add_fr3_sim_to_path  # noqa: E402

add_fr3_sim_to_path("fr3_mujoco", "so3_utils")
from fr3_mujoco import FR3MuJoCoEnv, Q_NEUTRAL          # noqa: E402
from so3_utils import skew                               # noqa: E402


@dataclass
class TrocarParams:
    L_instr:  float = 0.30        # instrument length, flange -> tip (m)
    d_trocar: float = 0.15        # trocar distance from flange along shaft (m)
    K_wall:   float = 200.0       # lateral abdominal-wall stiffness (N/m)
    mu_c:     float = 1.5         # axial Coulomb friction (N)
    mu_v:     float = 5.0         # axial viscous friction (N.s/m)
    r_clear:  float = 0.0         # radial port clearance / deadzone (m):
                                  #   no wall force until |e_rcm| exceeds it
    mu_s:     float = 0.0         # static breakaway friction (N); >mu_c enables
                                  #   Stribeck/stick-slip behaviour near zero speed
    stick_slip: bool = False      # enable stick-slip (Stribeck) trocar friction


@dataclass
class TissueParams:
    k_e:      float = 500.0       # Hunt--Crossley stiffness (N/m^n)
    n_hc:     float = 1.5         # Hunt--Crossley exponent
    b_hc:     float = 0.1         # Hunt--Crossley damping (s/m)
    resp_amp: float = 3.0e-3      # respiration amplitude (m)
    resp_f:   float = 0.20        # respiration frequency (Hz)


class LapEnv:
    """Laparoscopic wrapper around FR3MuJoCoEnv."""

    def __init__(self, trocar: TrocarParams | None = None,
                 tissue: TissueParams | None = None,
                 timestep: float = 0.001):
        self.tro = trocar or TrocarParams()
        self.tis = tissue or TissueParams()
        # force/penetration sensing model (0 = ideal); set by stress tests
        self.sensor_sigma = 0.0       # Gaussian force-sensor noise (N, 1-sigma)
        self.sensor_bias = 0.0        # constant force-sensor bias (N)
        self.rng = np.random.default_rng(0)
        self.env = FR3MuJoCoEnv(timestep=timestep)
        self.env.reset(Q_NEUTRAL)

        # Establish instrument geometry from the initial flange pose.
        dyn, st = self.env.get_dynamics_and_state()
        self.s0 = self._shaft_dir(st.ee_rot)             # initial shaft dir
        self.p_tip0 = st.ee_pos + self.tro.L_instr * self.s0
        # Trocar nominally on the initial shaft axis.
        self.p_rcm0 = st.ee_pos + self.tro.d_trocar * self.s0
        # Tissue surface placed just below the initial tip along the shaft.
        self.tissue_surf0 = self.p_tip0 + 0.02 * self.s0   # 2 cm beyond tip at rest
        self.R0 = st.ee_rot.copy()
        self._prev_delta = 0.0

    # ------------------------------------------------------------------
    # Instrument kinematics
    # ------------------------------------------------------------------
    @staticmethod
    def _shaft_dir(R: np.ndarray) -> np.ndarray:
        """Shaft unit vector = flange z-axis in world frame (flange -> tip)."""
        return R[:, 2].copy()

    def tip_state(self, dyn, st):
        """Return (p_tip, s_hat, v_tip, J_tip) for the current state."""
        s_hat = self._shaft_dir(st.ee_rot)
        p_tip = st.ee_pos + self.tro.L_instr * s_hat
        J_v = dyn.J[:3, :]
        J_w = dyn.J[3:, :]
        # v_tip = v_flange + omega x (L s) = v_flange - skew(L s) omega
        J_tip = J_v - skew(self.tro.L_instr * s_hat) @ J_w
        v_tip = J_tip @ st.dq
        return p_tip, s_hat, v_tip, J_tip

    # ------------------------------------------------------------------
    # RCM error and its Jacobian
    # ------------------------------------------------------------------
    def rcm_error(self, st, p_rcm):
        """Lateral deviation e_rcm = (I - s s^T)(p_rcm - p_flange)  in R^3."""
        s = self._shaft_dir(st.ee_rot)
        a = p_rcm - st.ee_pos
        P = np.eye(3) - np.outer(s, s)
        return P @ a

    def rcm_jacobian(self, dyn, st, p_rcm):
        """Analytic J_rcm = d e_rcm / dq  in R^{3x7}.

        e_rcm = P(s) a,  a = p_rcm - p_flange,  P = I - s s^T.
        d e_rcm = -P J_v dq + [alpha skew(s) + s (a^T skew(s))] J_w dq,
        with alpha = s^T a  (axial coordinate of the trocar along the shaft).
        """
        s = self._shaft_dir(st.ee_rot)
        a = p_rcm - st.ee_pos
        alpha = float(s @ a)
        P = np.eye(3) - np.outer(s, s)
        J_v = dyn.J[:3, :]
        J_w = dyn.J[3:, :]
        Ss = skew(s)
        term = alpha * Ss + np.outer(s, a @ Ss)          # 3x3
        return -P @ J_v + term @ J_w

    # ------------------------------------------------------------------
    # Force models
    # ------------------------------------------------------------------
    def respiration_offset(self, t):
        """Tissue-surface displacement along the shaft from respiration."""
        return self.tis.resp_amp * np.sin(2 * np.pi * self.tis.resp_f * t)

    def surface_point(self, t):
        """Current tissue-surface point along the shaft (read-only)."""
        return self.tissue_surf0 + self.respiration_offset(t) * self.s0

    def contact_probe(self, p_tip, v_tip, t):
        """Read-only contact sensing: (force magnitude, penetration delta, ddelta).

        Returns what a (calibrated, model-based) force/penetration sensor would
        report — used by the controller's RLS estimator and force governor.
        Does NOT apply any force.
        """
        surf = self.surface_point(t)
        delta = float((p_tip - surf) @ self.s0)
        ddelta = float(v_tip @ self.s0)
        if delta <= 0.0:
            return 0.0, 0.0, ddelta
        f_mag = max(self.tis.k_e * delta**self.tis.n_hc * (1.0 + self.tis.b_hc*ddelta),
                    0.0)
        if self.sensor_sigma > 0.0 or self.sensor_bias != 0.0:   # sensing model
            f_mag = max(f_mag + self.sensor_bias
                        + self.sensor_sigma*self.rng.standard_normal(), 0.0)
        return f_mag, delta, ddelta

    def tissue_force(self, p_tip, v_tip, t):
        """Hunt--Crossley reaction force at the tip (world frame, 3-vector).

        Returns (F_tip, delta) where delta is the penetration depth (>=0).
        Surface moves along +s0 (toward the tip) with respiration.
        """
        surf = self.tissue_surf0 + self.respiration_offset(t) * self.s0
        # Penetration measured along the inward shaft direction s0.
        delta = float((p_tip - surf) @ self.s0)
        if delta <= 0.0:
            self._prev_delta = max(delta, 0.0)
            return np.zeros(3), 0.0
        ddelta = float(v_tip @ self.s0)
        f_mag = self.tis.k_e * delta ** self.tis.n_hc * (1.0 + self.tis.b_hc * ddelta)
        f_mag = max(f_mag, 0.0)                            # no adhesion
        self._prev_delta = delta
        return -f_mag * self.s0, delta                    # opposes insertion

    def trocar_force(self, st, p_rcm):
        """Compliant-trocar force at the trocar point (world frame, 3-vector).

        Lateral abdominal-wall spring  -K_w e_rcm  plus axial Coulomb+viscous
        friction opposing the shaft sliding through the port.

        Returns (F_world, e_rcm).
        """
        e_rcm = self.rcm_error(st, p_rcm)
        # Radial port clearance (deadzone): the wall reacts only on the part of
        # the deviation beyond r_clear (models trocar play / soft seal).
        mag = float(np.linalg.norm(e_rcm))
        if self.tro.r_clear > 0.0 and mag > 1e-12:
            eff = max(mag - self.tro.r_clear, 0.0)
            F_lat = -self.tro.K_wall * eff * (e_rcm / mag)
        else:
            F_lat = -self.tro.K_wall * e_rcm              # restores shaft to axis
        s = self._shaft_dir(st.ee_rot)
        v_ax = float(st.ee_vel[:3] @ s)                   # axial sliding speed
        if self.tro.stick_slip and self.tro.mu_s > self.tro.mu_c:
            # Stribeck: elevated static friction near zero speed -> stick-slip
            v_brk = 2.0e-3
            mu_eff = self.tro.mu_c + (self.tro.mu_s - self.tro.mu_c) \
                * np.exp(-(v_ax / v_brk) ** 2)
            f_fric = -(mu_eff * np.tanh(v_ax / 5e-4) + self.tro.mu_v * v_ax)
        else:
            f_fric = -(self.tro.mu_c * np.tanh(v_ax / 1e-3) + self.tro.mu_v * v_ax)
        return F_lat + f_fric * s, e_rcm

    # ------------------------------------------------------------------
    # Apply a world-frame force at an arbitrary point as an EE-site wrench
    # ------------------------------------------------------------------
    def apply_point_force(self, st, F_world, p_apply):
        """Apply F_world acting at world point p_apply, as a wrench at the EE site."""
        r = p_apply - st.ee_pos                           # site -> application pt
        wrench = np.concatenate([F_world, np.cross(r, F_world)])
        self.env.apply_ee_wrench(wrench)

    # passthroughs -----------------------------------------------------
    @property
    def dt(self):
        return self.env.dt

    @property
    def time(self):
        return self.env.time

    def get_dynamics_and_state(self):
        return self.env.get_dynamics_and_state()

    def apply_torque(self, tau):
        self.env.apply_torque(tau)

    def step(self):
        self.env.step()

    def reset(self):
        self.env.reset(Q_NEUTRAL)
        self._prev_delta = 0.0
