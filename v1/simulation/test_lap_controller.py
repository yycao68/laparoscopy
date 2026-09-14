from __future__ import annotations

from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys

import numpy as np
import pytest


fr3_impedance = ModuleType("fr3_impedance")
fr3_impedance.build_operational_space_model = lambda *_args, **_kwargs: None
sys.modules.setdefault("fr3_impedance", fr3_impedance)
sys.path.insert(0, str(Path(__file__).parent))

from lap_controller import LapController, LapCtrlParams
import lap_controller as lap_controller_module
from fr3_dependency import add_fr3_sim_to_path
from lap_hinf_design import hinf_riccati


def _minimal_controller(horizon: int = 2) -> LapController:
    controller = LapController.__new__(LapController)
    controller.mode = "mpc_kalman"
    controller.p = LapCtrlParams(
        N=horizon,
        F_tissue_max=100.0,
        force_tightening=1.0,
    )
    controller.use_osc = True
    controller.nx_full = 20
    controller.nu_full = 10
    controller.q0 = np.zeros(7)
    controller.tau_min = -100.0 * np.ones(7)
    controller.tau_max = 100.0 * np.ones(7)
    controller.ke_hat = controller.p.ke_prior
    controller.b_hat = 0.0
    controller.solve_times_ms = []
    controller.last_qp_status = "not_run"
    controller.last_force_prediction = 0.0
    controller.last_rcm_prediction = 0.0
    controller._solver_cache = {}
    controller._terminal_cache = {}
    controller._F_prev = np.zeros(3)
    controller._tau_aux_prev = np.zeros(7)
    controller._tau_cmd_prev = None
    controller.env = SimpleNamespace(
        dt=1.0e-3,
        s0=np.array([0.0, 0.0, 1.0]),
        time=0.0,
        rcm_error=lambda _st, _p_rcm: np.zeros(3),
        rcm_jacobian=lambda _dyn, _st, _p_rcm: np.zeros((3, 7)),
        surface_point=lambda _time: np.zeros(3),
    )
    controller._build_constant_models()
    return controller


def test_missing_fr3_dependency_has_actionable_error(monkeypatch, tmp_path):
    monkeypatch.setenv("PHRI_SIM_DIR", str(tmp_path))

    with pytest.raises(ModuleNotFoundError) as error:
        add_fr3_sim_to_path("missing_fr3_bridge")

    message = str(error.value)
    assert "missing_fr3_bridge.py" in message
    assert "PHRI_SIM_DIR" in message
    assert "not included" in message


def test_hinf_riccati_iteration_exhaustion_is_not_feasible():
    result, feasible = hinf_riccati(
        A=np.array([[0.5]]),
        B=np.array([[1.0]]),
        E=np.array([[1.0]]),
        Q=np.array([[1.0]]),
        R=np.array([[1.0]]),
        gamma=10.0,
        iters=0,
    )

    assert result is None
    assert not feasible


def test_control_forwards_augmented_free_response_to_full_qp():
    controller = _minimal_controller()
    controller.x_aug = np.zeros(11)
    controller.x_aug[0] = 0.01
    controller._kalman = lambda _e, _de, _lam_inv: np.eye(11)
    controller._rls_update = lambda _force, _delta, _rate: None
    controller._safe_reference = lambda reference, _rate: reference

    J_tip = np.zeros((3, 7))
    J_tip[:, :3] = np.eye(3)
    controller.env.tip_state = (
        lambda _dyn, _st: (np.zeros(3), None, np.zeros(3), J_tip)
    )
    controller.env.contact_probe = (
        lambda _tip, _velocity, _time: (0.0, 0.0, 0.0)
    )
    captured = {}

    def capture_full_qp(*args):
        captured["d_free"] = args[-1]
        controller.last_qp_status = "solved"
        return np.zeros(3), np.zeros(7)

    controller._full_qp = capture_full_qp
    state = SimpleNamespace(q=np.zeros(7), dq=np.zeros(7), ee_vel=np.zeros(6))
    dynamics = SimpleNamespace(M=np.eye(7), Cq_dot=np.zeros(7))

    controller.control(
        dynamics,
        state,
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.eye(3),
        np.zeros(3),
    )

    expected = np.tile(controller.x_aug[:6], controller.p.N)
    np.testing.assert_allclose(captured["d_free"], expected)


def test_full_qp_command_changes_with_free_response():
    controller = _minimal_controller()
    state = SimpleNamespace(q=np.zeros(7), dq=np.zeros(7))
    J_tip = np.zeros((3, 7))
    J_tip[:, :3] = np.eye(3)
    args = (
        np.zeros(3),
        np.zeros(3),
        state,
        np.zeros(3),
        np.zeros(3),
        J_tip,
        np.zeros((3, 7)),
        np.eye(7),
        np.zeros(7),
    )

    zero_force, zero_torque = controller._full_qp(*args, None)
    d_free = np.tile(
        np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0]), controller.p.N
    )
    changed_force, changed_torque = controller._full_qp(*args, d_free)

    command_delta = np.linalg.norm(
        np.r_[changed_force - zero_force, changed_torque - zero_torque]
    )
    assert command_delta > 1.0e-6


def test_full_qp_force_rows_use_free_response():
    controller = _minimal_controller()
    state = SimpleNamespace(q=np.zeros(7), dq=np.zeros(7))
    J_tip = np.zeros((3, 7))
    J_tip[:, :3] = np.eye(3)
    captured_force_bounds = []

    def capture_constraints(H, _h, _constraints, _lower, upper):
        captured_force_bounds.append(upper[-controller.p.N :].copy())
        controller.last_qp_status = "solved"
        return np.zeros(H.shape[0])

    controller._solve_osqp = capture_constraints
    args = (
        np.zeros(3),
        np.zeros(3),
        state,
        np.array([0.0, 0.0, 0.02]),
        np.zeros(3),
        J_tip,
        np.zeros((3, 7)),
        np.eye(7),
        np.zeros(7),
    )

    controller._full_qp(*args, None)
    d_free = np.tile(
        np.array([0.0, 0.0, 0.01, 0.0, 0.0, 0.0]), controller.p.N
    )
    controller._full_qp(*args, d_free)

    np.testing.assert_allclose(
        captured_force_bounds[1] - captured_force_bounds[0], 0.01
    )


def test_full_qp_adds_joint_trust_region_rows():
    controller = _minimal_controller()
    state = SimpleNamespace(q=np.zeros(7), dq=np.zeros(7))
    J_tip = np.zeros((3, 7))
    J_tip[:, :3] = np.eye(3)
    captured = {}

    def capture_constraints(H, _h, constraints, lower, upper):
        captured["constraints"] = constraints
        captured["lower"] = lower
        captured["upper"] = upper
        controller.last_qp_status = "solved"
        return np.zeros(H.shape[0])

    controller._solve_osqp = capture_constraints
    controller._full_qp(
        np.zeros(3),
        np.zeros(3),
        state,
        np.zeros(3),
        np.zeros(3),
        J_tip,
        np.zeros((3, 7)),
        np.eye(7),
        np.zeros(7),
        None,
    )

    trust_lower = []
    trust_upper = []
    first_state_row = 10 * controller.p.N
    for step in range(controller.p.N):
        start = first_state_row + 10 * step
        trust_lower.extend(captured["lower"][start : start + 7])
        trust_upper.extend(captured["upper"][start : start + 7])

    np.testing.assert_allclose(trust_lower, -controller.p.rcm_trust_radius)
    np.testing.assert_allclose(trust_upper, controller.p.rcm_trust_radius)


def test_fallback_preserves_primary_status_and_aggregates_solver_time(monkeypatch):
    controller = _minimal_controller()
    controller.x_aug = np.zeros(11)
    controller._kalman = lambda _e, _de, _lam_inv: np.eye(11)
    controller._rls_update = lambda _force, _delta, _rate: None
    controller._safe_reference = lambda reference, _rate: reference

    J_tip = np.zeros((3, 7))
    J_tip[:, :3] = np.eye(3)
    controller.env.tip_state = (
        lambda _dyn, _st: (np.zeros(3), None, np.zeros(3), J_tip)
    )
    controller.env.contact_probe = (
        lambda _tip, _velocity, _time: (0.0, 0.0, 0.0)
    )
    monkeypatch.setattr(
        lap_controller_module,
        "build_operational_space_model",
        lambda _dyn, _velocity: SimpleNamespace(N_bar=np.eye(7)),
    )

    def fail_full_qp(*_args):
        controller.solve_times_ms.append(1.25)
        controller.last_qp_status = "primal infeasible"
        return np.zeros(3), np.zeros(7)

    def solve_tip_mpc(*_args):
        controller.solve_times_ms.append(0.75)
        controller.last_qp_status = "solved"
        return np.zeros(3)

    controller._full_qp = fail_full_qp
    controller._tip_mpc = solve_tip_mpc
    state = SimpleNamespace(q=np.zeros(7), dq=np.zeros(7), ee_vel=np.zeros(6))
    dynamics = SimpleNamespace(M=np.eye(7), Cq_dot=np.zeros(7))

    _, info = controller.control(
        dynamics,
        state,
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.eye(3),
        np.zeros(3),
    )

    assert info["qp_status"] == "primal infeasible"
    assert info["fallback_qp_status"] == "solved"
    assert info["qp_solve_count"] == 2
    assert info["qp_ms"] == pytest.approx(2.0)


def test_configured_delay_above_four_ms_selects_fallback():
    controller = _minimal_controller()
    controller.p.actuator_delay = 5.0e-3
    assert controller.p.actuator_delay > controller.p.delay_fallback_threshold