"""Repeated randomized laparoscopy benchmark.

Eight fixed seeds vary tissue stiffness/damping, respiration amplitude/frequency,
force-sensor noise/bias, and lateral disturbance magnitude.  All controllers
receive the same realization for a given seed.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import t as student_t

from lap_benchmark import (
    DT_DECIM,
    PUSH_WIN,
    T_END,
    lateral_axis,
    metrics,
    references,
    s_ref,
)
from lap_controller import LapController, LapCtrlParams
from lap_env import LapEnv, TissueParams


MODES = {
    "Position impedance": "impedance",
    "PID + force governor": "pid_force",
    "Tip-only MPC": "mpc",
    "Joint/task MPC": "mpc_kalman",
}
N_SEEDS = 8


def scenario(seed: int) -> dict[str, float]:
    rng = np.random.default_rng(seed)
    return {
        "k_e": float(rng.uniform(300.0, 900.0)),
        "b_hc": float(rng.uniform(0.05, 0.20)),
        "resp_amp": float(rng.uniform(2.0e-3, 4.0e-3)),
        "resp_f": float(rng.uniform(0.15, 0.30)),
        "sensor_sigma": float(rng.uniform(0.0, 0.08)),
        "sensor_bias": float(rng.uniform(-0.03, 0.03)),
        "push_mag": float(rng.uniform(3.0, 7.0)),
    }


def run(mode: str, cfg: dict[str, float], seed: int):
    tissue = TissueParams(
        k_e=cfg["k_e"],
        b_hc=cfg["b_hc"],
        resp_amp=cfg["resp_amp"],
        resp_f=cfg["resp_f"],
    )
    env = LapEnv(tissue=tissue)
    env.sensor_sigma = cfg["sensor_sigma"]
    env.sensor_bias = cfg["sensor_bias"]
    env.rng = np.random.default_rng(seed)
    params = LapCtrlParams()
    ctrl = LapController(mode, env, params)
    dt = env.dt
    u_lat = lateral_axis(env.s0)
    R_d = env.R0
    log = {
        key: []
        for key in (
            "t",
            "tip_err",
            "tip_s",
            "ref_s",
            "F_tissue",
            "rcm",
            "wall_F",
            "d_hat",
            "qp_ms",
            "control_ms",
        )
    }
    qp_status_counts: dict[str, int] = {}
    qp_fallback_count = 0
    torque_saturation_steps = 0
    force_violation_steps = 0
    for k in range(int(T_END / dt)):
        t = k * dt
        dyn, st = env.get_dynamics_and_state()
        p_d, dp_d, ddp_d = references(env, t, dt)
        resolve = k % DT_DECIM == 0
        tau, info = ctrl.control(
            dyn, st, p_d, dp_d, ddp_d, R_d, env.p_rcm0, resolve=resolve
        )
        torque_saturation_steps += int(info["tau_saturated"])
        if resolve and mode not in ("impedance", "pid_force"):
            status = str(info["qp_status"])
            qp_status_counts[status] = qp_status_counts.get(status, 0) + 1
            qp_fallback_count += int(info["qp_fallback"])
        env.apply_torque(tau)
        p_tip, _, v_tip, _ = env.tip_state(dyn, st)
        F_tis, _ = env.tissue_force(p_tip, v_tip, t)
        force_violation_steps += int(np.linalg.norm(F_tis) > params.F_tissue_max)
        env.apply_point_force(st, F_tis, p_tip)
        F_tro, e_rcm = env.trocar_force(st, env.p_rcm0)
        env.apply_point_force(st, F_tro, env.p_rcm0)
        if PUSH_WIN[0] <= t < PUSH_WIN[1]:
            env.apply_point_force(st, cfg["push_mag"] * u_lat, p_tip)
        log["t"].append(t)
        log["tip_err"].append(float(np.linalg.norm(p_d - p_tip)) * 1e3)
        log["tip_s"].append(float((p_tip - env.p_tip0) @ env.s0) * 1e3)
        log["ref_s"].append(s_ref(t) * 1e3)
        log["F_tissue"].append(float(np.linalg.norm(F_tis)))
        log["rcm"].append(float(np.linalg.norm(e_rcm)) * 1e3)
        log["wall_F"].append(float(np.linalg.norm(env.tro.K_wall * e_rcm)))
        log["d_hat"].append(float(np.linalg.norm(info["d_hat"])))
        if resolve and mode not in ("impedance", "pid_force"):
            log["qp_ms"].append(float(info["qp_ms"]))
            log["control_ms"].append(float(info["control_ms"]))
        env.step()
    result = metrics({key: np.asarray(value) for key, value in log.items()})
    result.update(
        seed=seed,
        qp_status_counts=qp_status_counts,
        qp_fallback_count=qp_fallback_count,
        torque_saturation_steps=torque_saturation_steps,
        force_violation_steps=force_violation_steps,
    )
    return result


def summarize(values):
    array = np.asarray(values, dtype=float)
    mean = float(np.mean(array))
    std = float(np.std(array, ddof=1))
    half_width = float(student_t.ppf(0.975, len(array) - 1) * std / np.sqrt(len(array)))
    return {
        "mean": mean,
        "std": std,
        "ci95": [mean - half_width, mean + half_width],
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def main():
    checkpoint = Path(__file__).with_name("lap_monte_carlo_checkpoint.json")
    if checkpoint.exists():
        saved = json.loads(checkpoint.read_text())
        raw = saved["raw_per_seed"]
        scenarios = saved["scenarios"]
    else:
        raw = {name: [] for name in MODES}
        scenarios = []
    for seed in range(N_SEEDS):
        cfg = scenario(seed)
        if seed == len(scenarios):
            scenarios.append(cfg)
        for name, mode in MODES.items():
            if len(raw[name]) > seed:
                continue
            raw[name].append(run(mode, cfg, seed))
            checkpoint.write_text(
                json.dumps(
                    {"scenarios": scenarios, "raw_per_seed": raw},
                    indent=2,
                )
            )

    keys = ("tip_rmse_mm", "ss_err_mm", "peak_tissue_N", "max_rcm_mm")
    summary = {
        name: {key: summarize([row[key] for row in rows]) for key in keys}
        for name, rows in raw.items()
    }
    for name, rows in raw.items():
        summary[name]["tail"] = {
            "rcm_over_0p5_mm_episodes": sum(
                row["max_rcm_mm"] > 0.5 for row in rows
            ),
            "force_over_3N_episodes": sum(
                row["peak_tissue_N"] > 3.0 for row in rows
            ),
            "qp_fallback_count": sum(row["qp_fallback_count"] for row in rows),
            "torque_saturation_steps": sum(
                row["torque_saturation_steps"] for row in rows
            ),
        }
    print("\nRandomized benchmark, mean +/- sample standard deviation")
    for name, values in summary.items():
        print(
            f"{name:<24}"
            f" RMSE {values['tip_rmse_mm']['mean']:.2f}+/-{values['tip_rmse_mm']['std']:.2f}"
            f" RCM {values['max_rcm_mm']['mean']:.3f}+/-{values['max_rcm_mm']['std']:.3f}"
            f" peakF {values['peak_tissue_N']['mean']:.2f}+/-{values['peak_tissue_N']['std']:.2f}"
        )
        print(
            f"{'':24} max RCM {values['max_rcm_mm']['max']:.3f}, "
            f"RCM>0.5 mm episodes {values['tail']['rcm_over_0p5_mm_episodes']}, "
            f"force>3 N episodes {values['tail']['force_over_3N_episodes']}, "
            f"QP fallbacks {values['tail']['qp_fallback_count']}"
        )

    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    plot_keys = ("tip_rmse_mm", "max_rcm_mm", "peak_tissue_N")
    labels = ("Tip RMSE [mm]", "Maximum RCM deviation [mm]", "Peak tissue force [N]")
    names = list(MODES)
    x = np.arange(len(names))
    for axis, key, label in zip(axes, plot_keys, labels):
        means = [summary[name][key]["mean"] for name in names]
        stds = [summary[name][key]["std"] for name in names]
        axis.bar(x, means, yerr=stds, capsize=3)
        axis.set_xticks(x)
        axis.set_xticklabels(names, rotation=22, ha="right", fontsize=8)
        axis.set_ylabel(label)
        axis.grid(axis="y", alpha=0.3)
    fig.suptitle(f"Randomized simulation benchmark ({N_SEEDS} fixed seeds)")
    fig.tight_layout()
    out = Path(__file__).with_suffix(".png")
    fig.savefig(out, dpi=170)
    Path(__file__).with_suffix(".json").write_text(
        json.dumps(
            {
                "n_seeds": N_SEEDS,
                "scenarios": scenarios,
                "raw_per_seed": raw,
                "summary": summary,
            },
            indent=2,
        )
    )
    checkpoint.unlink()
    print(f"[plot] saved -> {out}")


if __name__ == "__main__":
    main()
