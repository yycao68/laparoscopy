"""Multi-condition verification for the full-length v1 paper.

Sweeps tissue stiffness, force threshold, empirical force tightening, and the
linearized RCM band.
The output is intended to test whether the main conclusions survive changes in
the simulated operating condition rather than one nominal episode.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import lap_verify as verify


def main():
    out = {"tissue": {}, "force_threshold": {}, "force_tightening": {}, "rcm_band": {}}

    for ke in (200.0, 500.0, 1200.0):
        log = verify.simulate(
            target_pen=0.04,
            push=False,
            true_ke=ke,
            force_limit=3.0,
        )
        out["tissue"][str(int(ke))] = {
            "peak_force_N": float(log["F_tissue"].max()),
            "ke_hat": float(log["ke_hat"][-1]),
            "relative_error_pct": float(abs(log["ke_hat"][-1] - ke) / ke * 100.0),
        }

    for limit in (2.0, 3.0, 4.0):
        log = verify.simulate(
            target_pen=0.06,
            push=False,
            force_limit=limit,
        )
        out["force_threshold"][f"{limit:.0f}"] = {
            "peak_force_N": float(log["F_tissue"].max()),
        }

    for rho in (0.25, 0.33, 0.40):
        log = verify.simulate(
            target_pen=0.04,
            push=False,
            true_ke=1200.0,
            force_limit=3.0,
            force_tightening=rho,
        )
        out["force_tightening"][f"{rho:.2f}"] = {
            "peak_force_N": float(log["F_tissue"].max()),
        }

    for band_mm in (0.05, 0.10, 0.25, 0.50):
        log = verify.simulate(
            target_pen=0.015,
            push=True,
            rcm_band=band_mm * 1.0e-3,
        )
        out["rcm_band"][f"{band_mm:.2f}"] = {
            "max_rcm_mm": float(log["rcm"].max()),
        }

    print("\n== Tissue stiffness sweep ==")
    for key, value in out["tissue"].items():
        print(
            f"k={key:>4} N/m^1.5: peak={value['peak_force_N']:.2f} N, "
            f"khat={value['ke_hat']:.1f}, error={value['relative_error_pct']:.1f}%"
        )
    print("\n== Force-threshold sweep ==")
    for key, value in out["force_threshold"].items():
        print(f"limit={key} N: peak={value['peak_force_N']:.2f} N")
    print("\n== Force-tightening sensitivity (k=1200 N/m^1.5) ==")
    for key, value in out["force_tightening"].items():
        print(f"rho={key}: peak={value['peak_force_N']:.2f} N")
    print("\n== RCM-band sweep ==")
    for key, value in out["rcm_band"].items():
        print(f"linear band={key} mm: measured max={value['max_rcm_mm']:.3f} mm")

    fig, axes = plt.subplots(1, 4, figsize=(17.0, 3.8))
    tissue_keys = list(out["tissue"])
    axes[0].bar(
        tissue_keys,
        [out["tissue"][key]["relative_error_pct"] for key in tissue_keys],
        color="tab:blue",
    )
    axes[0].set_title("Tissue-stiffness adaptation")
    axes[0].set_xlabel(r"true $k_e$ [N/m$^{1.5}$]")
    axes[0].set_ylabel("final estimate error [%]")

    force_keys = list(out["force_threshold"])
    limits = np.asarray([float(key) for key in force_keys])
    peaks = np.asarray([out["force_threshold"][key]["peak_force_N"] for key in force_keys])
    axes[1].plot(limits, peaks, "o-", label="measured peak")
    axes[1].plot(limits, limits, "k:", label="threshold")
    axes[1].set_title("Force-threshold sweep")
    axes[1].set_xlabel("configured threshold [N]")
    axes[1].set_ylabel("measured peak [N]")
    axes[1].legend(fontsize=7)

    rho_keys = list(out["force_tightening"])
    axes[2].plot(
        [float(key) for key in rho_keys],
        [out["force_tightening"][key]["peak_force_N"] for key in rho_keys],
        "o-",
    )
    axes[2].axhline(3.0, color="k", ls=":")
    axes[2].set_title(r"Tightening sensitivity ($k_e=1200$)")
    axes[2].set_xlabel(r"$\rho_F$")
    axes[2].set_ylabel("measured peak [N]")

    band_keys = list(out["rcm_band"])
    axes[3].plot(
        [float(key) for key in band_keys],
        [out["rcm_band"][key]["max_rcm_mm"] for key in band_keys],
        "o-",
    )
    axes[3].axhline(0.5, color="k", ls=":")
    axes[3].set_title("RCM tightening")
    axes[3].set_xlabel("linearized QP band [mm]")
    axes[3].set_ylabel("measured maximum [mm]")

    for axis in axes:
        axis.grid(alpha=0.3)
    fig.tight_layout()
    root = Path(__file__).parent
    fig.savefig(root / "lap_sweep.png", dpi=170)
    (root / "lap_sweep.json").write_text(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
