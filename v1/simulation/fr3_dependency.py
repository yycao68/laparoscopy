from __future__ import annotations

import os
from pathlib import Path
import sys


def add_fr3_sim_to_path(*required_modules: str) -> Path:
    default_path = Path(__file__).resolve().parents[3] / "pHRI" / "simulation"
    sim_path = Path(os.environ.get("PHRI_SIM_DIR", default_path)).expanduser()
    missing = [
        module
        for module in required_modules
        if module not in sys.modules and not (sim_path / f"{module}.py").is_file()
    ]
    if missing:
        names = ", ".join(f"{module}.py" for module in missing)
        raise ModuleNotFoundError(
            f"Missing external FR3 simulator modules: {names}. "
            f"Expected them in '{sim_path}'. Set PHRI_SIM_DIR to the directory "
            "containing the pHRI simulation sources. These sources are not "
            "included in the laparoscopy repository."
        )
    sys.path.insert(0, str(sim_path))
    return sim_path