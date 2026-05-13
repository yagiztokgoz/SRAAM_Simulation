#!/usr/bin/env python3
"""Export SRAAM6 aerodynamic deck tables to a MATLAB .mat file.

The Simulink model should use the exported breakpoint vectors and table
arrays directly in 1-D and n-D Lookup Table blocks.
"""

from pathlib import Path
import sys

from scipy.io import savemat

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import config
from missile.database import SRAAM6_Database


def main():
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "simulink" / "data" / "aero_tables.mat"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    db = SRAAM6_Database()
    db.load_aero_deck(config.AERO_DECK)
    db.load_propulsion_deck(config.PROP_DECK)

    mat = {
        "mach_bp":  db.mach_grid,
        "alpha_bp": db.alpha_grid,
        "beta_bp":  db.beta_grid,
    }

    # Aero tables
    for name, interp in db.aero_db.items():
        if hasattr(interp, "values"):
            mat[name] = interp.values          # 3-D: shape (Mach, Alpha, Beta)
        else:
            mat[name] = interp.y               # 1-D: values at mach_bp

    # Propulsion tables — export breakpoints + values separately
    for name, interp in db.prop_db.items():
        mat[f"prop_{name}_t"] = interp.x      # time breakpoints
        mat[f"prop_{name}_v"] = interp.y      # values

    savemat(out_path, mat)
    print(f"Wrote {out_path}")
    print(f"  Aero tables:      {len(db.aero_db)}")
    print(f"  Propulsion tables: {len(db.prop_db)}")
    print(f"  Prop keys: {list(db.prop_db.keys())}")


if __name__ == "__main__":
    main()
