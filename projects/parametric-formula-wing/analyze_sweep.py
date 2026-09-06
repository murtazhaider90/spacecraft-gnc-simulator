from __future__ import annotations

import argparse
import numpy as np
import pandas as pd


def score(df: pd.DataFrame) -> pd.DataFrame:
    required = {"aoa_deg", "ground_mm", "flap_deg", "cl", "cd"}
    if not required.issubset(df.columns):
        raise ValueError(f"missing columns: {sorted(required - set(df.columns))}")
    out = df.copy()
    out["efficiency"] = out["cl"] / out["cd"].replace(0.0, np.nan)
    # Penalize designs that sit far from the median operating geometry.
    robust_penalty = (
        np.abs(out["aoa_deg"] - out["aoa_deg"].median()) / 10.0
        + np.abs(out["ground_mm"] - out["ground_mm"].median()) / 100.0
    )
    out["robust_score"] = out["cl"] + 0.02 * out["efficiency"] - 0.05 * robust_penalty
    return out.sort_values("robust_score", ascending=False)


def demo() -> None:
    rows = []
    for aoa in [2, 6, 10, 14]:
        for ground in [40, 70, 100]:
            for flap in [0, 8, 16]:
                cl = 0.4 + 0.07 * aoa + 0.015 * flap - 0.0008 * (ground - 70) ** 2 / 70
                cd = 0.05 + 0.0025 * aoa**2 + 0.0006 * flap**2
                rows.append((aoa, ground, flap, cl, cd))
    df = pd.DataFrame(rows, columns=["aoa_deg", "ground_mm", "flap_deg", "cl", "cd"])
    print(score(df).head(8).to_string(index=False))
    print("Demo coefficients are synthetic and are not CFD/test results.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()
    if args.demo:
        demo()
