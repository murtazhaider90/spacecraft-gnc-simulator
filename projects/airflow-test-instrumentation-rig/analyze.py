from __future__ import annotations

import argparse
import numpy as np
import pandas as pd


def summarize(df: pd.DataFrame) -> dict[str, float]:
    required = {"inlet_pa", "outlet_pa", "temperature_c", "flow_m3_s"}
    if not required.issubset(df.columns):
        raise ValueError(f"missing columns: {sorted(required - set(df.columns))}")
    dp = df["inlet_pa"] - df["outlet_pa"]
    return {
        "mean_pressure_drop_pa": float(dp.mean()),
        "pressure_drop_std_pa": float(dp.std(ddof=1)),
        "mean_flow_m3_s": float(df["flow_m3_s"].mean()),
        "mean_temperature_c": float(df["temperature_c"].mean()),
        "samples": int(len(df)),
    }


def cfd_error(measured_dp_pa: float, cfd_dp_pa: float) -> float:
    if measured_dp_pa == 0:
        raise ValueError("measured pressure drop cannot be zero")
    return 100.0 * (cfd_dp_pa - measured_dp_pa) / measured_dp_pa


def demo(seed: int = 7) -> None:
    rng = np.random.default_rng(seed)
    n = 300
    df = pd.DataFrame({
        "inlet_pa": 101450 + rng.normal(0, 4, n),
        "outlet_pa": 101320 + rng.normal(0, 4, n),
        "temperature_c": 26 + rng.normal(0, 0.15, n),
        "flow_m3_s": 0.42 + rng.normal(0, 0.004, n),
    })
    stats = summarize(df)
    for k, v in stats.items():
        print(f"{k}: {v}")
    print("Synthetic demo only; values are not experimental measurements.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()
    if args.demo:
        demo()
