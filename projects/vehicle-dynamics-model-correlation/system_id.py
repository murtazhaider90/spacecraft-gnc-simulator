from __future__ import annotations

import argparse
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class FirstOrderFit:
    gain: float
    tau: float
    rmse: float


def simulate_first_order(t: np.ndarray, u: np.ndarray, gain: float, tau: float) -> np.ndarray:
    if tau <= 0:
        raise ValueError("tau must be positive")
    y = np.zeros_like(t, dtype=float)
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        y[i] = y[i - 1] + dt * ((gain * u[i - 1] - y[i - 1]) / tau)
    return y


def fit_grid(t: np.ndarray, steer_rad: np.ndarray, yaw_rate: np.ndarray) -> FirstOrderFit:
    gains = np.linspace(0.2, 8.0, 120)
    taus = np.linspace(0.05, 2.0, 120)
    best = None
    for g in gains:
        for tau in taus:
            pred = simulate_first_order(t, steer_rad, g, tau)
            rmse = float(np.sqrt(np.mean((pred - yaw_rate) ** 2)))
            if best is None or rmse < best.rmse:
                best = FirstOrderFit(g, tau, rmse)
    assert best is not None
    return best


def demo(seed: int = 7) -> None:
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, 8.0, 401)
    steer = np.where(t >= 1.0, np.deg2rad(4.0), 0.0)
    truth = simulate_first_order(t, steer, gain=2.4, tau=0.55)
    measured = truth + rng.normal(0.0, 0.002, len(t))
    fit = fit_grid(t, steer, measured)
    print(f"gain={fit.gain:.3f}, tau={fit.tau:.3f} s, rmse={fit.rmse:.6f} rad/s")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()
    if args.demo:
        demo()
    else:
        p.error("provide --demo; real-data CSV loader can be added without changing the estimator")


if __name__ == "__main__":
    main()
