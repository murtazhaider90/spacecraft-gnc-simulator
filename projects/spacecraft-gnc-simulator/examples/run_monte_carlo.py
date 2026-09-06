from __future__ import annotations

import argparse
import os
import sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "src"))

from gnc.control import QuaternionPD
from gnc.dynamics import State, Vehicle
from gnc.math_utils import quat_from_axis_angle
from gnc.sensors import Gyroscope
from gnc.simulation import SimConfig, run_closed_loop


def one_run(rng: np.random.Generator) -> tuple[float, float | None]:
    angle = np.deg2rad(rng.uniform(10.0, 30.0))
    axis = rng.normal(size=3)
    inertia_scale = rng.normal(1.0, 0.04, 3)
    vehicle = Vehicle(120.0, np.diag(np.array([42.0, 38.0, 30.0]) * inertia_scale))
    state0 = State(np.zeros(3), np.zeros(3), quat_from_axis_angle(axis, angle), rng.normal(0.0, np.deg2rad(0.15), 3))
    gyro = Gyroscope(rng.normal(0.0, np.deg2rad(0.02), 3), np.deg2rad(0.01), rng)
    controller = QuaternionPD(0.8, 3.0, 1.0)
    result = run_closed_loop(state0, np.array([1.0, 0.0, 0.0, 0.0]), vehicle, controller, gyro, SimConfig())
    return result.final_error_deg, result.settling_time()


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--runs", type=int, default=200)
    p.add_argument("--seed", type=int, default=7)
    args = p.parse_args()
    rng = np.random.default_rng(args.seed)
    rows = [one_run(rng) for _ in range(args.runs)]
    final = np.array([r[0] for r in rows])
    settling = [r[1] for r in rows]
    achieved = np.array([x is not None for x in settling])
    print(f"runs: {args.runs}")
    print(f"final error median/p95: {np.median(final):.4f}/{np.percentile(final,95):.4f} deg")
    print(f"settled within simulation: {100*np.mean(achieved):.1f}%")
    if np.any(achieved):
        vals = np.array([x for x in settling if x is not None])
        print(f"settling time median/p95: {np.median(vals):.2f}/{np.percentile(vals,95):.2f} s")


if __name__ == "__main__":
    main()
