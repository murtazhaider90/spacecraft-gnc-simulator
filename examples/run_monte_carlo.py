from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from gnc.actuators import ReactionWheelAssembly
from gnc.control import QuaternionPD
from gnc.dynamics import State, Vehicle
from gnc.estimation import QuaternionComplementaryFilter
from gnc.guidance import FixedAttitudeGuidance
from gnc.math_utils import quat_from_axis_angle, quat_multiply
from gnc.sensors import Gyroscope, StarTracker
from gnc.simulation import SimConfig, run_closed_loop


def one_run(rng: np.random.Generator) -> dict[str, float | bool | None]:
    angle = np.deg2rad(rng.uniform(10.0, 35.0))
    axis = rng.normal(size=3)
    inertia_scale = np.clip(rng.normal(1.0, 0.05, 3), 0.85, 1.15)
    initial_rate = rng.normal(0.0, np.deg2rad(0.20), 3)
    vehicle = Vehicle(120.0, np.diag(np.array([42.0, 38.0, 30.0]) * inertia_scale))
    state = State(np.zeros(3), np.zeros(3), quat_from_axis_angle(axis, angle), initial_rate)
    q_ref = np.array([1.0, 0.0, 0.0, 0.0])
    guidance = FixedAttitudeGuidance(q_ref)
    controller = QuaternionPD(4.0, 18.0)
    gyro = Gyroscope(rng.normal(0.0, np.deg2rad(0.025), 3), np.deg2rad(0.01), rng)
    star = StarTracker(np.deg2rad(0.05), rng)
    estimator_initial_error = quat_from_axis_angle(rng.normal(size=3), np.deg2rad(rng.uniform(0.0, 1.0)))
    estimator_initial_q = quat_multiply(state.quaternion, estimator_initial_error)
    estimator = QuaternionComplementaryFilter(estimator_initial_q, 0.25)
    wheels = ReactionWheelAssembly(np.full(3, 1.0), np.full(3, 8.0))
    config = SimConfig(dt=0.02, duration=40.0, star_tracker_period_s=0.2)
    disturbance = rng.normal(0.0, 0.002, 3)
    result = run_closed_loop(
        state,
        vehicle,
        guidance,
        controller,
        gyro,
        star,
        estimator,
        wheels,
        config,
        disturbance_torque_b=lambda _t, d=disturbance: d,
    )
    settling = result.settling_time()
    return {
        "initial_error_deg": float(np.degrees(angle)),
        "final_error_deg": result.final_error_deg,
        "settling_time_s": settling,
        "settled": settling is not None,
        "peak_torque_nm": result.peak_torque_nm,
        "peak_wheel_momentum_nms": result.peak_wheel_momentum_nms,
        "rms_estimation_error_deg": result.rms_estimation_error_deg,
    }


def summarize(rows: list[dict[str, float | bool | None]]) -> dict[str, float]:
    final = np.array([float(r["final_error_deg"]) for r in rows])
    settled = np.array([bool(r["settled"]) for r in rows])
    settling = np.array([float(r["settling_time_s"]) for r in rows if r["settling_time_s"] is not None])
    est = np.array([float(r["rms_estimation_error_deg"]) for r in rows])
    peak_torque = np.array([float(r["peak_torque_nm"]) for r in rows])
    peak_h = np.array([float(r["peak_wheel_momentum_nms"]) for r in rows])
    return {
        "runs": float(len(rows)),
        "success_rate_percent": float(100.0 * np.mean(settled)),
        "final_error_median_deg": float(np.median(final)),
        "final_error_p95_deg": float(np.percentile(final, 95)),
        "settling_time_median_s": float(np.median(settling)) if settling.size else float("nan"),
        "settling_time_p95_s": float(np.percentile(settling, 95)) if settling.size else float("nan"),
        "rms_estimation_error_p95_deg": float(np.percentile(est, 95)),
        "peak_torque_max_nm": float(np.max(peak_torque)),
        "peak_wheel_momentum_p95_nms": float(np.percentile(peak_h, 95)),
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--runs", type=int, default=500)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--csv", type=Path)
    p.add_argument("--json", type=Path)
    args = p.parse_args()
    if args.runs <= 0:
        raise SystemExit("--runs must be positive")
    rng = np.random.default_rng(args.seed)
    rows = [one_run(rng) for _ in range(args.runs)]
    summary = summarize(rows)
    print(json.dumps(summary, indent=2, allow_nan=False))

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
