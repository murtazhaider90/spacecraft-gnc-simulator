from __future__ import annotations

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


def run_case(name: str, axis: np.ndarray, angle_deg: float, initial_rate_dps: np.ndarray, disturbance_nm: np.ndarray, seed: int) -> dict:
    rng = np.random.default_rng(seed)
    q0 = quat_from_axis_angle(axis, np.deg2rad(angle_deg))
    state = State(np.zeros(3), np.zeros(3), q0, np.deg2rad(initial_rate_dps))
    vehicle = Vehicle(120.0, np.diag([42.0, 38.0, 30.0]))
    guidance = FixedAttitudeGuidance(np.array([1.0, 0.0, 0.0, 0.0]))
    controller = QuaternionPD(4.0, 18.0)
    gyro = Gyroscope(np.deg2rad([0.02, -0.02, 0.01]), np.deg2rad(0.01), rng)
    star = StarTracker(np.deg2rad(0.05), rng)
    dq0 = quat_from_axis_angle(np.array([1.0, -1.0, 0.5]), np.deg2rad(0.5))
    estimator = QuaternionComplementaryFilter(quat_multiply(q0, dq0), 0.25)
    wheels = ReactionWheelAssembly(np.full(3, 1.0), np.full(3, 8.0))
    result = run_closed_loop(
        state,
        vehicle,
        guidance,
        controller,
        gyro,
        star,
        estimator,
        wheels,
        SimConfig(dt=0.02, duration=40.0, star_tracker_period_s=0.2),
        disturbance_torque_b=lambda _t, d=np.asarray(disturbance_nm, dtype=float): d,
    )
    settling = result.settling_time()
    return {
        "name": name,
        "initial_angle_deg": angle_deg,
        "settling_time_s": settling,
        "final_error_deg": result.final_error_deg,
        "rms_estimation_error_deg": result.rms_estimation_error_deg,
        "peak_torque_nm": result.peak_torque_nm,
        "peak_wheel_momentum_nms": result.peak_wheel_momentum_nms,
        "pass": settling is not None and result.final_error_deg < 1.0,
    }


def main() -> None:
    cases = [
        ("x_35deg", [1, 0, 0], 35.0, [0.2, 0.0, 0.0], [0.003, 0.0, 0.0]),
        ("y_35deg", [0, 1, 0], 35.0, [0.0, -0.2, 0.0], [0.0, -0.003, 0.0]),
        ("z_35deg", [0, 0, 1], 35.0, [0.0, 0.0, 0.2], [0.0, 0.0, 0.003]),
        ("diagonal_35deg", [1, 1, 1], 35.0, [0.2, -0.2, 0.2], [0.003, -0.003, 0.003]),
        ("diagonal_45deg_extension", [1, -2, 1], 45.0, [0.3, -0.3, 0.3], [0.004, -0.004, 0.004]),
        ("z_60deg_extension", [0, 0, 1], 60.0, [0.0, 0.0, -0.4], [0.0, 0.0, -0.004]),
    ]
    rows = [run_case(name, np.array(axis, dtype=float), angle, np.array(rate), np.array(dist), 100 + i) for i, (name, axis, angle, rate, dist) in enumerate(cases)]
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
