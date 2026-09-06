from __future__ import annotations

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


def main() -> None:
    rng = np.random.default_rng(7)
    vehicle = Vehicle(mass=120.0, inertia=np.diag([42.0, 38.0, 30.0]))
    state0 = State(
        position=np.zeros(3),
        velocity=np.zeros(3),
        quaternion=quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), np.deg2rad(20.0)),
        omega=np.zeros(3),
    )
    controller = QuaternionPD(kp=0.8, kd=3.0, torque_limit=1.0)
    gyro = Gyroscope(bias=np.deg2rad([0.01, -0.02, 0.015]), noise_std=np.deg2rad(0.005), rng=rng)
    result = run_closed_loop(state0, np.array([1.0, 0.0, 0.0, 0.0]), vehicle, controller, gyro, SimConfig())

    print(f"final attitude error: {result.final_error_deg:.4f} deg")
    settling = result.settling_time()
    print("settling time (<1 deg for 2 s):", "not achieved" if settling is None else f"{settling:.2f} s")
    print(f"peak commanded torque: {np.max(np.abs(result.torque_b)):.3f} N m")


if __name__ == "__main__":
    main()
