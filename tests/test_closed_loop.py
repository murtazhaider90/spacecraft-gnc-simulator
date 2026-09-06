import numpy as np

from gnc.actuators import ReactionWheelAssembly
from gnc.control import QuaternionPD
from gnc.dynamics import State, Vehicle
from gnc.estimation import QuaternionComplementaryFilter
from gnc.guidance import FixedAttitudeGuidance
from gnc.math_utils import quat_from_axis_angle
from gnc.sensors import Gyroscope, StarTracker
from gnc.simulation import SimConfig, run_closed_loop


def make_nominal(seed: int = 7):
    rng = np.random.default_rng(seed)
    q_ref = np.array([1.0, 0.0, 0.0, 0.0])
    state = State(
        np.zeros(3),
        np.zeros(3),
        quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), np.deg2rad(20.0)),
        np.zeros(3),
    )
    vehicle = Vehicle(120.0, np.diag([42.0, 38.0, 30.0]))
    guidance = FixedAttitudeGuidance(q_ref)
    controller = QuaternionPD(kp=4.0, kd=18.0)
    gyro = Gyroscope(np.deg2rad([0.01, -0.02, 0.015]), np.deg2rad(0.005), rng)
    star = StarTracker(np.deg2rad(0.03), rng)
    estimator = QuaternionComplementaryFilter(state.quaternion.copy(), correction_gain=0.25)
    wheels = ReactionWheelAssembly(np.full(3, 1.0), np.full(3, 8.0))
    config = SimConfig(dt=0.02, duration=40.0, star_tracker_period_s=0.2)
    return state, vehicle, guidance, controller, gyro, star, estimator, wheels, config


def test_nominal_requirement_is_met():
    result = run_closed_loop(*make_nominal())
    assert result.final_error_deg < 0.2
    settling = result.settling_time(1.0, 2.0)
    assert settling is not None and settling < 30.0
    assert result.peak_torque_nm <= 1.0 + 1e-12
    assert result.peak_wheel_momentum_nms <= 8.0 + 1e-12
    assert result.rms_estimation_error_deg < 0.2
