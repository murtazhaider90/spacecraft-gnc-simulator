import numpy as np
import pytest

from gnc.actuators import ReactionWheelAssembly
from gnc.control import QuaternionPD
from gnc.dynamics import State, Vehicle, derivative, rk4_step
from gnc.estimation import QuaternionComplementaryFilter
from gnc.math_utils import integrate_quaternion, normalize_quaternion, quat_conjugate, quat_from_axis_angle, slerp
from gnc.sensors import Gyroscope, StarTracker
from gnc.simulation import SimConfig


def test_vehicle_validation_errors():
    with pytest.raises(ValueError):
        Vehicle(0.0, np.eye(3))
    with pytest.raises(ValueError):
        Vehicle(1.0, np.eye(2))
    with pytest.raises(ValueError):
        Vehicle(1.0, np.diag([1.0, 1.0, -1.0]))


def test_state_and_dynamics_validation_errors():
    with pytest.raises(ValueError):
        State(np.zeros(2), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3))
    state = State(np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3))
    vehicle = Vehicle(1.0, np.eye(3))
    with pytest.raises(ValueError):
        derivative(state, vehicle, np.zeros(2), np.zeros(3))
    with pytest.raises(ValueError):
        rk4_step(state, vehicle, np.zeros(3), np.zeros(3), 0.0)


def test_controller_validation_errors():
    with pytest.raises(ValueError):
        QuaternionPD(0.0, 1.0)
    c = QuaternionPD(1.0, 1.0)
    with pytest.raises(ValueError):
        c.command(np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(2))


def test_actuator_validation_errors():
    with pytest.raises(ValueError):
        ReactionWheelAssembly(np.zeros(3), np.ones(3))
    with pytest.raises(ValueError):
        ReactionWheelAssembly(np.ones(3), np.ones(3), np.array([2.0, 0.0, 0.0]))
    wheels = ReactionWheelAssembly(np.ones(3), np.ones(3))
    with pytest.raises(ValueError):
        wheels.apply(np.zeros(3), 0.0)
    with pytest.raises(ValueError):
        wheels.apply(np.zeros(2), 0.1)


def test_math_validation_errors():
    with pytest.raises(ValueError):
        normalize_quaternion(np.ones(3))
    with pytest.raises(ValueError):
        quat_conjugate(np.ones(3))
    with pytest.raises(ValueError):
        quat_from_axis_angle(np.ones(2), 0.1)
    with pytest.raises(ValueError):
        quat_from_axis_angle(np.zeros(3), 0.1)
    with pytest.raises(ValueError):
        integrate_quaternion(np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), 0.0)
    with pytest.raises(ValueError):
        integrate_quaternion(np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(2), 0.1)
    with pytest.raises(ValueError):
        slerp(np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]), 2.0)


def test_sensor_estimator_and_config_validation_errors():
    rng = np.random.default_rng(0)
    with pytest.raises(ValueError):
        Gyroscope(np.zeros(2), 0.0, rng)
    with pytest.raises(ValueError):
        Gyroscope(np.zeros(3), -1.0, rng)
    gyro = Gyroscope(np.zeros(3), 0.0, rng)
    with pytest.raises(ValueError):
        gyro.measure(np.zeros(2))
    with pytest.raises(ValueError):
        StarTracker(-1.0, rng)
    with pytest.raises(ValueError):
        QuaternionComplementaryFilter(np.array([1.0, 0.0, 0.0, 0.0]), 1.5)
    with pytest.raises(ValueError):
        SimConfig(dt=0.0)


def test_remaining_validation_and_edge_branches():
    from gnc.dynamics import State
    from gnc.math_utils import slerp
    from gnc.sensors import StarTracker
    from gnc.simulation import SimResult

    with pytest.raises(ValueError):
        State.from_vector(np.zeros(12))

    q = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(slerp(q, -q, 0.5), q)

    rng = np.random.default_rng(1)
    assert np.allclose(StarTracker(0.0, rng).measure(q), q)

    one = SimResult(
        time=np.array([0.0]),
        attitude_error_deg=np.array([2.0]),
        estimation_error_deg=np.array([0.0]),
        commanded_torque_b=np.zeros((1, 3)),
        applied_torque_b=np.zeros((1, 3)),
        omega_b=np.zeros((1, 3)),
        wheel_momentum=np.zeros((1, 3)),
        position_i=np.zeros((1, 3)),
        velocity_i=np.zeros((1, 3)),
    )
    assert one.settling_time() is None

    two = SimResult(
        time=np.array([0.0, 1.0, 2.0]),
        attitude_error_deg=np.array([2.0, 2.0, 2.0]),
        estimation_error_deg=np.zeros(3),
        commanded_torque_b=np.zeros((3, 3)),
        applied_torque_b=np.zeros((3, 3)),
        omega_b=np.zeros((3, 3)),
        wheel_momentum=np.zeros((3, 3)),
        position_i=np.zeros((3, 3)),
        velocity_i=np.zeros((3, 3)),
    )
    assert two.settling_time(threshold_deg=1.0, hold_s=1.0) is None


def test_closed_loop_rejects_bad_external_vector_shape():
    from gnc.actuators import ReactionWheelAssembly
    from gnc.control import QuaternionPD
    from gnc.dynamics import State, Vehicle
    from gnc.estimation import QuaternionComplementaryFilter
    from gnc.guidance import FixedAttitudeGuidance
    from gnc.sensors import Gyroscope, StarTracker
    from gnc.simulation import SimConfig, run_closed_loop

    rng = np.random.default_rng(2)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        run_closed_loop(
            State(np.zeros(3), np.zeros(3), q, np.zeros(3)),
            Vehicle(1.0, np.eye(3)),
            FixedAttitudeGuidance(q),
            QuaternionPD(1.0, 1.0),
            Gyroscope(np.zeros(3), 0.0, rng),
            StarTracker(0.0, rng),
            QuaternionComplementaryFilter(q, 0.2),
            ReactionWheelAssembly(np.ones(3), np.ones(3)),
            SimConfig(dt=0.01, duration=0.01, star_tracker_period_s=0.01),
            disturbance_torque_b=lambda _t: np.zeros(2),
        )
