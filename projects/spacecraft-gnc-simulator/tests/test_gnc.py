import os
import sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "src"))

from gnc.control import QuaternionPD
from gnc.dynamics import State, Vehicle, rk4_step
from gnc.math_utils import normalize_quaternion, quat_from_axis_angle, attitude_error_deg


def test_quaternion_normalization():
    q = normalize_quaternion(np.array([2.0, 0.0, 0.0, 0.0]))
    assert np.allclose(q, [1.0, 0.0, 0.0, 0.0])


def test_zero_input_equilibrium():
    state = State(np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3))
    vehicle = Vehicle(10.0, np.diag([2.0, 3.0, 4.0]))
    nxt = rk4_step(state, vehicle, np.zeros(3), np.zeros(3), 0.1)
    assert np.allclose(nxt.vector(), state.vector(), atol=1e-12)


def test_controller_respects_torque_limit():
    c = QuaternionPD(kp=100.0, kd=0.0, torque_limit=0.5)
    q = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), np.deg2rad(90.0))
    tau = c.command(np.array([1.0, 0.0, 0.0, 0.0]), q, np.zeros(3))
    assert np.max(np.abs(tau)) <= 0.5 + 1e-12


def test_error_zero_for_same_attitude():
    q = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.3)
    assert attitude_error_deg(q, q) < 1e-9
