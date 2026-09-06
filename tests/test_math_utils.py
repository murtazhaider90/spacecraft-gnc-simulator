import numpy as np
import pytest

from gnc.math_utils import (
    attitude_error_deg,
    normalize_quaternion,
    quat_from_axis_angle,
    quat_multiply,
    slerp,
)


def test_normalize_quaternion_identity():
    assert np.allclose(normalize_quaternion(np.array([2.0, 0.0, 0.0, 0.0])), [1.0, 0.0, 0.0, 0.0])


def test_zero_quaternion_rejected():
    with pytest.raises(ValueError):
        normalize_quaternion(np.zeros(4))


def test_quaternion_multiply_identity():
    q = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), 0.4)
    ident = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(quat_multiply(ident, q), q)
    assert np.allclose(quat_multiply(q, ident), q)


def test_slerp_endpoints():
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    q1 = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), np.deg2rad(60.0))
    assert attitude_error_deg(q0, slerp(q0, q1, 0.0)) < 1e-9
    assert attitude_error_deg(q1, slerp(q0, q1, 1.0)) < 1e-9
