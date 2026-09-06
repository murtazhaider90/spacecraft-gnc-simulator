import numpy as np

from gnc.estimation import QuaternionComplementaryFilter
from gnc.math_utils import attitude_error_deg, quat_from_axis_angle


def test_absolute_measurement_correction_reduces_error():
    q_true = np.array([1.0, 0.0, 0.0, 0.0])
    q_bad = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), np.deg2rad(10.0))
    filt = QuaternionComplementaryFilter(q_bad, correction_gain=0.5)
    before = attitude_error_deg(q_true, filt.quaternion)
    q_est = filt.update(np.zeros(3), 0.01, q_true)
    after = attitude_error_deg(q_true, q_est)
    assert after < before
