"""Attitude-estimation algorithms."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import integrate_quaternion, normalize_quaternion, slerp


@dataclass
class QuaternionComplementaryFilter:
    """Gyro propagation with periodic absolute-attitude correction.

    This intentionally simple estimator demonstrates navigation-loop separation
    without claiming EKF-level fidelity. A star-tracker correction gain of zero
    gives pure gyro dead reckoning; one snaps directly to the measurement.
    """

    quaternion: np.ndarray
    correction_gain: float = 0.2

    def __post_init__(self) -> None:
        self.quaternion = normalize_quaternion(self.quaternion)
        if not 0.0 <= self.correction_gain <= 1.0:
            raise ValueError("correction_gain must lie in [0, 1]")

    def update(
        self,
        gyro_measurement: np.ndarray,
        dt: float,
        star_tracker_measurement: np.ndarray | None = None,
    ) -> np.ndarray:
        predicted = integrate_quaternion(self.quaternion, gyro_measurement, dt)
        if star_tracker_measurement is not None:
            predicted = slerp(predicted, star_tracker_measurement, self.correction_gain)
        self.quaternion = predicted
        return self.quaternion.copy()
