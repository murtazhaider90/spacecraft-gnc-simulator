"""Quaternion attitude-control laws."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import quat_error


@dataclass(frozen=True)
class QuaternionPD:
    """Quaternion-feedback PD controller.

    For small errors, ``2*q_error.vector`` approximates the rotation vector in
    radians, so ``kp`` has approximately N m/rad units and ``kd`` N m s/rad.
    """

    kp: float
    kd: float

    def __post_init__(self) -> None:
        if self.kp <= 0.0 or self.kd < 0.0:
            raise ValueError("kp must be positive and kd non-negative")

    def command(
        self,
        q_ref: np.ndarray,
        q_est: np.ndarray,
        omega_est: np.ndarray,
        omega_ref: np.ndarray | None = None,
    ) -> np.ndarray:
        q_e = quat_error(q_ref, q_est)
        omega_ref = np.zeros(3) if omega_ref is None else np.asarray(omega_ref, dtype=float)
        omega_est = np.asarray(omega_est, dtype=float)
        if omega_est.shape != (3,) or omega_ref.shape != (3,):
            raise ValueError("angular rates must have shape (3,)")
        rate_error = omega_ref - omega_est
        return 2.0 * self.kp * q_e[1:4] + self.kd * rate_error
