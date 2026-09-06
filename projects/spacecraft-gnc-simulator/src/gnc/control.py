"""Attitude control laws."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import quat_error


@dataclass(frozen=True)
class QuaternionPD:
    kp: float
    kd: float
    torque_limit: float

    def command(self, q_ref: np.ndarray, q: np.ndarray, omega: np.ndarray) -> np.ndarray:
        if self.kp < 0 or self.kd < 0 or self.torque_limit <= 0:
            raise ValueError("invalid controller gains/limits")
        q_e = quat_error(q_ref, q)
        torque = 2.0 * self.kp * q_e[1:4] - self.kd * np.asarray(omega, dtype=float)
        return np.clip(torque, -self.torque_limit, self.torque_limit)
