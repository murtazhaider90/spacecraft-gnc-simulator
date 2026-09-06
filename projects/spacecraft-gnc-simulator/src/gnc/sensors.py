"""Simple spacecraft attitude-sensor models."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import normalize_quaternion, quat_from_axis_angle, quat_multiply


@dataclass
class Gyroscope:
    bias: np.ndarray
    noise_std: float
    rng: np.random.Generator

    def __post_init__(self) -> None:
        self.bias = np.asarray(self.bias, dtype=float)
        if self.bias.shape != (3,):
            raise ValueError("bias must have shape (3,)")
        if self.noise_std < 0.0:
            raise ValueError("noise_std must be non-negative")

    def measure(self, omega_true: np.ndarray) -> np.ndarray:
        omega_true = np.asarray(omega_true, dtype=float)
        if omega_true.shape != (3,):
            raise ValueError("omega_true must have shape (3,)")
        return omega_true + self.bias + self.rng.normal(0.0, self.noise_std, 3)


@dataclass
class StarTracker:
    """Absolute attitude measurement with isotropic small-angle white noise."""

    noise_std_rad: float
    rng: np.random.Generator

    def __post_init__(self) -> None:
        if self.noise_std_rad < 0.0:
            raise ValueError("noise_std_rad must be non-negative")

    def measure(self, q_true: np.ndarray) -> np.ndarray:
        if self.noise_std_rad == 0.0:
            return normalize_quaternion(q_true)
        axis = self.rng.normal(size=3)
        angle = float(self.rng.normal(0.0, self.noise_std_rad))
        dq = quat_from_axis_angle(axis, angle)
        return normalize_quaternion(quat_multiply(q_true, dq))
