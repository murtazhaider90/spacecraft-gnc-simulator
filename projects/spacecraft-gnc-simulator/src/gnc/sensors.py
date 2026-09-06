"""Simple spacecraft sensor models."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class Gyroscope:
    bias: np.ndarray
    noise_std: float
    rng: np.random.Generator

    def measure(self, omega_true: np.ndarray) -> np.ndarray:
        if self.noise_std < 0:
            raise ValueError("noise_std must be non-negative")
        return (
            np.asarray(omega_true, dtype=float)
            + np.asarray(self.bias, dtype=float)
            + self.rng.normal(0.0, self.noise_std, 3)
        )
