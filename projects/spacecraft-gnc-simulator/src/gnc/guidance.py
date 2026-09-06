"""Minimal attitude-guidance profiles."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import normalize_quaternion


@dataclass(frozen=True)
class FixedAttitudeGuidance:
    quaternion_ref: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "quaternion_ref", normalize_quaternion(self.quaternion_ref))

    def reference(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        _ = time_s
        return self.quaternion_ref.copy(), np.zeros(3)
