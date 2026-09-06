"""Three-axis reaction-wheel actuator model."""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class ReactionWheelAssembly:
    """Ideal orthogonal reaction wheels with torque and stored-momentum limits.

    ``wheel_momentum`` is wheel angular momentum about each body axis. Applying
    positive body torque decreases wheel momentum on that axis by conservation
    of angular momentum.
    """

    torque_limit: np.ndarray
    momentum_limit: np.ndarray
    wheel_momentum: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        self.torque_limit = np.broadcast_to(np.asarray(self.torque_limit, dtype=float), (3,)).copy()
        self.momentum_limit = np.broadcast_to(np.asarray(self.momentum_limit, dtype=float), (3,)).copy()
        self.wheel_momentum = np.broadcast_to(np.asarray(self.wheel_momentum, dtype=float), (3,)).copy()
        if np.any(self.torque_limit <= 0.0) or np.any(self.momentum_limit <= 0.0):
            raise ValueError("actuator limits must be positive")
        if np.any(np.abs(self.wheel_momentum) > self.momentum_limit):
            raise ValueError("initial wheel momentum exceeds limit")

    def apply(self, requested_body_torque: np.ndarray, dt: float) -> np.ndarray:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        requested = np.asarray(requested_body_torque, dtype=float)
        if requested.shape != (3,):
            raise ValueError("requested torque must have shape (3,)")

        lower_from_momentum = (self.wheel_momentum - self.momentum_limit) / dt
        upper_from_momentum = (self.wheel_momentum + self.momentum_limit) / dt
        lower = np.maximum(-self.torque_limit, lower_from_momentum)
        upper = np.minimum(self.torque_limit, upper_from_momentum)
        body_torque = np.clip(requested, lower, upper)
        self.wheel_momentum = self.wheel_momentum - body_torque * dt
        self.wheel_momentum = np.clip(self.wheel_momentum, -self.momentum_limit, self.momentum_limit)
        return body_torque
