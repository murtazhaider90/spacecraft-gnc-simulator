"""Rigid-body 6-DOF spacecraft dynamics and fixed-step RK4 integration."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import normalize_quaternion, quat_multiply


@dataclass(frozen=True)
class Vehicle:
    mass: float
    inertia: np.ndarray

    def __post_init__(self) -> None:
        if self.mass <= 0.0:
            raise ValueError("mass must be positive")
        inertia = np.asarray(self.inertia, dtype=float)
        if inertia.shape != (3, 3):
            raise ValueError("inertia must be 3x3")
        if not np.allclose(inertia, inertia.T, atol=1e-12):
            raise ValueError("inertia must be symmetric")
        if np.any(np.linalg.eigvalsh(inertia) <= 0.0):
            raise ValueError("inertia must be positive definite")
        object.__setattr__(self, "inertia", inertia)


@dataclass
class State:
    position: np.ndarray
    velocity: np.ndarray
    quaternion: np.ndarray
    omega: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        self.quaternion = normalize_quaternion(self.quaternion)
        self.omega = np.asarray(self.omega, dtype=float)
        if self.position.shape != (3,) or self.velocity.shape != (3,) or self.omega.shape != (3,):
            raise ValueError("position, velocity and omega must have shape (3,)")

    def vector(self) -> np.ndarray:
        return np.r_[self.position, self.velocity, self.quaternion, self.omega]

    @classmethod
    def from_vector(cls, x: np.ndarray) -> "State":
        x = np.asarray(x, dtype=float)
        if x.shape != (13,):
            raise ValueError("state vector must have length 13")
        return cls(x[0:3], x[3:6], x[6:10], x[10:13])


def derivative(
    state: State,
    vehicle: Vehicle,
    force_i: np.ndarray,
    torque_b: np.ndarray,
) -> np.ndarray:
    force_i = np.asarray(force_i, dtype=float)
    torque_b = np.asarray(torque_b, dtype=float)
    if force_i.shape != (3,) or torque_b.shape != (3,):
        raise ValueError("force and torque must have shape (3,)")

    pos_dot = state.velocity
    vel_dot = force_i / vehicle.mass
    quat_dot = 0.5 * quat_multiply(state.quaternion, np.r_[0.0, state.omega])
    iw = vehicle.inertia @ state.omega
    omega_dot = np.linalg.solve(vehicle.inertia, torque_b - np.cross(state.omega, iw))
    return np.r_[pos_dot, vel_dot, quat_dot, omega_dot]


def rk4_step(
    state: State,
    vehicle: Vehicle,
    force_i: np.ndarray,
    torque_b: np.ndarray,
    dt: float,
) -> State:
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    x0 = state.vector()

    def f(x: np.ndarray) -> np.ndarray:
        return derivative(State.from_vector(x), vehicle, force_i, torque_b)

    k1 = f(x0)
    k2 = f(x0 + 0.5 * dt * k1)
    k3 = f(x0 + 0.5 * dt * k2)
    k4 = f(x0 + dt * k3)
    return State.from_vector(x0 + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0)
