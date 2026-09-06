"""Closed-loop spacecraft attitude simulation and verification metrics."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .control import QuaternionPD
from .dynamics import State, Vehicle, rk4_step
from .math_utils import attitude_error_deg
from .sensors import Gyroscope


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.02
    duration: float = 30.0
    settle_threshold_deg: float = 1.0
    settle_hold_s: float = 2.0


@dataclass
class SimResult:
    time: np.ndarray
    attitude_error_deg: np.ndarray
    torque_b: np.ndarray
    omega_b: np.ndarray

    @property
    def final_error_deg(self) -> float:
        return float(self.attitude_error_deg[-1])

    def settling_time(self, threshold_deg: float = 1.0, hold_s: float = 2.0) -> float | None:
        if len(self.time) < 2:
            return None
        dt = float(self.time[1] - self.time[0])
        n_hold = max(1, int(np.ceil(hold_s / dt)))
        ok = self.attitude_error_deg <= threshold_deg
        for i in range(0, len(ok) - n_hold + 1):
            if np.all(ok[i:i+n_hold]):
                return float(self.time[i])
        return None


def run_closed_loop(
    state0: State,
    q_ref: np.ndarray,
    vehicle: Vehicle,
    controller: QuaternionPD,
    gyro: Gyroscope,
    config: SimConfig = SimConfig(),
) -> SimResult:
    if config.dt <= 0 or config.duration <= 0:
        raise ValueError("simulation timing must be positive")

    n = int(np.floor(config.duration / config.dt)) + 1
    t = np.arange(n) * config.dt
    err = np.zeros(n)
    torque_hist = np.zeros((n, 3))
    omega_hist = np.zeros((n, 3))

    state = state0
    zero_force = np.zeros(3)

    for i in range(n):
        err[i] = attitude_error_deg(q_ref, state.quaternion)
        omega_meas = gyro.measure(state.omega)
        torque = controller.command(q_ref, state.quaternion, omega_meas)
        torque_hist[i] = torque
        omega_hist[i] = state.omega
        if i < n - 1:
            state = rk4_step(state, vehicle, zero_force, torque, config.dt)

    return SimResult(t, err, torque_hist, omega_hist)
