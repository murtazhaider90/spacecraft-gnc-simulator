"""Closed-loop 6-DOF spacecraft GNC simulation and verification metrics."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import numpy as np

from .actuators import ReactionWheelAssembly
from .control import QuaternionPD
from .dynamics import State, Vehicle, rk4_step
from .estimation import QuaternionComplementaryFilter
from .guidance import FixedAttitudeGuidance
from .math_utils import attitude_error_deg
from .sensors import Gyroscope, StarTracker

VectorFn = Callable[[float], np.ndarray]


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.02
    duration: float = 40.0
    star_tracker_period_s: float = 0.2
    settle_threshold_deg: float = 1.0
    settle_hold_s: float = 2.0

    def __post_init__(self) -> None:
        if self.dt <= 0.0 or self.duration <= 0.0 or self.star_tracker_period_s <= 0.0:
            raise ValueError("simulation timing values must be positive")


@dataclass
class SimResult:
    time: np.ndarray
    attitude_error_deg: np.ndarray
    estimation_error_deg: np.ndarray
    commanded_torque_b: np.ndarray
    applied_torque_b: np.ndarray
    omega_b: np.ndarray
    wheel_momentum: np.ndarray
    position_i: np.ndarray
    velocity_i: np.ndarray

    @property
    def final_error_deg(self) -> float:
        return float(self.attitude_error_deg[-1])

    @property
    def peak_torque_nm(self) -> float:
        return float(np.max(np.abs(self.applied_torque_b)))

    @property
    def peak_wheel_momentum_nms(self) -> float:
        return float(np.max(np.abs(self.wheel_momentum)))

    @property
    def rms_estimation_error_deg(self) -> float:
        return float(np.sqrt(np.mean(np.square(self.estimation_error_deg))))

    def settling_time(self, threshold_deg: float = 1.0, hold_s: float = 2.0) -> float | None:
        if len(self.time) < 2:
            return None
        dt = float(self.time[1] - self.time[0])
        n_hold = max(1, int(np.ceil(hold_s / dt)))
        ok = self.attitude_error_deg <= threshold_deg
        for i in range(0, len(ok) - n_hold + 1):
            if np.all(ok[i : i + n_hold]):
                return float(self.time[i])
        return None


def run_closed_loop(
    state0: State,
    vehicle: Vehicle,
    guidance: FixedAttitudeGuidance,
    controller: QuaternionPD,
    gyro: Gyroscope,
    star_tracker: StarTracker,
    estimator: QuaternionComplementaryFilter,
    actuator: ReactionWheelAssembly,
    config: SimConfig = SimConfig(),
    disturbance_torque_b: VectorFn | None = None,
    external_force_i: VectorFn | None = None,
) -> SimResult:
    n = int(np.floor(config.duration / config.dt)) + 1
    t = np.arange(n, dtype=float) * config.dt
    att_err = np.zeros(n)
    est_err = np.zeros(n)
    command_hist = np.zeros((n, 3))
    applied_hist = np.zeros((n, 3))
    omega_hist = np.zeros((n, 3))
    momentum_hist = np.zeros((n, 3))
    position_hist = np.zeros((n, 3))
    velocity_hist = np.zeros((n, 3))

    state = State(*[x.copy() for x in (state0.position, state0.velocity, state0.quaternion, state0.omega)])
    star_steps = max(1, int(round(config.star_tracker_period_s / config.dt)))

    for i, ti in enumerate(t):
        q_ref, omega_ref = guidance.reference(float(ti))
        gyro_meas = gyro.measure(state.omega)
        star_meas = star_tracker.measure(state.quaternion) if i % star_steps == 0 else None
        q_est = estimator.update(gyro_meas, config.dt, star_meas)

        command = controller.command(q_ref, q_est, gyro_meas, omega_ref)
        applied = actuator.apply(command, config.dt)
        disturbance = np.zeros(3) if disturbance_torque_b is None else np.asarray(disturbance_torque_b(float(ti)), dtype=float)
        force = np.zeros(3) if external_force_i is None else np.asarray(external_force_i(float(ti)), dtype=float)
        if disturbance.shape != (3,) or force.shape != (3,):
            raise ValueError("external force/disturbance functions must return shape (3,)")

        att_err[i] = attitude_error_deg(q_ref, state.quaternion)
        est_err[i] = attitude_error_deg(state.quaternion, q_est)
        command_hist[i] = command
        applied_hist[i] = applied
        omega_hist[i] = state.omega
        momentum_hist[i] = actuator.wheel_momentum
        position_hist[i] = state.position
        velocity_hist[i] = state.velocity

        if i < n - 1:
            state = rk4_step(state, vehicle, force, applied + disturbance, config.dt)

    return SimResult(
        time=t,
        attitude_error_deg=att_err,
        estimation_error_deg=est_err,
        commanded_torque_b=command_hist,
        applied_torque_b=applied_hist,
        omega_b=omega_hist,
        wheel_momentum=momentum_hist,
        position_i=position_hist,
        velocity_i=velocity_hist,
    )
