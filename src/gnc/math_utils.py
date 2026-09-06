"""Quaternion and rotation utilities.

Convention
----------
Quaternions are scalar-first ``[w, x, y, z]`` and represent body-to-inertial
rotation. Functions normalize quaternion inputs where appropriate to limit
numerical drift.
"""
from __future__ import annotations

import numpy as np


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.shape != (4,):
        raise ValueError("quaternion must have shape (4,)")
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise ValueError("zero quaternion cannot be normalized")
    return q / norm


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.shape != (4,):
        raise ValueError("quaternion must have shape (4,)")
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(a, dtype=float)
    bw, bx, by, bz = np.asarray(b, dtype=float)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def quat_error(q_ref: np.ndarray, q_body: np.ndarray) -> np.ndarray:
    """Shortest-rotation quaternion taking ``q_body`` toward ``q_ref``."""
    q_e = normalize_quaternion(
        quat_multiply(normalize_quaternion(q_ref), quat_conjugate(normalize_quaternion(q_body)))
    )
    return -q_e if q_e[0] < 0.0 else q_e


def quat_from_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    if axis.shape != (3,):
        raise ValueError("axis must have shape (3,)")
    n = float(np.linalg.norm(axis))
    if n <= 0.0:
        raise ValueError("axis must be non-zero")
    axis = axis / n
    h = 0.5 * float(angle_rad)
    return normalize_quaternion(np.r_[np.cos(h), axis * np.sin(h)])


def attitude_error_deg(q_ref: np.ndarray, q_body: np.ndarray) -> float:
    q_e = quat_error(q_ref, q_body)
    angle = 2.0 * np.arccos(np.clip(q_e[0], -1.0, 1.0))
    return float(np.degrees(angle))


def integrate_quaternion(q: np.ndarray, omega_b: np.ndarray, dt: float) -> np.ndarray:
    """First-order quaternion propagation for estimator prediction.

    The plant uses RK4; this intentionally lightweight predictor represents the
    lower-fidelity onboard navigation propagation step.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    q = normalize_quaternion(q)
    omega_b = np.asarray(omega_b, dtype=float)
    if omega_b.shape != (3,):
        raise ValueError("omega_b must have shape (3,)")
    q_dot = 0.5 * quat_multiply(q, np.r_[0.0, omega_b])
    return normalize_quaternion(q + dt * q_dot)


def slerp(q0: np.ndarray, q1: np.ndarray, fraction: float) -> np.ndarray:
    """Spherical linear interpolation on the shortest quaternion arc."""
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction must lie in [0, 1]")
    q0 = normalize_quaternion(q0)
    q1 = normalize_quaternion(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion((1.0 - fraction) * q0 + fraction * q1)
    theta = np.arccos(dot)
    s = np.sin(theta)
    return normalize_quaternion(
        np.sin((1.0 - fraction) * theta) / s * q0
        + np.sin(fraction * theta) / s * q1
    )
