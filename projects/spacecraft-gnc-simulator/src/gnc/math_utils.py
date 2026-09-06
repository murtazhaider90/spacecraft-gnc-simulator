"""Quaternion helpers for the spacecraft GNC simulator.

Convention: scalar-first q = [w, x, y, z].
"""
from __future__ import annotations

import numpy as np


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise ValueError("zero quaternion cannot be normalized")
    return q / norm


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(a, dtype=float)
    bw, bx, by, bz = np.asarray(b, dtype=float)
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_error(q_ref: np.ndarray, q_body: np.ndarray) -> np.ndarray:
    """Return shortest-rotation error taking q_body toward q_ref."""
    q_e = normalize_quaternion(quat_multiply(q_ref, quat_conjugate(q_body)))
    return -q_e if q_e[0] < 0.0 else q_e


def quat_from_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n == 0.0:
        raise ValueError("axis must be non-zero")
    axis = axis / n
    h = 0.5 * angle_rad
    return normalize_quaternion(np.r_[np.cos(h), axis * np.sin(h)])


def attitude_error_deg(q_ref: np.ndarray, q_body: np.ndarray) -> float:
    q_e = quat_error(q_ref, q_body)
    angle = 2.0 * np.arccos(np.clip(q_e[0], -1.0, 1.0))
    return float(np.degrees(angle))
