import numpy as np

from gnc.dynamics import State, Vehicle, rk4_step


def test_zero_input_equilibrium():
    state = State(np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3))
    vehicle = Vehicle(10.0, np.diag([2.0, 3.0, 4.0]))
    nxt = rk4_step(state, vehicle, np.zeros(3), np.zeros(3), 0.1)
    assert np.allclose(nxt.vector(), state.vector(), atol=1e-12)


def test_constant_force_matches_analytic_translation():
    vehicle = Vehicle(2.0, np.diag([1.0, 1.5, 2.0]))
    state = State(np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3))
    force = np.array([4.0, 0.0, 0.0])
    dt = 0.01
    for _ in range(100):
        state = rk4_step(state, vehicle, force, np.zeros(3), dt)
    assert np.allclose(state.position, [1.0, 0.0, 0.0], atol=1e-8)
    assert np.allclose(state.velocity, [2.0, 0.0, 0.0], atol=1e-8)
    assert abs(np.linalg.norm(state.quaternion) - 1.0) < 1e-12


def test_asymmetric_inertia_requires_symmetric_matrix():
    try:
        Vehicle(10.0, np.array([[1.0, 0.2, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]]))
    except ValueError:
        return
    raise AssertionError("non-symmetric inertia should be rejected")
