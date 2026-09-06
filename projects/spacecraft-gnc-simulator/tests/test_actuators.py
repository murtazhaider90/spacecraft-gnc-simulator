import numpy as np

from gnc.actuators import ReactionWheelAssembly


def test_reaction_wheel_torque_limit():
    wheels = ReactionWheelAssembly(np.array([0.5, 0.5, 0.5]), np.array([5.0, 5.0, 5.0]))
    applied = wheels.apply(np.array([2.0, -2.0, 0.1]), 0.1)
    assert np.allclose(applied, [0.5, -0.5, 0.1])


def test_reaction_wheel_momentum_limit():
    wheels = ReactionWheelAssembly(
        np.ones(3),
        np.ones(3),
        wheel_momentum=np.array([0.99, 0.0, 0.0]),
    )
    # Negative body torque would increase positive wheel momentum. The model
    # should reduce the command to hit, but not exceed, the momentum limit.
    applied = wheels.apply(np.array([-1.0, 0.0, 0.0]), 0.1)
    assert np.isclose(applied[0], -0.1, atol=1e-12)
    assert np.isclose(wheels.wheel_momentum[0], 1.0, atol=1e-12)
