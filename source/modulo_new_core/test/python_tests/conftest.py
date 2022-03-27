import pytest
import rclpy.clock
from state_representation import CartesianState, JointState


@pytest.fixture
def cart_state():
    return CartesianState().Random("test", "ref")


@pytest.fixture
def joint_state():
    return JointState().Random("robot", 3)


@pytest.fixture
def clock():
    return rclpy.clock.Clock()
