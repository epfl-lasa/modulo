import pytest
import rclpy


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
