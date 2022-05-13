import clproto
import geometry_msgs.msg
import numpy as np
import pytest
import rclpy
import sensor_msgs.msg
import state_representation as sr

import modulo_new_core.translators.message_readers as modulo_readers
import modulo_new_core.translators.message_writers as modulo_writers


def read_xyz(msg):
    return [msg.x, msg.y, msg.z]


def read_quaternion(msg):
    return [msg.w, msg.x, msg.y, msg.z]


def assert_np_array_equal(a: np.array, b: np.array, places=3):
    try:
        np.testing.assert_almost_equal(a, b, decimal=places)
    except AssertionError as e:
        pytest.fail(f'{e}')


def test_accel(cart_state: sr.CartesianState, clock: rclpy.clock.Clock):
    msg = geometry_msgs.msg.Accel()
    modulo_writers.write_msg(msg, cart_state)
    assert_np_array_equal(read_xyz(msg.linear), cart_state.get_linear_acceleration())
    assert_np_array_equal(read_xyz(msg.angular), cart_state.get_angular_acceleration())

    new_state = sr.CartesianState("new")
    with pytest.raises(RuntimeError):
        modulo_writers.write_msg(new_state, msg)
    modulo_readers.read_msg(new_state, msg)
    assert_np_array_equal(cart_state.get_acceleration(), new_state.get_acceleration())

    msg_stamped = geometry_msgs.msg.AccelStamped()
    modulo_writers.write_msg_stamped(msg_stamped, cart_state, clock.now().to_msg())
    assert msg_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(msg.linear), cart_state.get_linear_acceleration())
    assert_np_array_equal(read_xyz(msg.angular), cart_state.get_angular_acceleration())
    new_state = sr.CartesianState("new")
    modulo_readers.read_msg_stamped(new_state, msg_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_acceleration(), new_state.get_acceleration())


def test_pose(cart_state: sr.CartesianState, clock: rclpy.clock.Clock):
    msg = geometry_msgs.msg.Pose()
    modulo_writers.write_msg(msg, cart_state)
    assert_np_array_equal(read_xyz(msg.position), cart_state.get_position())
    assert_np_array_equal(read_quaternion(msg.orientation), cart_state.get_orientation_coefficients())

    new_state = sr.CartesianState("new")
    with pytest.raises(RuntimeError):
        modulo_writers.write_msg(new_state, msg)
    modulo_readers.read_msg(new_state, msg)
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())

    msg_stamped = geometry_msgs.msg.PoseStamped()
    modulo_writers.write_msg_stamped(msg_stamped, cart_state, clock.now().to_msg())
    assert msg_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(msg.position), cart_state.get_position())
    assert_np_array_equal(read_quaternion(msg.orientation), cart_state.get_orientation_coefficients())
    new_state = sr.CartesianState("new")
    modulo_readers.read_msg_stamped(new_state, msg_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())


def test_transform(cart_state: sr.CartesianState, clock: rclpy.clock.Clock):
    msg = geometry_msgs.msg.Transform()
    modulo_writers.write_msg(msg, cart_state)
    assert_np_array_equal(read_xyz(msg.translation), cart_state.get_position())
    assert_np_array_equal(read_quaternion(msg.rotation), cart_state.get_orientation_coefficients())

    new_state = sr.CartesianState("new")
    with pytest.raises(RuntimeError):
        modulo_writers.write_msg(new_state, msg)
    modulo_readers.read_msg(new_state, msg)
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())

    msg_stamped = geometry_msgs.msg.TransformStamped()
    modulo_writers.write_msg_stamped(msg_stamped, cart_state, clock.now().to_msg())
    assert msg_stamped.header.frame_id == cart_state.get_reference_frame()
    assert msg_stamped.child_frame_id == cart_state.get_name()
    assert_np_array_equal(read_xyz(msg.translation), cart_state.get_position())
    assert_np_array_equal(read_quaternion(msg.rotation), cart_state.get_orientation_coefficients())
    new_state = sr.CartesianState("new")
    modulo_readers.read_msg_stamped(new_state, msg_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert cart_state.get_name() == new_state.get_name()
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())


def test_twist(cart_state: sr.CartesianState, clock: rclpy.clock.Clock):
    msg = geometry_msgs.msg.Twist()
    modulo_writers.write_msg(msg, cart_state)
    assert_np_array_equal(read_xyz(msg.linear), cart_state.get_linear_velocity())
    assert_np_array_equal(read_xyz(msg.angular), cart_state.get_angular_velocity())

    new_state = sr.CartesianState("new")
    with pytest.raises(RuntimeError):
        modulo_writers.write_msg(new_state, msg)
    modulo_readers.read_msg(new_state, msg)
    assert_np_array_equal(cart_state.get_twist(), new_state.get_twist())

    msg_stamped = geometry_msgs.msg.TwistStamped()
    modulo_writers.write_msg_stamped(msg_stamped, cart_state, clock.now().to_msg())
    assert msg_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(msg.linear), cart_state.get_linear_velocity())
    assert_np_array_equal(read_xyz(msg.angular), cart_state.get_angular_velocity())
    new_state = sr.CartesianState("new")
    modulo_readers.read_msg_stamped(new_state, msg_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_twist(), new_state.get_twist())


def test_wrench(cart_state: sr.CartesianState, clock: rclpy.clock.Clock):
    msg = geometry_msgs.msg.Wrench()
    modulo_writers.write_msg(msg, cart_state)
    assert_np_array_equal(read_xyz(msg.force), cart_state.get_force())
    assert_np_array_equal(read_xyz(msg.torque), cart_state.get_torque())

    new_state = sr.CartesianState("new")
    with pytest.raises(RuntimeError):
        modulo_writers.write_msg(new_state, msg)
    modulo_readers.read_msg(new_state, msg)
    assert_np_array_equal(cart_state.get_wrench(), new_state.get_wrench())

    msg_stamped = geometry_msgs.msg.WrenchStamped()
    modulo_writers.write_msg_stamped(msg_stamped, cart_state, clock.now().to_msg())
    assert msg_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(msg.force), cart_state.get_force())
    assert_np_array_equal(read_xyz(msg.torque), cart_state.get_torque())
    new_state = sr.CartesianState("new")
    modulo_readers.read_msg_stamped(new_state, msg_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_wrench(), new_state.get_wrench())


def test_joint_state(joint_state: sr.JointState):
    msg = sensor_msgs.msg.JointState()
    modulo_writers.write_msg(msg, joint_state)
    for i in range(joint_state.get_size()):
        assert msg.name[i] == joint_state.get_names()[i]
    assert_np_array_equal(msg.position, joint_state.get_positions())
    assert_np_array_equal(msg.velocity, joint_state.get_velocities())
    assert_np_array_equal(msg.effort, joint_state.get_torques())

    new_state = sr.JointState("test", ["1", "2", "3"])
    with pytest.raises(RuntimeError):
        modulo_writers.write_msg(new_state, msg)
    modulo_readers.read_msg(new_state, msg)
    for i in range(joint_state.get_size()):
        assert new_state.get_names()[i] == joint_state.get_names()[i]
    assert_np_array_equal(new_state.get_positions(), joint_state.get_positions())
    assert_np_array_equal(new_state.get_velocities(), joint_state.get_velocities())
    assert_np_array_equal(new_state.get_torques(), joint_state.get_torques())


def test_encoded_state(cart_state: sr.CartesianState):
    msg = modulo_writers.write_clproto_msg(cart_state, clproto.MessageType.CARTESIAN_STATE_MESSAGE)

    new_state = modulo_readers.read_clproto_msg(msg)
    assert_np_array_equal(new_state.data(), cart_state.data())
    assert new_state.get_name() == cart_state.get_name()
    assert new_state.get_reference_frame() == cart_state.get_reference_frame()
