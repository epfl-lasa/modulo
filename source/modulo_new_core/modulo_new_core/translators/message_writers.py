from typing import TypeVar, Union

import clproto
import geometry_msgs.msg as geometry
import numpy as np
import rclpy.time
import sensor_msgs.msg
import state_representation as sr
from modulo_new_core.encoded_state import EncodedState

MsgT = TypeVar('MsgT')
StateT = TypeVar('StateT')


def __write_xyz(msg: Union[geometry.Point, geometry.Vector3], vector: np.array):
    """
    Helper function to write a vector to a Point or Vector3 message.

    :param msg: The message to populate
    :param vector: The vector to read from
    """
    if vector.size != 3:
        raise RuntimeError("Provide a vector of size 3 to transform to a Point or Vector3 message.")
    msg.x = vector[0]
    msg.y = vector[1]
    msg.z = vector[2]


def __write_quaternion(msg: geometry.Quaternion, quat: np.array):
    """
    Helper function to write a vector of quaternion coefficients (w,x,y,z) to a Quaternion message.

    :param msg: The message to populate
    :param quat: The vector to read from
    """
    if quat.size != 4:
        raise RuntimeError("Provide a vector of size 4 to transform to a Quaternion message.")
    msg.w = quat[0]
    msg.x = quat[1]
    msg.y = quat[2]
    msg.z = quat[3]


def write_msg(msg: MsgT, state: StateT):
    """
    Convert a state_representation State to a ROS message.

    :param msg: The ROS message to populate
    :param state: The state to read from
    """
    if not isinstance(state, sr.State):
        raise RuntimeError("This state type is not supported.")
    if state.is_empty():
        raise RuntimeError(f"{state.get_name()} state is empty while attempting to write it to message.")
    if isinstance(state, sr.CartesianState):
        if isinstance(msg, geometry.Accel):
            __write_xyz(msg.linear, state.get_linear_acceleration())
            __write_xyz(msg.angular, state.get_angular_acceleration())
        elif isinstance(msg, geometry.Pose):
            __write_xyz(msg.position, state.get_position())
            __write_quaternion(msg.orientation, state.get_orientation_coefficients())
        elif isinstance(msg, geometry.Transform):
            __write_xyz(msg.translation, state.get_position())
            __write_quaternion(msg.rotation, state.get_orientation_coefficients())
        elif isinstance(msg, geometry.Twist):
            __write_xyz(msg.linear, state.get_linear_velocity())
            __write_xyz(msg.angular, state.get_angular_velocity())
        elif isinstance(msg, geometry.Wrench):
            __write_xyz(msg.force, state.get_force())
            __write_xyz(msg.torque, state.get_torque())
        else:
            raise RuntimeError("The provided combination of state type and message type is not supported")
    elif isinstance(msg, sensor_msgs.msg.JointState) and isinstance(state, sr.JointState):
        msg.name = state.get_names()
        msg.position = state.get_positions().tolist()
        msg.velocity = state.get_velocities().tolist()
        msg.effort = state.get_torques().tolist()
    else:
        raise RuntimeError("The provided combination of state type and message type is not supported")


def write_stamped_msg(msg: MsgT, state: StateT, time: rclpy.time.Time):
    """
    Convert a state_representation State to a stamped ROS message.

    :param msg: The ROS message to populate
    :param state: The state to read from
    :param time: The time of the message
    """
    if isinstance(msg, geometry.AccelStamped):
        write_msg(msg.accel, state)
    elif isinstance(msg, geometry.PoseStamped):
        write_msg(msg.pose, state)
    elif isinstance(msg, geometry.TransformStamped):
        write_msg(msg.transform, state)
        msg.child_frame_id = state.get_name()
    elif isinstance(msg, geometry.TwistStamped):
        write_msg(msg.twist, state)
    elif isinstance(msg, geometry.WrenchStamped):
        write_msg(msg.wrench, state)
    else:
        raise RuntimeError("The provided combination of state type and message type is not supported")
    msg.header.stamp = time
    msg.header.frame_id = state.get_reference_frame()


def write_clproto_msg(state: StateT, clproto_message_type: clproto.MessageType) -> EncodedState():
    """
    Convert a state_representation State to an EncodedState message.

    :param state: The state to read from
    :param clproto_message_type: The clproto message type to encode the state
    """
    if not isinstance(state, sr.State):
        raise RuntimeError("This state type is not supported.")
    msg = EncodedState()
    msg.data = clproto.encode(state, clproto_message_type)
    return msg
