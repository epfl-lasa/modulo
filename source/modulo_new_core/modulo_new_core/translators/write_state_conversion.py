import clproto
import geometry_msgs.msg as geometry
import numpy as np
import rclpy.time
import sensor_msgs.msg
import state_representation as sr
from modulo_new_core.encoded_state import EncodedState


def __write_xyz(msg, vector: np.array):
    msg.x = vector[0]
    msg.y = vector[1]
    msg.z = vector[2]
    return msg


def __write_quaternion(msg: geometry.Quaternion, quat: np.array) -> geometry.Quaternion:
    msg.w = quat[0]
    msg.x = quat[1]
    msg.y = quat[2]
    msg.z = quat[3]
    return msg


def write_msg(msg, state):
    if not isinstance(state, sr.State):
        raise RuntimeError("This state type is not supported")
    if state.is_empty():
        raise RuntimeError(f"{state.get_name()} state is empty while attempting to write it to message")
    if isinstance(msg, geometry.Point) and isinstance(state, sr.CartesianState):
        __write_xyz(msg, state)
    elif isinstance(msg, geometry.Vector3) and isinstance(state, sr.CartesianState):
        __write_xyz(msg, state)
    elif isinstance(msg, geometry.Quaternion) and isinstance(state, sr.CartesianState):
        __write_quaternion(msg, state)
    elif isinstance(msg, geometry.Accel) and isinstance(state, sr.CartesianState):
        __write_xyz(msg.linear, state.get_linear_acceleration())
        __write_xyz(msg.angular, state.get_angular_acceleration())
    elif isinstance(msg, geometry.Pose) and isinstance(state, sr.CartesianState):
        __write_xyz(msg.position, state.get_position())
        __write_quaternion(msg.orientation, state.get_orientation())
    elif isinstance(msg, geometry.Transform) and isinstance(state, sr.CartesianState):
        __write_xyz(msg.translation, state.get_position())
        __write_quaternion(msg.rotation, state.get_orientation())
    elif isinstance(msg, geometry.Twist) and isinstance(state, sr.CartesianState):
        __write_xyz(msg.linear, state.get_linear_velocity())
        __write_xyz(msg.angular, state.get_angular_velocity())
    elif isinstance(msg, geometry.Wrench) and isinstance(state, sr.CartesianState):
        __write_xyz(msg.force, state.get_force())
        __write_xyz(msg.torque, state.get_torque())
    elif isinstance(msg, sensor_msgs.msg.JointState) and isinstance(state, sr.JointState):
        msg.name = state.get_names()
        msg.position = state.get_positions().tolist()
        msg.velocity = state.get_velocities().tolist()
        msg.effort = state.get_torques().tolist()
    else:
        raise RuntimeError("This message type is not supported")
    return msg


def write_msg_stamped(msg, state, time: rclpy.time.Time):
    if isinstance(msg, geometry.AccelStamped):
        write_msg(msg.accel, state)
        msg.header.stamp = time
        msg.header.frame_id = state.get_reference_frame()
    elif isinstance(msg, geometry.PoseStamped):
        write_msg(msg.pose, state)
        msg.header.stamp = time
        msg.header.frame_id = state.get_reference_frame()
    elif isinstance(msg, geometry.TransformStamped):
        write_msg(msg.transform, state)
        msg.header.stamp = time
        msg.header.frame_id = state.get_reference_frame()
        msg.child_frame_id = state.get_name()
    elif isinstance(msg, geometry.TwistStamped):
        write_msg(msg.twist, state)
        msg.header.stamp = time
        msg.header.frame_id = state.get_reference_frame()
    elif isinstance(msg, geometry.WrenchStamped):
        write_msg(msg.wrench, state)
        msg.header.stamp = time
        msg.header.frame_id = state.get_reference_frame()
    else:
        raise RuntimeError("This message type is not supported")
    return state


def write_clproto_msg(state: sr.State, msg_type: clproto.MessageType) -> EncodedState():
    msg = EncodedState()
    if isinstance(state, sr.State) and isinstance(msg_type, clproto.MessageType):
        msg.data = clproto.encode(state, msg_type)
    else:
        raise RuntimeError("Your inputs do not have the correct types")
    return msg
