import clproto
import geometry_msgs.msg as geometry
import numpy as np
import rclpy.time
import sensor_msgs.msg
import state_representation as sr
from modulo_new_core.encoded_state import EncodedState


def write_point(vector: np.array) -> geometry.Point:
    msg = geometry.Point()
    msg.x = vector[0]
    msg.y = vector[1]
    msg.z = vector[2]
    return msg


def write_vector(vector: np.array) -> geometry.Vector3:
    msg = geometry.Vector3()
    msg.x = vector[0]
    msg.y = vector[1]
    msg.z = vector[2]
    return msg


def write_quaternion(quat: np.array) -> geometry.Quaternion:
    msg = geometry.Quaternion()
    msg.w = quat[0]
    msg.x = quat[1]
    msg.y = quat[2]
    msg.z = quat[3]
    return msg


def write_msg(msg, state):
    if isinstance(msg, geometry.Accel) and isinstance(state, sr.CartesianState):
        msg.linear = write_vector(state.get_linear_acceleration())
        msg.angular = write_vector(state.get_angular_acceleration())
    elif isinstance(msg, geometry.Pose) and isinstance(state, sr.CartesianState):
        msg.position = write_point(state.get_position())
        msg.orientation = write_quaternion(state.get_orientation())
    elif isinstance(msg, geometry.Transform) and isinstance(state, sr.CartesianState):
        msg.translation = write_vector(state.get_position())
        msg.rotation = write_quaternion(state.get_orientation())
    elif isinstance(msg, geometry.Twist) and isinstance(state, sr.CartesianState):
        msg.linear = write_vector(state.get_linear_velocity())
        msg.angular = write_vector(state.get_angular_velocity())
    elif isinstance(msg, geometry.Wrench) and isinstance(state, sr.CartesianState):
        msg.force = write_vector(state.get_force())
        msg.torque = write_vector(state.get_torque())
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
