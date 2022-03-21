from typing import List

import clproto
import geometry_msgs.msg as geometry
import sensor_msgs.msg
import std_msgs.msg


def read_point(msg: geometry.Point) -> List[float]:
    return [msg.x, msg.y, msg.z]


def read_vector(msg: geometry.Vector3) -> List[float]:
    return [msg.x, msg.y, msg.z]


def read_quaternion(msg: geometry.Quaternion) -> List[float]:
    return [msg.w, msg.x, msg.y, msg.z]


def read_msg(state, msg):
    if isinstance(msg, geometry.Accel):
        state.set_linear_acceleration(read_vector(msg.linear))
        state.set_angular_acceleration(read_vector(msg.angular))
    elif isinstance(msg, geometry.Pose):
        state.set_position(read_point(msg.position))
        state.set_orientation(read_quaternion(msg.orientation))
    elif isinstance(msg, geometry.Transform):
        state.set_position(read_vector(msg.translation))
        state.set_orientation(read_quaternion(msg.rotation))
    elif isinstance(msg, geometry.Twist):
        state.set_linear_velocity(read_vector(msg.linear))
        state.set_angular_velocity(read_vector(msg.angular))
    elif isinstance(msg, geometry.Wrench):
        state.set_force(read_vector(msg.force))
        state.set_torque(read_vector(msg.torque))
    elif isinstance(msg, sensor_msgs.msg.JointState):
        state.set_names(msg.name)
        state.set_positions(msg.position)
        state.set_velocities(msg.velocity)
        state.set_torques(msg.effort)
    else:
        raise RuntimeError("This message type is not supported")
    return state


def read_msg_stamped(state, msg):
    if isinstance(msg, geometry.AccelStamped):
        read_msg(state, msg.accel)
        state.set_reference_frame(msg.header.frame_id)
    elif isinstance(msg, geometry.PoseStamped):
        read_msg(state, msg.pose)
        state.set_reference_frame(msg.header.frame_id)
    elif isinstance(msg, geometry.TransformStamped):
        read_msg(state, msg.transform)
        state.set_reference_frame(msg.header.frame_id)
        state.set_name(msg.child_frame_id)
    elif isinstance(msg, geometry.TwistStamped):
        read_msg(state, msg.twist)
        state.set_reference_frame(msg.header.frame_id)
    elif isinstance(msg, geometry.WrenchStamped):
        read_msg(state, msg.wrench)
        state.set_reference_frame(msg.header.frame_id)
    else:
        raise RuntimeError("This message type is not supported")
    return state


def read_clproto_msg(msg: std_msgs.msg.UInt8MultiArray):
    if isinstance(msg, std_msgs.msg.UInt8MultiArray):
        return clproto.decode(msg.data.tobytes())
    else:
        raise RuntimeError("Your input does not have the correct type")
