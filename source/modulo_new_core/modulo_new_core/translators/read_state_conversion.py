from typing import List

import clproto
import geometry_msgs.msg as geometry
import sensor_msgs.msg
from modulo_new_core.encoded_state import EncodedState


def __read_xyz(msg) -> List[float]:
    return [msg.x, msg.y, msg.z]


def __read_quaternion(msg: geometry.Quaternion) -> List[float]:
    return [msg.w, msg.x, msg.y, msg.z]


def read_msg(state, msg):
    if isinstance(msg, geometry.Accel):
        state.set_linear_acceleration(__read_xyz(msg.linear))
        state.set_angular_acceleration(__read_xyz(msg.angular))
    elif isinstance(msg, geometry.Pose):
        state.set_position(__read_xyz(msg.position))
        state.set_orientation(__read_quaternion(msg.orientation))
    elif isinstance(msg, geometry.Transform):
        state.set_position(__read_xyz(msg.translation))
        state.set_orientation(__read_quaternion(msg.rotation))
    elif isinstance(msg, geometry.Twist):
        state.set_linear_velocity(__read_xyz(msg.linear))
        state.set_angular_velocity(__read_xyz(msg.angular))
    elif isinstance(msg, geometry.Wrench):
        state.set_force(__read_xyz(msg.force))
        state.set_torque(__read_xyz(msg.torque))
    elif isinstance(msg, sensor_msgs.msg.JointState):
        state.set_names(msg.name)
        if len(msg.position):
            state.set_positions(msg.position)
        if len(msg.velocity):
            state.set_velocities(msg.velocity)
        if len(msg.effort):
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


def read_clproto_msg(msg: EncodedState):
    if isinstance(msg, EncodedState):
        return clproto.decode(msg.data.tobytes())
    else:
        raise RuntimeError("Your input does not have the correct type")
