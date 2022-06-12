from typing import List
from typing import TypeVar, Union

import clproto
import geometry_msgs.msg as geometry
import sensor_msgs.msg
import sensor_msgs.msg
import state_representation as sr
from modulo_new_core.encoded_state import EncodedState

DataT = TypeVar('DataT')
MsgT = TypeVar('MsgT')
StateT = TypeVar('StateT')


def read_xyz(msg: Union[geometry.Point, geometry.Vector3]) -> List[float]:
    """
    Helper function to read a list from a Point or Vector3 message.

    :param msg: The message to read from
    """
    return [msg.x, msg.y, msg.z]


def read_quaternion(msg: geometry.Quaternion) -> List[float]:
    """
    Helper function to read a list of quaternion coefficients (w,x,y,z) from a Quaternion message.

    :param msg: The message to read from
    """
    return [msg.w, msg.x, msg.y, msg.z]


def read_msg(state: StateT, msg: MsgT):
    """
    Convert a ROS message to a state_representation State.

    :param state: The state to populate
    :param msg: The ROS message to read from
    """
    if not isinstance(state, sr.State):
        raise RuntimeError("This state type is not supported.")
    if isinstance(state, sr.CartesianState):
        if isinstance(msg, geometry.Accel):
            state.set_linear_acceleration(read_xyz(msg.linear))
            state.set_angular_acceleration(read_xyz(msg.angular))
        elif isinstance(msg, geometry.Pose):
            state.set_position(read_xyz(msg.position))
            state.set_orientation(read_quaternion(msg.orientation))
        elif isinstance(msg, geometry.Transform):
            state.set_position(read_xyz(msg.translation))
            state.set_orientation(read_quaternion(msg.rotation))
        elif isinstance(msg, geometry.Twist):
            state.set_linear_velocity(read_xyz(msg.linear))
            state.set_angular_velocity(read_xyz(msg.angular))
        elif isinstance(msg, geometry.Wrench):
            state.set_force(read_xyz(msg.force))
            state.set_torque(read_xyz(msg.torque))
        else:
            raise RuntimeError("The provided combination of state type and message type is not supported")
    elif isinstance(msg, sensor_msgs.msg.JointState) and isinstance(state, sr.JointState):
        state.set_names(msg.name)
        if len(msg.position):
            state.set_positions(msg.position)
        if len(msg.velocity):
            state.set_velocities(msg.velocity)
        if len(msg.effort):
            state.set_torques(msg.effort)
    else:
        raise RuntimeError("The provided combination of state type and message type is not supported")
    return state


def read_stamped_msg(state: StateT, msg: MsgT):
    """
    Convert a stamped ROS message to a state_representation State.

    :param state: The state to populate
    :param msg: The ROS message to read from
    """
    if isinstance(msg, geometry.AccelStamped):
        read_msg(state, msg.accel)
    elif isinstance(msg, geometry.PoseStamped):
        read_msg(state, msg.pose)
    elif isinstance(msg, geometry.TransformStamped):
        read_msg(state, msg.transform)
        state.set_name(msg.child_frame_id)
    elif isinstance(msg, geometry.TwistStamped):
        read_msg(state, msg.twist)
    elif isinstance(msg, geometry.WrenchStamped):
        read_msg(state, msg.wrench)
    else:
        raise RuntimeError("The provided combination of state type and message type is not supported")
    state.set_reference_frame(msg.header.frame_id)
    return state


def read_std_msg(msg: MsgT) -> DataT:
    """
    Read the data field of an std_msg.msg message.

    :param msg: The message to read from
    """
    return msg.data


def read_clproto_msg(msg: EncodedState) -> StateT:
    """
    Convert an EncodedState message to a state_representation State.

    :param msg: The EncodedState message to read from
    """
    return clproto.decode(msg.data.tobytes())
