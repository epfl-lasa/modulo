from typing import List
from typing import TypeVar, Union

import clproto
import geometry_msgs.msg as geometry
import sensor_msgs.msg
import sensor_msgs.msg
import state_representation as sr
from modulo_new_core.encoded_state import EncodedState

MsgT = TypeVar('MsgT')
StateT = TypeVar('StateT')


def read_xyz(message: Union[geometry.Point, geometry.Vector3]) -> List[float]:
    """
    Helper function to read a list from a Point or Vector3 message.

    :param message: The message to read from
    """
    return [message.x, message.y, message.z]


def read_quaternion(message: geometry.Quaternion) -> List[float]:
    """
    Helper function to read a list of quaternion coefficients (w,x,y,z) from a Quaternion message.

    :param message: The message to read from
    """
    return [message.w, message.x, message.y, message.z]


def read_message(state: StateT, message: MsgT):
    """
    Convert a ROS message to a state_representation State.

    :param state: The state to populate
    :param message: The ROS message to read from
    """
    if not isinstance(state, sr.State):
        raise RuntimeError("This state type is not supported.")
    if isinstance(state, sr.CartesianState):
        if isinstance(message, geometry.Accel):
            state.set_linear_acceleration(read_xyz(message.linear))
            state.set_angular_acceleration(read_xyz(message.angular))
        elif isinstance(message, geometry.Pose):
            state.set_position(read_xyz(message.position))
            state.set_orientation(read_quaternion(message.orientation))
        elif isinstance(message, geometry.Transform):
            state.set_position(read_xyz(message.translation))
            state.set_orientation(read_quaternion(message.rotation))
        elif isinstance(message, geometry.Twist):
            state.set_linear_velocity(read_xyz(message.linear))
            state.set_angular_velocity(read_xyz(message.angular))
        elif isinstance(message, geometry.Wrench):
            state.set_force(read_xyz(message.force))
            state.set_torque(read_xyz(message.torque))
        else:
            raise RuntimeError("The provided combination of state type and message type is not supported")
    elif isinstance(message, sensor_msgs.msg.JointState) and isinstance(state, sr.JointState):
        state.set_names(message.name)
        if len(message.position):
            state.set_positions(message.position)
        if len(message.velocity):
            state.set_velocities(message.velocity)
        if len(message.effort):
            state.set_torques(message.effort)
    else:
        raise RuntimeError("The provided combination of state type and message type is not supported")
    return state


def read_stamped_message(state: StateT, message: MsgT):
    """
    Convert a stamped ROS message to a state_representation State.

    :param state: The state to populate
    :param message: The ROS message to read from
    """
    if isinstance(message, geometry.AccelStamped):
        read_message(state, message.accel)
    elif isinstance(message, geometry.PoseStamped):
        read_message(state, message.pose)
    elif isinstance(message, geometry.TransformStamped):
        read_message(state, message.transform)
        state.set_name(message.child_frame_id)
    elif isinstance(message, geometry.TwistStamped):
        read_message(state, message.twist)
    elif isinstance(message, geometry.WrenchStamped):
        read_message(state, message.wrench)
    else:
        raise RuntimeError("The provided combination of state type and message type is not supported")
    state.set_reference_frame(message.header.frame_id)


def read_clproto_message(message: EncodedState) -> StateT:
    """
    Convert an EncodedState message to a state_representation State.

    :param message: The EncodedState message to read from
    """
    return clproto.decode(message.data.tobytes())
