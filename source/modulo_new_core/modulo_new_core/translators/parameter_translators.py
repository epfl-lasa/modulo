from rclpy import Parameter
import state_representation as sr
import clproto
import numpy as np
from typing import Union


def write_parameter(parameter: sr.Parameter) -> Parameter:
    if parameter.get_parameter_type() == sr.ParameterType.BOOL or \
            parameter.get_parameter_type() == sr.ParameterType.BOOL_ARRAY or \
            parameter.get_parameter_type() == sr.ParameterType.INT or \
            parameter.get_parameter_type() == sr.ParameterType.INT_ARRAY or \
            parameter.get_parameter_type() == sr.ParameterType.DOUBLE or \
            parameter.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY or \
            parameter.get_parameter_type() == sr.ParameterType.STRING or \
            parameter.get_parameter_type() == sr.ParameterType.STRING_ARRAY:
        return Parameter(parameter.get_name(), value=parameter.get_value())
    elif parameter.get_parameter_type() == sr.ParameterType.STATE:
        if parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_STATE:
            return Parameter(parameter.get_name(), Parameter.Type.STRING, clproto.to_json(
                clproto.encode(parameter.get_value(), clproto.MessageType.CARTESIAN_STATE_MESSAGE)))
        elif parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_POSE:
            return Parameter(parameter.get_name(), Parameter.Type.STRING, clproto.to_json(
                clproto.encode(parameter.get_value(), clproto.MessageType.CARTESIAN_POSE_MESSAGE)))
        elif parameter.get_parameter_state_type() == sr.StateType.JOINT_STATE:
            return Parameter(parameter.get_name(), Parameter.Type.STRING, clproto.to_json(
                clproto.encode(parameter.get_value(), clproto.MessageType.JOINT_STATE_MESSAGE)))
        if parameter.get_parameter_state_type() == sr.StateType.JOINT_POSITIONS:
            return Parameter(parameter.get_name(), Parameter.Type.STRING, clproto.to_json(
                clproto.encode(parameter.get_value(), clproto.MessageType.JOINT_POSITIONS_MESSAGE)))
        else:
            raise RuntimeError(f"Parameter {parameter.get_name()} could not be written!")
    elif parameter.get_parameter_type() == sr.ParameterType.VECTOR or \
            parameter.get_parameter_type() == sr.ParameterType.MATRIX:
        return Parameter(parameter.get_name(), value=parameter.get_value().flatten().tolist())
    else:
        raise RuntimeError(f"Parameter {parameter.get_name()} could not be written!")


def copy_parameter_value(source_parameter: sr.Parameter, parameter: sr.Parameter):
    if source_parameter.get_parameter_type() != parameter.get_parameter_type():
        raise RuntimeError(f"Source parameter {source_parameter.get_name()} to be copied does not have the same type "
                           f"as destination parameter {parameter.get_name()}")
    if source_parameter.get_parameter_type() == sr.ParameterType.BOOL or \
            source_parameter.get_parameter_type() == sr.ParameterType.BOOL_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.INT or \
            source_parameter.get_parameter_type() == sr.ParameterType.INT_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.DOUBLE or \
            source_parameter.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.STRING or \
            source_parameter.get_parameter_type() == sr.ParameterType.STRING_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.VECTOR or \
            source_parameter.get_parameter_type() == sr.ParameterType.MATRIX:
        parameter.set_value(source_parameter.get_value())
    elif source_parameter.get_parameter_type() == sr.ParameterType.STATE:
        if source_parameter.get_parameter_state_type() != parameter.get_parameter_state_type():
            raise RuntimeError(
                f"Source parameter {source_parameter.get_name()} to be copied does not have the same state type "
                f"as destination parameter {parameter.get_name()}")
        if source_parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_STATE or \
                source_parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_POSE or \
                source_parameter.get_parameter_state_type() == sr.StateType.JOINT_STATE or \
                source_parameter.get_parameter_state_type() == sr.StateType.JOINT_POSITIONS:
            parameter.set_value(source_parameter.get_value())
        else:
            raise RuntimeError(f"Could not copy the value from source parameter {source_parameter.get_name()} "
                               f"into parameter {parameter.get_name()}")
    else:
        raise RuntimeError(f"Could not copy the value from source parameter {source_parameter.get_name()} "
                           f"into parameter {parameter.get_name()}")


def read_parameter(ros_parameter: Parameter, parameter: Union[None, sr.Parameter] = None) -> sr.Parameter:
    def read_new_parameter(ros_param: Parameter) -> sr.Parameter:
        if ros_param.type_ == Parameter.Type.BOOL:
            return sr.Parameter(ros_param.name, ros_param.get_parameter_value().bool_value, sr.ParameterType.BOOL)
        elif ros_param.type_ == Parameter.Type.BOOL_ARRAY:
            return sr.Parameter(ros_param.name, ros_param.get_parameter_value().bool_array_value,
                                sr.ParameterType.BOOL_ARRAY)
        elif ros_param.type_ == Parameter.Type.INTEGER:
            return sr.Parameter(ros_param.name, ros_param.get_parameter_value().integer_value, sr.ParameterType.INT)
        elif ros_param.type_ == Parameter.Type.INTEGER_ARRAY:
            return sr.Parameter(ros_param.name, ros_param.get_parameter_value().integer_array_value,
                                sr.ParameterType.INT_ARRAY)
        elif ros_param.type_ == Parameter.Type.DOUBLE:
            return sr.Parameter(ros_param.name, ros_param.get_parameter_value().double_value, sr.ParameterType.DOUBLE)
        elif ros_param.type_ == Parameter.Type.DOUBLE_ARRAY:
            return sr.Parameter(ros_param.name, ros_param.get_parameter_value().double_array_value,
                                sr.ParameterType.DOUBLE_ARRAY)
        elif ros_param.type_ == Parameter.Type.STRING_ARRAY:
            return sr.Parameter(ros_param.name, ros_param.get_parameter_value().string_array_value,
                                sr.ParameterType.STRING_ARRAY)
        elif ros_param.type_ == Parameter.Type.STRING:
            encoding = ""
            try:
                encoding = clproto.from_json(ros_param.get_parameter_value().string_value)
            except Exception:
                if not clproto.is_valid(encoding):
                    return sr.Parameter(ros_param.name, ros_param.get_parameter_value().string_value,
                                        sr.ParameterType.STRING)
            message_type = clproto.check_message_type(encoding)
            if message_type == clproto.MessageType.CARTESIAN_STATE_MESSAGE:
                return sr.Parameter(ros_param.name, clproto.decode(encoding), sr.ParameterType.STATE,
                                    sr.StateType.CARTESIAN_STATE)
            elif message_type == clproto.MessageType.CARTESIAN_POSE_MESSAGE:
                return sr.Parameter(ros_param.name, clproto.decode(encoding), sr.ParameterType.STATE,
                                    sr.StateType.CARTESIAN_POSE)
            elif message_type == clproto.MessageType.JOINT_STATE_MESSAGE:
                return sr.Parameter(ros_param.name, clproto.decode(encoding), sr.ParameterType.STATE,
                                    sr.StateType.JOINT_STATE)
            elif message_type == clproto.MessageType.JOINT_POSITIONS_MESSAGE:
                return sr.Parameter(ros_param.name, clproto.decode(encoding), sr.ParameterType.STATE,
                                    sr.StateType.JOINT_POSITIONS)
            else:
                raise RuntimeError(f"Parameter {ros_param.name} has an unsupported encoded message type!")
        elif ros_param.type_ == Parameter.Type.BYTE_ARRAY:
            raise RuntimeError("Parameter byte arrays are not currently supported.")
        else:
            raise RuntimeError(f"Parameter {ros_param.name} could not be read!")

    new_parameter = read_new_parameter(ros_parameter)
    if parameter is None:
        return new_parameter
    else:
        copy_parameter_value(new_parameter, parameter)
        return parameter


def read_parameter_const(ros_parameter: Parameter, parameter: sr.Parameter) -> sr.Parameter:
    if ros_parameter.name != parameter.get_name():
        raise RuntimeError(
            f"The ROS parameter {ros_parameter.name} to be read does not have the same name as the reference parameter {parameter.get_name()}")
    new_parameter = read_parameter(ros_parameter)
    if new_parameter.get_parameter_type() == parameter.get_parameter_type():
        return new_parameter
    elif new_parameter.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY:
        value = new_parameter.get_value()
        if parameter.get_parameter_type() == sr.ParameterType.VECTOR:
            vector = np.array(value)
            return sr.Parameter(parameter.get_name(), vector, sr.ParameterType.VECTOR)
        elif parameter.get_parameter_type() == sr.ParameterType.MATRIX:
            matrix = parameter.get_value()
            if matrix.size != len(value):
                raise RuntimeError(f"The ROS parameter {ros_parameter.name} with type double array has size "
                                   f"{len(value)} while the reference parameter matrix {parameter.get_name()} "
                                   f"has size {len(matrix)}")
            matrix = np.array(value).reshape(matrix.shape)
            return sr.Parameter(parameter.get_name(), matrix, sr.ParameterType.MATRIX)
        else:
            raise RuntimeError(f"The ROS parameter {ros_parameter.name} with type double array cannot be interpreted "
                               f"by reference parameter {parameter.get_name()} (type code "
                               f"{parameter.get_parameter_type()}")
    else:
        raise RuntimeError(f"Something went wrong while reading parameter {parameter.get_name()}")
