import re

import state_representation as sr
from rclpy import Parameter


def generate_predicate_topic(node_name: str, predicate_name: str) -> str:
    """
    Generate a topic name from the name of node and the predicate.

    :param node_name: The name of the node
    :param predicate_name: The name of the associated predicate
    :return: The name of the topic
    """
    return f'/predicates/{node_name}/{predicate_name}'


def parse_signal_name(signal_name: str) -> str:
    """
    Parse a string signal name from a user-provided input.
    This functions removes all characters different from a-z, A-Z, 0-9, and _ from a string.

    :param signal_name: The input string
    :return: The sanitized string
    """
    sanitized_string = re.sub("\W", "", signal_name, flags=re.ASCII).lower()
    sanitized_string = sanitized_string.lstrip("_")
    return sanitized_string


def get_ros_parameter_type(parameter_type: sr.ParameterType) -> Parameter.Type:
    """
    Given a state representation parameter type, get the corresponding ROS parameter type.

    :param parameter_type: The state representation parameter type
    :return: The corresponding ROS parameter type
    """
    if parameter_type == sr.ParameterType.BOOL:
        return Parameter.Type.BOOL
    elif parameter_type == sr.ParameterType.BOOL_ARRAY:
        return Parameter.Type.BOOL_ARRAY
    elif parameter_type == sr.ParameterType.INT:
        return Parameter.Type.INTEGER
    elif parameter_type == sr.ParameterType.INT_ARRAY:
        return Parameter.Type.INTEGER_ARRAY
    elif parameter_type == sr.ParameterType.DOUBLE:
        return Parameter.Type.DOUBLE
    elif parameter_type in [sr.ParameterType.DOUBLE_ARRAY, sr.ParameterType.VECTOR, sr.ParameterType.MATRIX]:
        return Parameter.Type.DOUBLE_ARRAY
    elif parameter_type in [sr.ParameterType.STRING, sr.ParameterType.STATE]:
        return Parameter.Type.STRING
    elif parameter_type == sr.ParameterType.STRING_ARRAY:
        return Parameter.Type.STRING_ARRAY
    else:
        return Parameter.Type.NOT_SET
