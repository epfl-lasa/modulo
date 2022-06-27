import re


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
