def generate_predicate_topic(node_name: str, predicate_name: str) -> str:
    """
    Generate a topic name from the name of node and the predicate.

    :param node_name: The name of the node
    :param predicate_name: The name of the associated predicate
    :return: The name of the topic
    """
    return f'/predicates/{node_name}/{predicate_name}'
