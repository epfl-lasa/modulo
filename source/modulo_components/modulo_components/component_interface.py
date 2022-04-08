from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool

from typing import Union
from typing import Callable


class ComponentInterface(Node):
    """
    Abstract class to represent a Component in python, following the same logic pattern
    as the c++ modulo_component::ComponentInterface class.
    ...
    Attributes:
        predicates (dict(Parameters(bool))): map of predicates added to the Component.
        predicate_publishers (dict(rclpy.Publisher(Bool))): map of publishers associated to each predicate.
        tf_buffer (tf2_ros.Buffer): the buffer to lookup transforms published on tf2.
        tf_listener (tf2_ros.TransformListener): the listener to lookup transforms published on tf2.
        tf_broadcaster (tf2_ros.TransformBroadcaster): the broadcaster to publish transforms on tf2
    Parameters:
        period (double): period (in s) between step function calls.
        has_tf_listener (bool): if true, add a tf listener to enable calls to lookup transforms.
        has_tf_broadcaster (bool): if true, add a tf broadcaster to enable the publishing of transforms.
    """

    def __init__(self, node_name, *kargs, **kwargs):
        """
        Constructs all the necessary attributes and declare all the parameters.
            Parameters:
                node_name (str): name of the node to be passed to the base Node class
        """
        super().__init__(node_name, *kargs, **kwargs)
        self.declare_parameter('period', 0.1,
                               ParameterDescriptor(description="Period (in s) between step function calls."))
        self.declare_parameter('has_tf_listener', False,
                               ParameterDescriptor(
                                   description="If true, add a tf listener to enable calls to lookup transforms."))
        self.declare_parameter('has_tf_broadcaster', False,
                               ParameterDescriptor(
                                   description="If true, add a tf broadcaster to enable the publishing of transforms."))

        self._period = self.get_parameter('period').get_parameter_value().double_value
        self._has_tf_listener = self.get_parameter('has_tf_listener').get_parameter_value().double_value
        self._has_tf_broadcaster = self.get_parameter('has_tf_broadcaster').get_parameter_value().double_value
        self._predicates = {}
        self._predicate_publishers = {}

        if self._has_tf_listener:
            self.__tf_buffer = Buffer()
            self.__tf_listener = TransformListener(self.__tf_buffer, self)
        if self._has_tf_broadcaster:
            self.__tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self._period, self.__step)

    def __step(self):
        for predicate_name in self._predicates.keys():
            msg = Bool()
            msg.data = self.get_predicate(predicate_name)
            if predicate_name not in self._predicate_publishers.keys():
                self.get_logger().error(f"No publisher for predicate {predicate_name} found.",
                                        throttle_duration_sec=1.0)
                return
            self._predicate_publishers[predicate_name].publish(msg)

    def __generate_predicate_topic(self, predicate_name: str) -> str:
        return f'/predicates/{self.get_name()}/{predicate_name}'

    def add_predicate(self, predicate_name: str, predicate_value: Union[bool, Callable]):
        """
        Add a predicate to the map of predicates.

        :param predicate_name: the name of the associated predicate
        :param predicate_value: the value of the predicate as a bool or a callable function
        """
        if predicate_name in self._predicates.keys():
            self.get_logger().debug(f"Predicate {predicate_name} already exists, overwriting.")
        else:
            self._predicate_publishers[predicate_name] = self.create_publisher(Bool, self.__generate_predicate_topic(
                predicate_name), 10)
        self._predicates[predicate_name] = predicate_value

    def get_predicate(self, predicate_name: str) -> bool:
        """
        Get the value of the predicate given as parameter, if the predicate is not found or the callable function fails
        return false.

        :param predicate_name: the name of the predicate to retrieve from the map of predicates
        :return: the value of the predicate as a boolean
        """
        if predicate_name not in self._predicates.keys():
            self.get_logger().error(f"Predicate {predicate_name} does not exists, returning false.",
                                    throttle_duration_sec=1.0)
            return False
        value = self._predicates[predicate_name]
        if callable(value):
            bool_value = False
            try:
                bool_value = value()
            except Exception as e:
                self.get_logger().error(f"Error while evaluating the callback function: {e}",
                                        throttle_duration_sec=1.0)
            return bool_value
        return value

    def set_predicate(self, predicate_name: str, predicate_value: Union[bool, Callable]):
        """
        Set the value of the predicate given as parameter, if the predicate is not found does not do anything.

        :param predicate_name: the name of the predicate to retrieve from the map of predicates
        :param predicate_value: the new value of the predicate as a bool or a callable function
        """
        if predicate_name not in self._predicates.keys():
            self.get_logger().error(f"Predicate {predicate_name} does not exists, can't set it with a new value.",
                                    throttle_duration_sec=1.0)
            return
        self._predicates[predicate_name] = predicate_value
