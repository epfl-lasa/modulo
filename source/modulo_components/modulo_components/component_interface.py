import sys
from typing import Callable, Dict, List, Optional, TypeVar, Union

import rclpy
import state_representation as sr
import tf2_py
from geometry_msgs.msg import TransformStamped
from modulo_components.exceptions.component_exceptions import ComponentParameterError, LookupTransformError
from modulo_components.utilities.utilities import generate_predicate_topic
from modulo_new_core.translators.message_readers import read_stamped_message
from modulo_new_core.translators.message_writers import write_stamped_message
from modulo_new_core.translators.parameter_translators import write_parameter, read_parameter_const
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

T = TypeVar('T')


class ComponentInterface(Node):
    """
    Abstract class to represent a Component in python, following the same logic pattern  as the C++
    modulo_component::ComponentInterface class.
    ...
    Attributes:
        predicates (dict(Parameters(bool))): map of predicates added to the Component.
        predicate_publishers (dict(rclpy.Publisher(Bool))): map of publishers associated to each predicate.
        parameter_dict: dict of parameters
        periodic_callbacks: dict of periodic callback functions
        qos: the Quality of Service for publishers and subscribers
        tf_buffer (tf2_ros.Buffer): the buffer to lookup transforms published on tf2.
        tf_listener (tf2_ros.TransformListener): the listener to lookup transforms published on tf2.
        tf_broadcaster (tf2_ros.TransformBroadcaster): the broadcaster to publish transforms on tf2
    Parameters:
        period (double): period (in s) between step function calls.
    """

    def __init__(self, node_name: str, *kargs, **kwargs):
        """
        Constructs all the necessary attributes and declare all the parameters.
            Parameters:
                node_name (str): name of the node to be passed to the base Node class
        """
        super().__init__(node_name, *kargs, **kwargs)
        self._parameter_dict: Dict[str, Union[str, sr.Parameter]] = {}
        self._predicates: Dict[str, Union[bool, Callable[[], bool]]] = {}
        self._predicate_publishers: Dict[str, Publisher] = {}
        self._periodic_callbacks: Dict[str, Callable[[], None]] = {}
        self.__tf_buffer: Optional[Buffer] = None
        self.__tf_listener: Optional[TransformListener] = None
        self.__tf_broadcaster: Optional[TransformBroadcaster] = None

        self._qos = QoSProfile(depth=10)

        self.add_on_set_parameters_callback(self.__on_set_parameters_callback)
        self.add_parameter(sr.Parameter("period", 0.1, sr.ParameterType.DOUBLE),
                           "Period (in s) between step function calls.")

        self.add_predicate("in_error_state", False)

        self.create_timer(self.get_parameter_value("period"), self._step)

    def _step(self) -> None:
        """
        Step function that is called periodically.
        """
        pass

    def add_parameter(self, parameter: Union[str, sr.Parameter], description: str, read_only=False) -> None:
        """
        Add a parameter. This method stores either the name of the attribute corresponding to the parameter object or
        a parameter object directly in the local parameter dictionary and declares the equivalent ROS parameter on the
        ROS interface. If an attribute name is provided, changing the ROS parameter will also update the provided
        attribute and vice versa. If the provided argument is not an attribute name or a Parameter object, nothing
        happens.

        :param parameter: Either the name of the parameter attribute or the parameter itself
        :param description: The parameter description
        :param read_only: If True, the value of the parameter cannot be changed after declaration
        """
        try:
            if isinstance(parameter, sr.Parameter):
                sr_parameter = parameter
            elif isinstance(parameter, str):
                attr = self.__getattribute__(parameter)
                if isinstance(attr, sr.Parameter):
                    sr_parameter = attr
                else:
                    raise TypeError(
                        f"The attribute with the provided name '{parameter}' does not contain a Parameter object.")
            else:
                raise TypeError("Provide either a state_representation.Parameter object or a string "
                                "containing the name of the attribute that refers to the parameter to add.")
            ros_param = write_parameter(sr_parameter)
            if not self.has_parameter(sr_parameter.get_name()):
                self.get_logger().debug(f"Adding parameter '{sr_parameter.get_name()}'.")
                self._parameter_dict[sr_parameter.get_name()] = parameter
                # TODO ignore override
                self.declare_parameter(ros_param.name, ros_param.value,
                                       descriptor=ParameterDescriptor(description=description),
                                       ignore_override=read_only)
            else:
                self.get_logger().debug(f"Parameter '{sr_parameter.get_name()}' already exists, overwriting.")
                self.set_parameters([ros_param])
        except Exception as e:
            self.get_logger().error(f"Failed to add parameter: {e}")

    def get_parameter(self, name: str) -> Union[sr.Parameter, rclpy.parameter.Parameter]:
        """
        Get a parameter by name. If this method is called from a file that contains 'rclpy' in its path, the
        rclpy.node.Node.get_parameter method will be invoked, otherwise return the parameter from the local parameter
        dictionary.

        :param name: The name of the parameter
        :raises ComponentParameterError if the parameter does not exist
        :return: The requested parameter
        """
        try:
            co_filename = sys._getframe().f_back.f_code.co_filename
            self.get_logger().debug(f"get_parameter called from {co_filename}")
            if "rclpy" in co_filename:
                return rclpy.node.Node.get_parameter(self, name)
            else:
                return self._get_component_parameter(name)
        except Exception as e:
            raise ComponentParameterError(f"Failed to get parameter '{name}': {e}")

    def _get_component_parameter(self, name: str) -> sr.Parameter:
        """
        Get the parameter from the parameter dictionary by its name.

        :param name: The name of the parameter
        :raises ComponentParameterError if the parameter does not exist
        :return: The parameter, if it exists
        """
        if name not in self._parameter_dict.keys():
            raise ComponentParameterError(f"Parameter '{name}' is not in the dict of parameters")
        try:
            if isinstance(self._parameter_dict[name], str):
                return self.__getattribute__(self._parameter_dict[name])
            else:
                return self._parameter_dict[name]
        except AttributeError as e:
            raise ComponentParameterError(f"{e}")

    def get_parameter_value(self, name: str) -> T:
        """
        Get the parameter value from the parameter dictionary by its name.

        :param name: The name of the parameter
        :raises ComponentParameterError if the parameter does not exist
        :return: The value of the parameter, if the parameter exists
        """
        return self._get_component_parameter(name).get_value()

    def set_parameter_value(self, name: str, value: T, parameter_type: sr.ParameterType) -> None:
        """
        Set the value of a parameter. The parameter must have been previously declared. If the parameter is an
        attribute, the attribute is updated.

        :param name: The name of the parameter
        :param value: The value of the parameter
        :param parameter_type: The type of the parameter
        """
        try:
            ros_param = write_parameter(sr.Parameter(name, value, parameter_type))
            result = self.set_parameters([ros_param])[0]
            if not result.successful:
                self.get_logger().error(f"Failed to set parameter value of parameter '{name}': {result.reason}",
                                        throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f"Failed to set parameter value of parameter '{name}': {e}",
                                    throttle_duration_sec=1.0)

    def _validate_parameter(self, parameter: sr.Parameter) -> bool:
        """
        Parameter validation function to be redefined by derived Component classes. This method is automatically invoked
        whenever the ROS interface tried to modify a parameter. If the validation returns True, the updated parameter
        value (including any modifications) is applied. If the validation returns False, any changes to the parameter
        are discarded and the parameter value is not changed.

        :param parameter: The parameter to be validated
        :return: The validation result
        """
        return True

    def __on_set_parameters_callback(self, ros_parameters: List[rclpy.Parameter]) -> SetParametersResult:
        """
        Callback function to validate and update parameters on change.

        :param ros_parameters: The new parameter objects provided by the ROS interface
        :return: The result of the validation
        """
        result = SetParametersResult(successful=True)
        for ros_param in ros_parameters:
            try:
                parameter = self._get_component_parameter(ros_param.name)
                new_parameter = read_parameter_const(ros_param, parameter)
                if not self._validate_parameter(new_parameter):
                    result.successful = False
                    result.reason = f"Parameter {ros_param.name} could not be set!"
                else:
                    if isinstance(self._parameter_dict[ros_param.name], str):
                        self.__setattr__(self._parameter_dict[ros_param.name], new_parameter)
                    else:
                        self._parameter_dict[ros_param.name] = new_parameter
            except Exception as e:
                result.successful = False
                result.reason += str(e)
        return result

    def add_predicate(self, name: str, value: Union[bool, Callable[[], bool]]):
        """
        Add a predicate to the map of predicates.

        :param name: The name of the associated predicate
        :param value: The value of the predicate as a bool or a callable function
        """
        if not name:
            self.get_logger().error("Failed to add predicate: Provide a non empty string as a name.")
        if name in self._predicates.keys():
            self.get_logger().debug(f"Predicate {name} already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding predicate '{name}'.")
            self._predicate_publishers[name] = self.create_publisher(Bool,
                                                                     generate_predicate_topic(self.get_name(), name),
                                                                     10)
        self._predicates[name] = value

    def get_predicate(self, name: str) -> bool:
        """
        Get the value of the predicate given as parameter. If the predicate is not found or the callable function fails,
        this method returns False.

        :param name: The name of the predicate to retrieve from the map of predicates
        :return: The value of the predicate as a boolean
        """
        if name not in self._predicates.keys():
            self.get_logger().error(f"Predicate {name} does not exist, returning false.",
                                    throttle_duration_sec=1.0)
            return False
        value = self._predicates[name]
        if callable(value):
            bool_value = False
            try:
                bool_value = value()
            except Exception as e:
                self.get_logger().error(f"Error while evaluating the callback function: {e}",
                                        throttle_duration_sec=1.0)
            return bool_value
        return value

    def set_predicate(self, name: str, value: Union[bool, Callable[[], bool]]):
        """
        Set the value of the predicate given as parameter, if the predicate is not found does not do anything.

        :param name: The name of the predicate to retrieve from the map of predicates
        :param value: The new value of the predicate as a bool or a callable function
        """
        if name not in self._predicates.keys():
            self.get_logger().error(
                f"Cannot set predicate {name} with a new value because it does not exist.",
                throttle_duration_sec=1.0)
            return
        self._predicates[name] = value

    def add_tf_broadcaster(self):
        """
        Configure a transform broadcaster.
        """
        if not self.__tf_broadcaster:
            self.get_logger().debug("Adding TF broadcaster.")
            self.__tf_broadcaster = TransformBroadcaster(self)
        else:
            self.get_logger().error("TF broadcaster already exists.")

    def add_tf_listener(self):
        """
        Configure a transform buffer and listener.
        """
        if not self.__tf_buffer or not self.__tf_listener:
            self.get_logger().debug("Adding TF buffer and listener.")
            self.__tf_buffer = Buffer()
            self.__tf_listener = TransformListener(self.__tf_buffer, self)
        else:
            self.get_logger().error("TF buffer and listener already exist.")

    def send_transform(self, transform: sr.CartesianPose):
        """
        Send a transform to TF.

        :param transform: The transform to send
        """
        if not self.__tf_broadcaster:
            self.get_logger().error("Failed to send transform: No TF broadcaster configured.",
                                    throttle_duration_sec=1.0)
            return
        try:
            transform_message = TransformStamped()
            write_stamped_message(transform_message, transform, self.get_clock().now())
            self.__tf_broadcaster.sendTransform(transform_message)
        except tf2_py.TransformException as e:
            self.get_logger().error(f"Failed to send transform: {e}", throttle_duration_sec=1.0)

    def lookup_transform(self, frame_name: str, reference_frame_name="world", time_point=Time(),
                         duration=Duration(nanoseconds=1e4)) -> sr.CartesianPose:
        """
        Look up a transform from TF.

        :param frame_name: The desired frame of the transform
        :param reference_frame_name: The desired reference frame of the transform
        :param time_point: The time at which the value of the transform is desired (default: 0, will get the latest)
        :param duration: How long to block the lookup call before failing
        :return: If it exists, the requested transform
        """
        if not self.__tf_buffer or not self.__tf_listener:
            raise LookupTransformError("Failed to lookup transform: To TF buffer / listener configured.")
        try:
            result = sr.CartesianPose(frame_name, reference_frame_name)
            transform = self.__tf_buffer.lookup_transform(reference_frame_name, frame_name, time_point, duration)
            read_stamped_message(result, transform)
            return result
        except tf2_py.TransformException as e:
            raise LookupTransformError(f"Failed to lookup transform: {e}")

    def get_qos(self) -> QoSProfile:
        """
        Getter of the Quality of Service attribute.
        """
        return self._qos

    def set_qos(self, qos: QoSProfile):
        """
        Setter of the Quality of Service for ROS publishers and subscribers.

        :param qos: The desired Quality of Service
        """
        self._qos = qos

    def add_periodic_callback(self, name: str, callback: Callable[[], None]):
        """
        Add a periodic callback function. The provided function is evaluated periodically at the component step period.

        :param name: The name of the callback
        :param callback: The callback function that is evaluated periodically
        """
        if not name:
            self.get_logger().error("Failed to add periodic function: Provide a non empty string as a name.")
            return
        if name in self._periodic_callbacks.keys():
            self.get_logger().debug(f"Periodic function '{name}' already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding periodic function '{name}'.")
        self._periodic_callbacks[name] = callback

    def _publish_predicates(self):
        """
        Helper function to publish all predicates.
        """
        for name in self._predicates.keys():
            message = Bool()
            message.data = self.get_predicate(name)
            if name not in self._predicate_publishers.keys():
                self.get_logger().error(f"No publisher for predicate {name} found.", throttle_duration_sec=1.0)
                return
            self._predicate_publishers[name].publish(message)

    def _evaluate_periodic_callbacks(self):
        """
        Helper function to evaluate all periodic function callbacks.
        """
        for name, callback in self._periodic_callbacks.items():
            try:
                callback()
            except Exception as e:
                self.get_logger().error(f"Failed to evaluate periodic function callback '{name}': {e}",
                                        throttle_duration_sec=1.0)

    def raise_error(self):
        """
        Put the component in error state by setting the 'in_error_state' predicate to true.
        """
        self.set_predicate("in_error_state", True)
