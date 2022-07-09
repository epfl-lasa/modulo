import sys
from functools import partial
from typing import Callable, Dict, List, Optional, TypeVar, Union

import clproto
import modulo_core.translators.message_readers as modulo_readers
import modulo_core.translators.message_writers as modulo_writers
import state_representation as sr
from geometry_msgs.msg import TransformStamped
from modulo_components.exceptions.component_exceptions import AddSignalError, ComponentParameterError, \
    LookupTransformError
from modulo_components.utilities.utilities import generate_predicate_topic, parse_signal_name
from modulo_core.encoded_state import EncodedState
from modulo_core.exceptions.core_exceptions import ParameterTranslationError
from modulo_core.translators.parameter_translators import get_ros_parameter_type, read_parameter_const, write_parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray, String
from tf2_py import TransformException
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

MsgT = TypeVar('MsgT')
T = TypeVar('T')


class ComponentInterface(Node):
    """
    Abstract class to represent a Component in python, following the same logic pattern  as the C++
    modulo_component::ComponentInterface class.
    """

    def __init__(self, node_name: str, *kargs, **kwargs):
        super().__init__(node_name, *kargs, **kwargs)
        self._parameter_dict: Dict[str, Union[str, sr.Parameter]] = {}
        self._predicates: Dict[str, Union[bool, Callable[[], bool]]] = {}
        self._predicate_publishers: Dict[str, Publisher] = {}
        self._periodic_callbacks: Dict[str, Callable[[], None]] = {}
        self._inputs = {}
        self._outputs = {}
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
        :raises ComponentParameterError if the parameter could not be added
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
        except (TypeError, ParameterTranslationError) as e:
            raise ComponentParameterError(f"Failed to add parameter: {e}")
        if not self.has_parameter(sr_parameter.get_name()):
            self.get_logger().debug(f"Adding parameter '{sr_parameter.get_name()}'.")
            self._parameter_dict[sr_parameter.get_name()] = parameter
            try:
                descriptor = ParameterDescriptor(description=description, read_only=read_only)
                if sr_parameter.is_empty():
                    descriptor.dynamic_typing = True
                    descriptor.type = get_ros_parameter_type(sr_parameter.get_parameter_type()).value
                    self.declare_parameter(ros_param.name, None, descriptor=descriptor)
                else:
                    self.declare_parameter(ros_param.name, ros_param.value, descriptor=descriptor)
            except Exception as e:
                del self._parameter_dict[sr_parameter.get_name()]
                raise ComponentParameterError(f"Failed to add parameter: {e}")
        else:
            self.get_logger().warn(f"Parameter '{sr_parameter.get_name()}' already exists.")

    def get_parameter(self, name: str) -> Union[sr.Parameter, Parameter]:
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
                return Node.get_parameter(self, name)
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

    def __on_set_parameters_callback(self, ros_parameters: List[Parameter]) -> SetParametersResult:
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
                    result.reason = f"Validation of parameter '{ros_param.name}' returned false!"
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
            self.get_logger().warn(f"Predicate {name} already exists, overwriting.")
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

    def _create_output(self, signal_name: str, data: str, message_type: MsgT,
                       clproto_message_type: clproto.MessageType, fixed_topic: bool, default_topic: str) -> str:
        """
        Helper function to parse the signal name and add an output without Publisher to the dict of outputs.

        :param signal_name: Name of the output signal
        :param data: Name of the attribute to transmit over the channel
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :param default_topic: If set, the default value for the topic name to use
        :raises AddSignalError if there is a problem adding the output
        :return: The parsed signal name
        """
        try:
            if message_type == EncodedState and clproto_message_type == clproto.MessageType.UNKNOWN_MESSAGE:
                raise AddSignalError(f"Provide a valid clproto message type for outputs of type EncodedState.")
            parsed_signal_name = parse_signal_name(signal_name)
            if not parsed_signal_name:
                raise AddSignalError("The parsed signal name is empty. Provide a string with valid "
                                     "characters for the signal name ([a-zA-Z0-9_]).")
            if parsed_signal_name in self._outputs.keys():
                raise AddSignalError(f"Output with parsed name '{parsed_signal_name}' already exists.")
            topic_name = default_topic if default_topic else "~/" + parsed_signal_name
            parameter_name = parsed_signal_name + "_topic"
            if self.has_parameter(parameter_name) and self.get_parameter(parameter_name).is_empty():
                self.set_parameter_value(parameter_name, topic_name)
            else:
                self.add_parameter(sr.Parameter(parameter_name, topic_name, sr.ParameterType.STRING),
                                   f"Output topic name of signal '{parsed_signal_name}'", fixed_topic)
            translator = None
            if message_type == Bool or message_type == Float64 or \
                    message_type == Float64MultiArray or message_type == Int32 or message_type == String:
                translator = modulo_writers.write_std_message
            elif message_type == EncodedState:
                translator = partial(modulo_writers.write_clproto_message,
                                     clproto_message_type=clproto_message_type)
            else:
                raise AddSignalError("The provided message type is not supported to create a component output")
            self._outputs[parsed_signal_name] = {"attribute": data, "message_type": message_type,
                                                 "translator": translator}
            return parsed_signal_name
        except Exception as e:
            raise AddSignalError(f"{e}")

    def __subscription_callback(self, message: MsgT, attribute_name: str, reader: Callable):
        """
        Subscription callback for the ROS subscriptions.

        :param message: The message from the ROS network
        :param attribute_name: The name of the attribute that is updated by the subscription
        :param reader: A callable that can read the ROS message and translate to the desired type
        """
        try:
            self.__setattr__(attribute_name, reader(message))
        except Exception as e:
            self.get_logger().error(f"Failed to read message for attribute {attribute_name}", throttle_duration_sec=1.0)

    def add_input(self, signal_name: str, subscription: Union[str, Callable], message_type: MsgT, fixed_topic=False,
                  default_topic=""):
        # TODO could be nice to add an optional callback here that would be executed from within the subscription
        #  callback in order to manipulate the data pointer upon reception of a message
        """
        Add and configure an input signal of the component.

        :param signal_name: Name of the output signal
        :param subscription: Name of the attribute to receive messages for or the callback to use for the subscription
        :param message_type: ROS message type of the subscription
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :param default_topic: If set, the default value for the topic name to use
        """
        try:
            parsed_signal_name = parse_signal_name(signal_name)
            if not parsed_signal_name:
                raise AddSignalError(f"Failed to add input '{signal_name}': Parsed signal name is empty.")
            if parsed_signal_name in self._inputs.keys():
                raise AddSignalError(f"Failed to add input '{parsed_signal_name}': Input already exists")
            topic_name = default_topic if default_topic else "~/" + parsed_signal_name
            parameter_name = parsed_signal_name + "_topic"
            if self.has_parameter(parameter_name) and self.get_parameter(parameter_name).is_empty():
                self.set_parameter_value(parameter_name, topic_name)
            else:
                self.add_parameter(sr.Parameter(parameter_name, topic_name, sr.ParameterType.STRING),
                                   f"Input topic name of signal '{parsed_signal_name}'", fixed_topic)
            topic_name = self.get_parameter_value(parsed_signal_name + "_topic")
            self.get_logger().debug(f"Adding input '{parsed_signal_name}' with topic name '{topic_name}'.")
            if isinstance(subscription, Callable):
                self._inputs[parsed_signal_name] = self.create_subscription(message_type, topic_name, subscription,
                                                                            self._qos)
            elif isinstance(subscription, str):
                if message_type == Bool or message_type == Float64 or \
                        message_type == Float64MultiArray or message_type == Int32 or message_type == String:
                    self._inputs[parsed_signal_name] = self.create_subscription(message_type, topic_name,
                                                                                partial(self.__subscription_callback,
                                                                                        attribute_name=subscription,
                                                                                        reader=modulo_readers.read_std_message),
                                                                                self._qos)
                elif message_type == EncodedState:
                    self._inputs[parsed_signal_name] = self.create_subscription(message_type, topic_name,
                                                                                partial(self.__subscription_callback,
                                                                                        attribute_name=subscription,
                                                                                        reader=modulo_readers.read_clproto_message),
                                                                                self._qos)
                else:
                    raise TypeError("The provided message type is not supported to create a component input")
            else:
                raise TypeError("Provide either a string containing the name of an attribute or a callable.")
        except Exception as e:
            self.get_logger().error(f"Failed to add input '{signal_name}': {e}")

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
            modulo_writers.write_stamped_message(transform_message, transform, self.get_clock().now())
            self.__tf_broadcaster.sendTransform(transform_message)
        except TransformException as e:
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
            modulo_readers.read_stamped_message(result, transform)
            return result
        except TransformException as e:
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
            self.get_logger().warn(f"Periodic function '{name}' already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding periodic function '{name}'.")
        self._periodic_callbacks[name] = callback

    def _publish_predicates(self):
        """
        Helper function to publish all predicates.
        """
        for name in self._predicates.keys():
            message = Bool()
            value = self.get_predicate(name)
            try:
                message.data = value
            except AssertionError:
                self.get_logger().error(f"Predicate '{name}' has invalid type: expected 'bool', got '{type(value)}'.",
                                        throttle_duration_sec=1.0)
                continue
            if name not in self._predicate_publishers.keys():
                self.get_logger().error(f"No publisher for predicate '{name}' found.", throttle_duration_sec=1.0)
                continue
            self._predicate_publishers[name].publish(message)

    def _publish_outputs(self):
        """
        Helper function to publish all outputs.
        """
        for signal, output_dict in self._outputs.items():
            try:
                message = output_dict["message_type"]()
                output_dict["translator"](message, self.__getattribute__(output_dict["attribute"]))
                output_dict["publisher"].publish(message)
            except Exception as e:
                self.get_logger().error(f"{e}")

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
