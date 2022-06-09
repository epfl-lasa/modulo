import sys
from typing import Union, TypeVar, Callable, List

import rclpy
import state_representation as sr
from modulo_components.exceptions.component_exceptions import ComponentParameterError
from modulo_new_core.translators.parameter_translators import write_parameter, read_parameter_const
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Bool

T = TypeVar('T')


class ComponentInterface(Node):
    """
    Abstract class to represent a Component in python, following the same logic pattern
    as the c++ modulo_component::ComponentInterface class.
    ...
    Attributes:
        predicates (dict(Parameters(bool))): map of predicates added to the Component.
        predicate_publishers (dict(rclpy.Publisher(Bool))): map of publishers associated to each predicate.
        parameter_dict: dict of parameters
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
        self._parameter_dict = {}
        self._predicates = {}
        self._predicate_publishers = {}
        # TODO add_XXX
        self.__tf_buffer = None
        self.__tf_listener = None
        self.__tf_broadcaster = None

        self.add_on_set_parameters_callback(self.__on_set_parameters_callback)
        self.add_parameter(sr.Parameter("period", 0.1, sr.ParameterType.DOUBLE),
                           "Period (in s) between step function calls.")

        self.create_timer(self.get_parameter_value("period"), self.__step)

    def __step(self) -> None:
        """
        Step function that is called periodically.
        """
        for predicate_name in self._predicates.keys():
            msg = Bool()
            msg.data = self.get_predicate(predicate_name)
            if predicate_name not in self._predicate_publishers.keys():
                self.get_logger().error(f"No publisher for predicate {predicate_name} found.",
                                        throttle_duration_sec=1.0)
                return
            self._predicate_publishers[predicate_name].publish(msg)

    def __generate_predicate_topic(self, predicate_name: str) -> str:
        """
        Generate the predicate topic name from the name of the predicate.

        :param predicate_name: The predicate name
        :return: The predicate topic as /predicates/component_name/predicate_name
        """
        return f"/predicates/{self.get_name()}/{predicate_name}"

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

    def add_predicate(self, predicate_name: str, predicate_value: Union[bool, Callable]):
        """
        Add a predicate to the map of predicates.

        :param predicate_name: The name of the associated predicate
        :param predicate_value: The value of the predicate as a bool or a callable function
        """
        if not predicate_name:
            self.get_logger().error("Failed to add predicate: Provide a non empty string as a name.")
        if predicate_name in self._predicates.keys():
            self.get_logger().debug(f"Predicate {predicate_name} already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding predicate '{predicate_name}'.")
            self._predicate_publishers[predicate_name] = self.create_publisher(Bool,
                                                                               self.__generate_predicate_topic(
                                                                                   predicate_name), 10)
        self._predicates[predicate_name] = predicate_value

    def get_predicate(self, predicate_name: str) -> bool:
        """
        Get the value of the predicate given as parameter. If the predicate is not found or the callable function fails,
        this method returns False.

        :param predicate_name: The name of the predicate to retrieve from the map of predicates
        :return: The value of the predicate as a boolean
        """
        if predicate_name not in self._predicates.keys():
            self.get_logger().error(f"Predicate {predicate_name} does not exist, returning false.",
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

        :param predicate_name: The name of the predicate to retrieve from the map of predicates
        :param predicate_value: The new value of the predicate as a bool or a callable function
        """
        if predicate_name not in self._predicates.keys():
            self.get_logger().error(
                f"Cannot set predicate {predicate_name} with a new value because it does not exist.",
                throttle_duration_sec=1.0)
            return
        self._predicates[predicate_name] = predicate_value
