import pytest
import rclpy
import state_representation as sr
from modulo_components.component_interface import ComponentInterface
from modulo_components.exceptions.component_exceptions import ComponentParameterError
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from rclpy.parameter import Parameter


class EmtpyParameterInterface(ComponentInterface):
    def __init__(self, node_name, allow_empty=True, add_parameter=True, *kargs, **kwargs):
        super().__init__(node_name, *kargs, **kwargs)
        self._allow_empty = allow_empty
        if add_parameter:
            self.add_parameter(sr.Parameter("name", sr.ParameterType.STRING), "Test parameter")

    def get_ros_parameter(self, name: str) -> rclpy.Parameter:
        return rclpy.node.Node.get_parameter(self, name)

    def set_ros_parameter(self, param: rclpy.Parameter) -> SetParametersResult:
        return rclpy.node.Node.set_parameters(self, [param])[0]

    def _validate_parameter(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() == "name":
            if parameter.is_empty():
                return self._allow_empty
            elif not parameter.get_value():
                self.get_logger().error("Provide a non empty value for parameter 'name'")
                return False
        return True


@pytest.fixture()
def component_test(ros_context):
    yield EmtpyParameterInterface("component")


def test_not_allow_empty_on_construction(ros_context):
    # Construction with empty parameter should raise if the empty parameter is not allowed
    with pytest.raises(ComponentParameterError):
        EmtpyParameterInterface("component", False)


def test_not_allow_empty(ros_context):
    component = EmtpyParameterInterface("component", allow_empty=False, add_parameter=False)
    with pytest.raises(ComponentParameterError):
        component.add_parameter(sr.Parameter("name", sr.ParameterType.STRING), "Test parameter")
    with pytest.raises(ComponentParameterError):
        component.get_parameter("name")
    with pytest.raises(ParameterNotDeclaredException):
        assert Node.get_parameter(component, "name").type_ == Parameter.Type.NOT_SET

    component = EmtpyParameterInterface("component", allow_empty=True, add_parameter=False)
    component.add_parameter(sr.Parameter("name", sr.ParameterType.STRING), "Test parameter")
    assert component.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component.get_parameter("name").is_empty()
    assert Node.get_parameter(component, "name").type_ == Parameter.Type.NOT_SET


def test_validate_empty_parameter(component_test):
    # component_test comes with empty parameter 'name'
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter("name").is_empty()
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.NOT_SET

    # Trying to overwrite a parameter is not possible
    component_test.add_parameter(sr.Parameter("name", sr.ParameterType.BOOL), "Test parameter")
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter("name").is_empty()
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.NOT_SET

    # Set parameter with empty parameter isn't possible because there is no method for that
    # component_test.set_parameter(sr.Parameter("name", sr.ParameterType.STRING))

    # Set parameter value from rclpy interface should update ROS parameter type
    Node.set_parameters(component_test, [Parameter("name", value="test")])
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "test"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "test"

    # Set parameter value from component interface
    component_test.set_parameter_value("name", "again", sr.ParameterType.STRING)
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "again"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "again"

    # Setting it with empty value should be rejected in parameter evaluation
    component_test.set_parameter_value("name", "", sr.ParameterType.STRING)
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "again"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "again"

    # Setting it with empty value should be rejected in parameter evaluation
    component_test.set_parameter_value("name", "", sr.ParameterType.STRING)
    Node.set_parameters(component_test, [Parameter("name", type_=Parameter.Type.STRING, value="")])
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "again"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "again"

    # TODO
    # Setting a parameter with type NOT_SET undeclares that parameter
