import pytest
import state_representation as sr
from modulo_components.component import Component
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String


@pytest.fixture()
def component(ros_context):
    yield Component('component')


def test_add_output(component):
    component.add_output("_tEsT_#1@3", "test", Bool)
    assert "test_13" in component._outputs.keys()
    assert component.get_parameter_value("test_13_topic") == "~/test_13"

    component.add_output("_tEsT_#1@5", "test", Bool, default_topic="/topic")
    assert "test_15" in component._outputs.keys()
    assert component.get_parameter_value("test_15_topic") == "/topic"

    component.add_output("test_13", "test", String)
    assert component._outputs["test_13"]["message_type"] == Bool


def test_empty_parameter(component):
    component.add_parameter(sr.Parameter("test", sr.ParameterType.DOUBLE_ARRAY), "description")
    assert component.get_parameter("test").get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY
    assert Node.get_parameter(component, "test").type_ == Parameter.Type.NOT_SET

    component.set_parameter_value("test", [1.0, 2.0], sr.ParameterType.DOUBLE_ARRAY)
    assert Node.get_parameter(component, "test").type_ == Parameter.Type.DOUBLE_ARRAY

    component.set_parameter_value("test", "test", sr.ParameterType.STRING)
    assert component.get_parameter("test").get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY
    assert Node.get_parameter(component, "test").type_ == Parameter.Type.DOUBLE_ARRAY

    Node.set_parameters(component, [Parameter("test", value="test")])
    assert Node.get_parameter(component, "test").type_ == Parameter.Type.DOUBLE_ARRAY
