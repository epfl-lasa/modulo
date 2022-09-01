import pytest
from modulo_components.component import Component


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[Component, "/topic"]], indirect=True)
def test_input_output(ros_exec, random_state, minimal_cartesian_output, minimal_cartesian_input):
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    ros_exec.remove_node(minimal_cartesian_input)
    ros_exec.remove_node(minimal_cartesian_output)
    assert minimal_cartesian_input.received_future.result()
    assert random_state.get_name() == minimal_cartesian_input.input.get_name()
    assert random_state.dist(minimal_cartesian_input.input) < 1e-3


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
def test_input_output_invalid(ros_exec, make_minimal_invalid_encoded_state_publisher, minimal_cartesian_input):
    invalid_publisher = make_minimal_invalid_encoded_state_publisher("/topic")
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(invalid_publisher)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.result()
