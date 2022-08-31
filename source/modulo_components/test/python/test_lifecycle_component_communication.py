import pytest
from modulo_components.lifecycle_component import LifecycleComponent


@pytest.mark.parametrize("minimal_cartesian_input", [[LifecycleComponent, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[LifecycleComponent, "/topic"]], indirect=True)
def test_input_output(ros_exec, make_lifecycle_service_client, helpers, random_state, minimal_cartesian_output,
                      minimal_cartesian_input):
    input_client = make_lifecycle_service_client("minimal_cartesian_input")
    output_client = make_lifecycle_service_client("minimal_cartesian_output")
    ros_exec.add_node(input_client)
    ros_exec.add_node(output_client)
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    helpers.configure(ros_exec, input_client)
    helpers.configure(ros_exec, output_client)
    helpers.activate(ros_exec, input_client)
    helpers.activate(ros_exec, output_client)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert minimal_cartesian_input.received_future.result()
    assert random_state.get_name() == minimal_cartesian_input.input.get_name()
    assert random_state.dist(minimal_cartesian_input.input) < 1e-3
