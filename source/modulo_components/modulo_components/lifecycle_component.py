from typing import TypeVar

import clproto
from lifecycle_msgs.msg import State
from modulo_components.component_interface import ComponentInterface
from modulo_components.exceptions import AddSignalError
from rclpy.lifecycle import LifecycleNodeMixin, LifecycleState
from rclpy.lifecycle.node import TransitionCallbackReturn

MsgT = TypeVar('MsgT')


class LifecycleComponent(ComponentInterface, LifecycleNodeMixin):
    """
    Class to represent a LifecycleComponent in python, following the same logic pattern
    as the C++ modulo_components::LifecycleComponent class.
    """

    def __init__(self, node_name: str, enable_communication_interface=True, *args, **kwargs):
        """
        Constructs all the necessary attributes and declare all the parameters.

        :param node_name: The name of the node to be passed to the base Node class
        """
        ComponentInterface.__init__(self, node_name, *args, **kwargs)
        LifecycleNodeMixin.__init__(self, enable_communication_interface=enable_communication_interface, *args,
                                    **kwargs)

        self.add_predicate("is_unconfigured", lambda: self.get_state().state_id == State.PRIMARY_STATE_UNCONFIGURED)
        self.add_predicate("is_inactive", lambda: self.get_state().state_id == State.PRIMARY_STATE_INACTIVE)
        self.add_predicate("is_active", lambda: self.get_state().state_id == State.PRIMARY_STATE_ACTIVE)
        self.add_predicate("is_finalized", lambda: self.get_state().state_id == State.PRIMARY_STATE_FINALIZED)

    def get_state(self) -> LifecycleState:
        """
        Get the current state of the component.

        :return: The current state
        """
        return LifecycleState(self._state_machine.current_state[1], self._state_machine.current_state[0])

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'Configuring'.

        on_configure callback is called when the lifecycle component enters the 'Configuring' transition state.
        The component must be in the 'Unconfigured' state.
        Depending on the return value of this function, the component may either transition to the 'Inactive' state
        via the 'configure' transition, stay 'Unconfigured' or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'
        TRANSITION_CALLBACK_FAILURE transitions to 'Unconfigured'
        TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().error(f"on_configure called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().warn(f"Invalid transition 'configure' from state {previous_state.label}")
            return TransitionCallbackReturn.FAILURE
        if not self.__handle_configure():
            self.get_logger().warn("Configuration failed! Reverting to the unconfigured state.")
            if self.__handle_cleanup():
                return TransitionCallbackReturn.FAILURE
            else:
                self.get_logger().error(
                    "Could not revert to the unconfigured state! Entering into the error processing transition state.")
                return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_configure(self) -> bool:
        """
        Handle the configure transition by related transition steps and invoking the user callback.

        :return: True if the transition was successful
        """
        return self.on_configure_callback() and self.__configure_outputs()

    def on_configure_callback(self) -> bool:
        """
        Function called from the configure transition service callback. To be redefined in derived classes.

        :return: True if the component configured successfully
        """
        return True

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'CleaningUp'.

        on_cleanup callback is called when the lifecycle component enters the 'CleaningUp' transition state.
        The component must be in the 'Inactive' state.
        Depending on the return value of this function, the component may either transition to the 'Unconfigured' state
        via the 'cleanup' transition or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Unconfigured'
        TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().debug(f"on_cleanup called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_INACTIVE:
            self.get_logger().warn(f"Invalid transition 'cleanup' from state {previous_state.label}")
        if not self.__handle_cleanup():
            self.get_logger().warn("Cleanup failed! Entering into the error processing transition state.")
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_cleanup(self) -> bool:
        """
        Handle the cleanup transition by related transition steps and invoking the user callback.

        :return: True if the transition was successful
        """
        return self.on_cleanup_callback()

    def on_cleanup_callback(self) -> bool:
        """
        Function called from the cleanup transition service callback. To be redefined in derived classes.

        :return: True if the component was cleaned up successfully
        """
        return True

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'Activating'.

        on_activate callback is called when the lifecycle component enters the 'Activating' transition state.
        The component must be in the 'Inactive' state.
        Depending on the return value of this function, the component may either transition to the 'Active' state
        via the 'activate' transition, stay 'Inactive' or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Active'
        TRANSITION_CALLBACK_FAILURE transitions to 'Inactive'
        TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().debug(f"on_activate called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_INACTIVE:
            self.get_logger().warn(f"Invalid transition 'activate' from state {previous_state.label}")
            return TransitionCallbackReturn.FAILURE
        if not self.__handle_activate():
            self.get_logger().warn("Activation failed! Reverting to the inactive state.")
            # perform deactivation actions to ensure the component is inactive
            if self.__handle_deactivate():
                return TransitionCallbackReturn.FAILURE
            else:
                self.get_logger().error(
                    "Could not revert to the inactive state! Entering into the error processing transition state.")
                return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_activate(self) -> bool:
        """
        Handle the activate transition by related transition steps and invoking the user callback.

        :return: True if the transition was successful
        """
        return self.on_activate_callback()

    def on_activate_callback(self) -> bool:
        """
        Function called from the activate transition service callback. To be redefined in derived classes.

        :return: True if the component was activated successfully
        """
        return True

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'Deactivating'.

        on_deactivate callback is called when the lifecycle component enters the 'Deactivating' transition state.
        The component must be in the 'Active' state.
        Depending on the return value of this function, the component may either transition to the 'Inactive' state
        via the 'deactivate' transition or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'
        TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().debug(f"on_deactivate called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().warn(f"Invalid transition 'deactivate' from state {previous_state.label}")
            return TransitionCallbackReturn.FAILURE
        if not self.__handle_deactivate():
            self.get_logger().warn("Deactivation failed! Reverting to the inactive state.")
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_deactivate(self) -> bool:
        """
        Handle the deactivate transition by related transition steps and invoking the user callback.

        :return: True if the transition was successful
        """
        return self.on_deactivate_callback()

    def on_deactivate_callback(self) -> bool:
        """
        Function called from the deactivate transition service callback. To be redefined in derived classes.

        :return: True if the component was deactivated successfully
        """
        return True

    # TODO on shutdown and on error

    def _step(self):
        """
        Step function that is called periodically and publishes predicates, outputs, evaluates daemon callbacks, and
        calls the on_step function.
        """
        try:
            self._publish_predicates()
            if self.get_state().state_id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().error("stepping")
                self._publish_outputs()
                self._evaluate_periodic_callbacks()
                self.on_step_callback()
        except Exception as e:
            self.get_logger().error(f"Failed to execute step function: {e}", throttle_duration_sec=1.0)

    def on_step_callback(self):
        """
        Steps to execute periodically. To be redefined by derived classes.
        """
        pass

    def __configure_outputs(self) -> bool:
        """
        Configure all outputs.

        :return: True if configuration was successful
        """
        success = True
        for signal_name, output_dict in self._outputs.items():
            try:
                topic_name = self.get_parameter_value(signal_name + "_topic")
                self.get_logger().error(f"Configuring output '{signal_name}' with topic name '{topic_name}'.")
                publisher = self.create_publisher(output_dict["message_type"], topic_name, self._qos)
                self._outputs[signal_name]["publisher"] = publisher
            except Exception as e:
                success = False
                self.get_logger().debug(f"Failed to configure output '{signal_name}': {e}")
        return success

    def add_output(self, signal_name: str, data: str, message_type: MsgT,
                   clproto_message_type=clproto.MessageType.UNKNOWN_MESSAGE, default_topic="", fixed_topic=False):
        """
        Add an output signal of the component.

        :param signal_name: Name of the output signal
        :param data: Name of the attribute to transmit over the channel
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the output signal is fixed
        """
        if self.get_state().state_id not in [State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE]:
            self.get_logger().warn(f"Adding output in state {self.get_state().label} is not allowed.",
                                   throttle_duration_sec=1.0)
            return
        try:
            parsed_signal_name = self._create_output(signal_name, data, message_type, clproto_message_type,
                                                     default_topic, fixed_topic)
            topic_name = self.get_parameter_value(parsed_signal_name + "_topic")
            self.get_logger().debug(f"Adding output '{parsed_signal_name}' with topic name '{topic_name}'.")
        except AddSignalError as e:
            self.get_logger().error(f"Failed to add output '{signal_name}': {e}")
