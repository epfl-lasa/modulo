from typing import TypeVar

import clproto
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State
from modulo_components.component_interface import ComponentInterface
from modulo_components.exceptions.component_exceptions import AddSignalError

MsgT = TypeVar('MsgT')


class LifecycleComponent(ComponentInterface):
    """
    Class to represent a LifecycleComponent in python, following the same logic pattern
    as the c++ modulo_component::LifecycleComponent class.
    ...
    Attributes:
    # TODO
    """

    def __init__(self, node_name: str, *kargs, **kwargs):
        """
        Constructs all the necessary attributes and declare all the parameters.
            Parameters:
                node_name (str): name of the node to be passed to the base Node class
        """
        super().__init__(node_name, *kargs, **kwargs)
        self.__state = State.PRIMARY_STATE_UNCONFIGURED

        # add the service to mimic the lifecycle paradigm
        self._change_state_srv = self.create_service(ChangeState, '~/change_state', self.__change_state)

        self.add_predicate("is_unconfigured", lambda: self.__state == State.PRIMARY_STATE_UNCONFIGURED)
        self.add_predicate("is_inactive", lambda: self.__state == State.PRIMARY_STATE_INACTIVE)
        self.add_predicate("is_active", lambda: self.__state == State.PRIMARY_STATE_ACTIVE)
        self.add_predicate("is_finalized", lambda: self.__state == State.PRIMARY_STATE_FINALIZED)

    def __change_state(self, request: ChangeState.Request, response: ChangeState.Response) -> ChangeState.Response:
        """
        Change state service callback which calls the requested transition method.

        :param request: The lifecycle_msgs.msg.ChangeState.Request message
        :param response: The lifecycle_msgs.msg.ChangeState.Response message
        :return: The response message
        """
        self.get_logger().debug(f'Change state service called with request {request}')
        if request.transition.id == Transition.TRANSITION_CONFIGURE:
            response = self._configure(request, response)
        elif request.transition.id == Transition.TRANSITION_ACTIVATE:
            response = self._activate(request, response)
        elif request.transition.id == Transition.TRANSITION_DEACTIVATE:
            response = self._deactivate(request, response)

        else:
            self.get_logger().error(f'Unsupported state transition! {request}')
            response.success = False
        return response

    def _configure(self, request: ChangeState.Request, response: ChangeState.Response) -> ChangeState.Response:
        """
        Configure service callback. Simply call the on_configure function.

        :param request: request to change the state to non active
        :param response: the response object from the service call
        :return: true if the node configured properly
        """
        if not self.__state == State.PRIMARY_STATE_UNCONFIGURED:
            response.success = False
            return response

        response.success = self._configure_outputs() and self.on_configure()
        if response.success:
            self.__state = State.PRIMARY_STATE_INACTIVE
        return response

    def on_configure(self) -> bool:
        """
        Function called from the configure service callback. To be redefined in derived classes.

        :return: true if the node configured properly
        """
        return True

    def _configure_outputs(self) -> bool:
        success = True
        for signal_name, output_dict in self._outputs.items():
            try:
                topic_name = self.get_parameter_value(signal_name + "_topic")
                self.get_logger().debug(f"Configuring output '{signal_name}' with topic name '{topic_name}'.")
                publisher = self.create_publisher(output_dict["message_type"], topic_name, self._qos)
                self._outputs[signal_name]["publisher"] = publisher
            except Exception as e:
                success = False
                self.get_logger().debug(f"Failed to configure output '{signal_name}': {e}")
        return success

    def _cleanup(self, request: ChangeState.Request, response: ChangeState.Response) -> ChangeState.Response:
        """
        Cleanup service callback. Simply call the on_cleanup function.

        :param request: request to change the state to unconfigured
        :param response: the response object from the service call
        :return: true if the node was clean up properly
        """
        if self.__state != State.PRIMARY_STATE_INACTIVE:
            response.success = False
            return response

        response.success = self.on_cleanup()
        if response.success:
            self.__state = State.PRIMARY_STATE_UNCONFIGURED
        return response

    def on_cleanup(self) -> bool:
        """
        Function called from the cleanup service callback. To be redefined in derived classes.

        :return: true if the node was cleaned up properly
        """
        return True

    def _activate(self, request: ChangeState.Request, response: ChangeState.Response) -> ChangeState.Response:
        """
        Activate service callback. Set the node as active and call the on_activate function.

        :param request: request to change the state to active state
        :param response: the response object from the service call
        :return: true if the node is activated properly
        """
        if self.__state != State.PRIMARY_STATE_INACTIVE:
            response.success = False
            return response

        response.success = self.on_activate()
        if response.success:
            self.__state = State.PRIMARY_STATE_ACTIVE
        return response

    def on_activate(self) -> bool:
        """
        Function called from the activate service callback. To be redefined in derived classes.

        :return: true if the node is activated properly
        """
        return True

    def _deactivate(self, request: ChangeState.Request, response: ChangeState.Response) -> ChangeState.Response:
        """
        Deactivate service callback. Set the node as not active and call the on_deactivate function.

        :param request: request to change the state to active state
        :param response: the response object from the service call
        :return: true if the node is deactivated properly
        """
        if self.__state != State.PRIMARY_STATE_ACTIVE:
            response.success = False
            return response

        response.success = self.on_deactivate()
        if response.success:
            self.__state = State.PRIMARY_STATE_INACTIVE
        return response

    def on_deactivate(self) -> bool:
        """
        Function called from the deactivate service callback. To be redefined in derived classes.

        :return: true if the node is deactivated properly
        """
        return True

    # TODO on shutdown and on error

    def _step(self):
        try:
            self._publish_predicates()
            if self.__state == State.PRIMARY_STATE_ACTIVE:
                self._publish_outputs()
                self._evaluate_periodic_callbacks()
                self.on_step()
        except Exception as e:
            self.get_logger().error(f"Failed to execute step function: {e}", throttle_duration_sec=1.0)

    def add_output(self, signal_name: str, data: str, message_type: MsgT,
                   clproto_message_type=clproto.MessageType.UNKNOWN_MESSAGE, fixed_topic=False, default_topic=""):
        """
        Add an output signal of the component.

        :param signal_name: Name of the output signal
        :param data: Name of the attribute to transmit over the channel
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :param default_topic: If set, the default value for the topic name to use
        """
        try:
            parsed_signal_name = self._create_output(signal_name, data, message_type, clproto_message_type, fixed_topic,
                                                     default_topic)
            self.get_logger().debug(f"Adding output '{parsed_signal_name}.")
        except AddSignalError as e:
            self.get_logger().error(f"Failed to add output '{signal_name}': {e}")

    def on_step(self):
        pass
