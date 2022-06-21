from threading import Thread
from typing import TypeVar

import clproto
from modulo_components.component_interface import ComponentInterface

DataT = TypeVar('DataT')
MsgT = TypeVar('MsgT')


class Component(ComponentInterface):
    """
    Class to represent a Component in python, following the same logic pattern as the c++ modulo_component::Component
    class.
    ...
    Attributes:
        started (bool): flag that indicates if execution has started or not
        run_thread (Thread): the execution thread
    """

    def __init__(self, node_name: str, start_thread=True, *kargs, **kwargs):
        """
        Constructs all the necessary attributes and declare all the parameters.
            Parameters:
                node_name (str): name of the node to be passed to the base Node class
                start_thread (bool): start the execution thread at construction
        """
        super().__init__(node_name, *kargs, **kwargs)
        self.__started = False
        self.__run_thread = None
        self.add_predicate("is_finished", False)

        if start_thread:
            self.start_thread()

    def _step(self):
        """
        Step function that is called periodically.
        """
        try:
            # TODO catch here or in helpers...? (or re raise with ComponentError)
            self._publish_predicates()
            self._publish_outputs()
            self._evaluate_periodic_callbacks()
        except Exception as e:
            self.get_logger().error(f"Failed to execute step function: {e}", throttle_duration_sec=1.0)

    def start_thread(self):
        """
        Start the execution thread.
        """
        if self.__started:
            self.get_logger().error(f"Run thread for component {self.get_name()} has already been started",
                                    throttle_duration_sec=1.0)
            return
        self.__started = True
        self.__run_thread = Thread(target=self.__run)
        self.__run_thread.start()

    def __run(self):
        """
        Run the execution function in a try catch block and set the predicates according to the outcome of the
        execution.
        """
        try:
            if not self.execute():
                self.raise_error()
                return
        except Exception as e:
            self.get_logger().error(f"Failed to run component {self.get_name()}: {e}", throttle_duration_sec=1.0)
            self.raise_error()
            return
        self.set_predicate("is_finished", True)

    def execute(self):
        """
        Execute the component logic. To be redefined in the derived classes.

        :return: True, if the execution was successful, false otherwise
        """
        return True

    def _raise_error(self):
        """
        Put the component in error state by setting the 'in_error_state'
        predicate to true.
        """
        self.set_predicate("in_error_state", True)

    def add_output(self, signal_name: str, data: DataT, message_type: MsgT,
                   clproto_message_type=clproto.MessageType.UNKNOWN_MESSAGE, fixed_topic=False, default_topic=""):
        """
        Add and configure an output signal of the component.

        :param signal_name: Name of the output signal
        :param data: Data to transmit on the output signal
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :param default_topic: If set, the default value for the topic name to use
        """
        try:
            parsed_signal_name = self._create_output(signal_name, data, message_type, clproto_message_type, fixed_topic,
                                                     default_topic)
            topic_name = self.get_parameter_value(parsed_signal_name + "_topic")
            self.get_logger().debug(f"Adding output '{parsed_signal_name}' with topic name '{topic_name}'.")
            publisher = self.create_publisher(message_type, topic_name, self._qos)
            self._outputs[parsed_signal_name]["publisher"] = publisher
        except Exception as e:
            self.get_logger().error(f"Failed to add output '{signal_name}: {e}")
