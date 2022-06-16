from threading import Thread

from modulo_components.component_interface import ComponentInterface


class Component(ComponentInterface):
    """
    Class to represent a Component in python, following the same logic pattern
    as the c++ modulo_component::Component class.
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
        Run the execution function in a try catch block and set
        the predicates according to the outcome of the execution.
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
