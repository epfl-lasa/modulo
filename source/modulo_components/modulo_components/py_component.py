import rclpy
from lifecycle_msgs.srv import ChangeState

from base_components.component import Component


class PYComponent(Component):
    def __init__(self, node_name:str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

    def on_configure(self) -> bool:
        # configuration steps before activation
        return True

    def on_activate(self) -> bool:
        # activation steps before running
        return True

    def _step(self):
        # do something periodically
        pass


def main():
    rclpy.init()
    node = PYComponent('py_component')
    node.configure(ChangeState.Request(), ChangeState.Response())
    node.activate(ChangeState.Request(), ChangeState.Response())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.deactivate(ChangeState.Request(), ChangeState.Response())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()