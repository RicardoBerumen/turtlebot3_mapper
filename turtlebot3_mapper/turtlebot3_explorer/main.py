import rclpy
from rclpy.executors import ExternalShutdownException

from turtlebot3_mapper.turtlebot3_explorer.turtlebot3_explorer \
    import Turtlebot3Explorer


def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    node = Turtlebot3Explorer()
    try:
        rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
