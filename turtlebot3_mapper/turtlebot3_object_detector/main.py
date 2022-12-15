import rclpy
from rclpy.executors import ExternalShutdownException

from turtlebot3_mapper.turtlebot3_object_detector.turtlebot3_object_detector \
    import Turtlebot3ObjectDetector


def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    node = Turtlebot3ObjectDetector(node_name="turtlebot3_object_detector")
    try:
        rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
