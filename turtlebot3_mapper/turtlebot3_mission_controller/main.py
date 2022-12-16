import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from turtlebot3_mapper.turtlebot3_mission_controller.turtlebot3_mission_controller \
    import Turtlebot3MissionController


def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    executor = MultiThreadedExecutor()
    node = Turtlebot3MissionController()
    try:
        rclpy.spin(node=node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
