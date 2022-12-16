import rclpy
import argparse

from rclpy.executors import ExternalShutdownException

from turtlebot3_mapper.turtlebot3_mission_client.turtlebot3_mission_client \
    import Turtlebot3MissionClient


def main(*args, **kwargs):
    parser = argparse.ArgumentParser(description='Explore enviromment')
    parser.add_argument('-f',
                        action='store',
                        dest='frontiers',
                        type=int,
                        default=10,
                        required=True,
                        help='Minimal number of frontiers to reach')
    parser.add_argument('--width',
                        action='store',
                        dest='width',
                        type=int,
                        default=8,
                        required=False,
                        help='Width of map')
    parser.add_argument('--height',
                        action='store',
                        dest='height',
                        type=int,
                        default=8,
                        required=False,
                        help='Height of map')
    parser.add_argument('--resolution',
                        action='store',
                        dest='resolution',
                        type=float,
                        default=0.03,
                        required=False,
                        help='Resolution of map')
    args = parser.parse_args()
    rclpy.init(args=None)
    node = Turtlebot3MissionClient(
        resolution=args.resolution,
        width=args.width,
        height=args.height,
        node_name="turtlebot3_mission_client",
    )
    node.send_goal(frontiers=args.frontiers)
    try:
        rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.cancel()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
