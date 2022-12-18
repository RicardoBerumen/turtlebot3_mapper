# Copyright 2022 Luiz Carlos Cosmi Filho and others.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
