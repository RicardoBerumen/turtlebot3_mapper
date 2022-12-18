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
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from turtlebot3_mapper.turtlebot3_occupancy_grid.turtlebot3_occupancy_grid \
    import Turtlebot3OccupancyGrid


def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    node = Turtlebot3OccupancyGrid(node_name="turtlebot3_occupancy_grid")
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node=node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
