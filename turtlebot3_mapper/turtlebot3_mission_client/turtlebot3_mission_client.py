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
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from turtlebot3_interfaces.action import Mission


class Turtlebot3MissionClient(Node):

    def __init__(self,
                 resolution: float,
                 width: int,
                 height: int,
                 node_name: str = "turtlebot3_mission_client"):
        super().__init__(node_name=node_name)
        self._action_client = ActionClient(
            self,
            Mission,
            'mission',
        )
        self.resolution = resolution
        self.width = width
        self.height = height
        self.world = (-self.width // 2, -self.height // 2)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.frontiers))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        with open("results.txt", "w") as file:
            file.write("x;y;width;height\n")
            for bbox in result.bbox.boxes:
                x = round((bbox.center.position.x * self.resolution) + self.world[0], 2)
                y = round((bbox.center.position.y * self.resolution) + self.world[0], 2)
                size_x = round(bbox.size_x * self.resolution, 2)
                size_y = round(bbox.size_y * self.resolution, 2)
                file.write(f"{x};{y};{size_x};{size_y}\n")

        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback, frontiers={0}'.format(
            feedback.feedback.frontiers))

    def cancel(self):
        self.get_logger().info('Canceling goal')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def send_goal(self, frontiers: int):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Mission.Goal()
        goal_msg.frontiers = frontiers
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
