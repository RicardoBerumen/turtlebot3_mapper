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

import time
import threading
from functools import partial
from typing import Tuple, List

import numpy as np

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import BoundingBox2DArray
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from turtlebot3_interfaces.action import Mission
from turtlebot3_mapper.utils import occupancy_grid_to_numpy


class Turtlebot3MissionController(Node):

    def parameter_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        for param in parameters:
            if param.name == "free_limit":
                self.free_limit = param.value
                self.get_logger().info("Updated parameter free_limit=%.2f" % self.free_limit)
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def __init__(self, node_name: str = 'turtlebot_mission_controller'):
        super(Turtlebot3MissionController, self).__init__(node_name=node_name)
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "free_limit",
                    30,
                    ParameterDescriptor(description="Max probability to consider cell free"),
                ),
            ],
        )
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.free_limit = int(self.get_parameter("free_limit").value)
        action_callback_group = ReentrantCallbackGroup()
        service_callback_group = ReentrantCallbackGroup()
        subscription_callback_group = ReentrantCallbackGroup()
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._subscriber_scan = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/custom_map",
            callback=self._grid_callback,
            qos_profile=10,
            callback_group=subscription_callback_group,
        )
        self._subscriber_bbox = self.create_subscription(
            msg_type=BoundingBox2DArray,
            topic="/detections",
            callback=self._bbox_callback,
            qos_profile=10,
            callback_group=subscription_callback_group,
        )
        self._action_server = ActionServer(
            node=self,
            action_type=Mission,
            action_name='mission',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_callback_group,
        )
        self.cli = self.create_client(
            SetBool,
            'enable',
            callback_group=service_callback_group,
        )
        self.grid = None
        self.bbox = None
        self.get_logger().info("Init {}".format(node_name))

    def _goal_callback(self, goal_request: Mission.Goal):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse.ACCEPT:
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> Mission.Result:
        frontiers = np.inf
        self._send_request(enable=True)
        while frontiers > goal_handle.request.frontiers:
            if not goal_handle.is_active:
                self._send_request(enable=False)
                self.get_logger().info('Goal aborted')
                response = Mission.Result()
                response.frontiers = int(frontiers)
                response.bbox = self.bbox
                return response
            if goal_handle.is_cancel_requested:
                self._send_request(enable=False)
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                response = Mission.Result()
                response.frontiers = int(frontiers)
                response.bbox = self.bbox
                return response

            frontiers = self.count_frontiers(message=self.grid, free_limit=self.free_limit)
            feedback_msg = Mission.Feedback()
            feedback_msg.frontiers = int(frontiers)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("Frontiers={}".format(frontiers))
            time.sleep(1)
        self._send_request(enable=False)
        goal_handle.succeed()
        self.get_logger().info("Reached goal! :)")
        response = Mission.Result()
        response.frontiers = int(frontiers)
        response.bbox = self.bbox
        return response

    def _grid_callback(self, message: OccupancyGrid):
        self.grid = message
        self.get_logger().info("Received grid")

    def _bbox_callback(self, message: BoundingBox2DArray):
        self.bbox = message
        self.get_logger().info("Received grid")

    def _send_request(self, enable: bool):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetBool.Request()
        request.data = enable
        future = self.cli.call_async(request)
        future.add_done_callback(partial(self._callback_set))

    def _callback_set(self, future):
        try:
            _ = future.result()
        except Exception as ex:
            self.get_logger().warn("Service call failed: %r" % (ex, ))

    def count_frontiers(self, message: OccupancyGrid, free_limit: int = 30):
        count = 0
        array = occupancy_grid_to_numpy(msg=message)
        free = np.argwhere(array < free_limit)
        for n in range(free.shape[0]):
            item = free[n, :]
            neighboors = self.get_neighboors(x=item[0], y=item[1], shape=array.shape)
            for i in range(neighboors.shape[0]):
                coord = neighboors[i, :]
                value = array[coord[0], coord[1]]
                if value == 50:
                    count += 1
        return count

    @staticmethod
    def get_neighboors(x: int, y: int, shape: Tuple[int, int]):
        neighboors = []
        if (x + 1) < shape[1]:
            neighboors.append([x + 1, y])
        if (x - 1) >= 0:
            neighboors.append([x - 1, y])
        if (y + 1) < shape[0]:
            neighboors.append([x, y + 1])
        if (y - 1) >= 0:
            neighboors.append([x, y - 1])
        return np.array(neighboors)
