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

import random
from enum import Enum
from typing import List

import numpy as np

from rclpy.node import Node
from rclpy.parameter import Parameter

from std_srvs.srv import SetBool

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.qos import QoSPresetProfiles

from tf2_ros.buffer import Buffer
from tf2_ros import LookupException, TransformException
from tf2_ros.transform_listener import TransformListener

from turtlebot3_mapper.utils import (
    euler_from_quaternion,
    laser_scan_to_polar,
)


class State(Enum):
    FORWARD = 1
    ROTATE = 2
    STOP = 3
    UNKOWN = 4


class Turtlebot3Explorer(Node):

    def parameter_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        for param in parameters:
            if param.name == "view_angle":
                self.view_angle = param.value
                self.get_logger().info("Updated parameter view_angle=%.2f" % self.view_angle)
            elif param.name == "min_rand":
                self.min_rand = param.value
                self.get_logger().info("Updated parameter min_rand=%.2f" % self.min_rand)
            elif param.name == "max_rand":
                self.max_rand = param.value
                self.get_logger().info("Updated parameter max_rand=%.2f" % self.max_rand)
            elif param.name == "safety_distance":
                self.safety_distance = param.value
                self.get_logger().info("Updated parameter safety_distance=%.2f" %
                                       self.safety_distance)
            elif param.name == "linear_speed":
                self.linear_speed = param.value
                self.get_logger().info("Updated parameter linear_speed=%.2f" % self.linear_speed)
            elif param.name == "angular_speed":
                self.angular_speed = param.value
                self.get_logger().info("Updated parameter angular_speed=%.2f" % self.angular_speed)
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def __init__(self, node_name: str = 'turtlebot_explorer'):
        super(Turtlebot3Explorer, self).__init__(node_name=node_name)
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "view_angle", #Ángulo de visión del robot para detectar obstáculos.
                    30.0,
                    ParameterDescriptor(description="View angle"),
                ),
                (
                    "min_rand", #Define el tiempo aleatorio minimo durante el cual el robot cambia de dirección mientras explora
                    ParameterDescriptor(description="Min range random limit"),
                ),
                (
                    "max_rand", #Define el tiempo aleatorio maximo durante el cual el robot cambia de dirección mientras explora
                    10,
                    ParameterDescriptor(description="Max range random limit"),
                ),
                (
                    "safety_distance", #Mínima distancia segura ante obstáculos.
                    0.6,
                    ParameterDescriptor(description="Safety distance"),
                ),
                (
                    "linear_speed", #Velocidad de movimiento 
                    0.15,
                    ParameterDescriptor(description="Linear speed"),
                ),
                (
                    "angular_speed", #Rotación del robot
                    0.15,
                    ParameterDescriptor(description="Angular speed"),
                ),
                (
                    "update_rate", #Frecuencia de actualización del control
                    0.5,
                    ParameterDescriptor(description="Update rate"),
                ),
            ],
        )
        self.view_angle = float(self.get_parameter("view_angle").value)
        self.min_rand = float(self.get_parameter("min_rand").value)
        self.max_rand = float(self.get_parameter("max_rand").value)
        self.safety_distance = float(self.get_parameter("safety_distance").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self._subscriber_scan = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self._scan_callback,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._publisher = self.create_publisher(
            msg_type=Twist,
            topic="/cmd_vel",
            qos_profile=10,
        )
        self.update_timer = self.create_timer(
            timer_period_sec=(1 / self.get_parameter("update_rate").value),
            callback=self._update_callback,
        )
        self.srv = self.create_service(
            SetBool,
            'enable',
            self._enable_callback,
        )
        self._scan_init = False

        self.count = 0
        self.multiplier = 1.0
        self.limit = random.randint(self.min_rand, self.max_rand)

        self.state = State.UNKOWN
        self.enable = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info("Init {}".format(node_name))

    def _enable_callback(self, request, response):
        self.enable = request.data
        response.success = True
        response.message = "Configured"
        return response

    def _scan_callback(self, message: LaserScan):
        
        self._scan = message
        self._scan_init = True

        max_range = message.range_max
        ranges = np.array(message.ranges)
        ranges[np.isnan(ranges)] = max_range
        message.ranges = ranges.tolist()  # Asignar los valores corregidos al mensaje

    def _update_callback(self):
        if self._scan_init:
            self.get_logger().info("updating...")
            self.detect_obstacle()

    def set_robot(self, state: State):
        twist = Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        if state == State.STOP:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._publisher.publish(twist)
        elif state == State.FORWARD:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self._publisher.publish(twist)
        elif state == State.ROTATE:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed * self.multiplier
            self.count += 1
            if self.count >= self.limit:
                self.limit = random.randint(self.min_rand, self.max_rand)
                self.count = 0
                self.multiplier *= -1.0
            self._publisher.publish(twist)

    def detect_obstacle(self):
        scan = self._scan
        try:
            trans = self._tf_buffer.lookup_transform(scan.header.frame_id, "base_link",
                                                     scan.header.stamp)
        except (TransformException, LookupException) as ex:
            self.get_logger().warn(f'Could not transform "odom" to {scan.header.frame_id}: {ex}')
            return

        _, _, theta = euler_from_quaternion(trans.transform.rotation)
        min_angle = (-self.view_angle + theta) * (np.pi / 180) + (2 * np.pi)
        max_angle = (self.view_angle + theta) * (np.pi / 180)

        polar = laser_scan_to_polar(message=scan)
        mask = (polar[:, 1] >= (min_angle)) | (polar[:, 1] <= max_angle)
        distance_range = polar[mask, :]

        obstacle_distance = np.min(distance_range[:, 0])
        self.get_logger().info("Closest obstacle={}".format(obstacle_distance))

        if not self.enable:
            if self.state != State.STOP:
                self.state = State.STOP
                self.set_robot(state=self.state)
        else:
            if self.state == State.UNKOWN or self.state == State.STOP:
                if obstacle_distance < self.safety_distance:
                    self.state = State.ROTATE
                    self.set_robot(state=self.state)
                else:
                    self.state = State.FORWARD
                    self.set_robot(state=self.state)
            elif self.state == State.FORWARD:
                if obstacle_distance < self.safety_distance:
                    self.state = State.ROTATE
                    self.set_robot(state=self.state)
                else:
                    self.state = State.FORWARD
                    self.set_robot(state=self.state)
            elif self.state == State.ROTATE:
                if obstacle_distance > self.safety_distance:
                    self.state = State.FORWARD
                    self.set_robot(state=self.state)
