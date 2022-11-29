import rclpy
import threading
import numpy as np

from enum import Enum
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from turtlebot3_mapper.utils import euler_from_quaternion


class State(Enum):
    FORWARD = 1
    ROTATE = 2
    UNKOWN = 3

class TurtlebotExplorer(Node):


    def __init__(self, node_name: str = 'turtlebot_explorer'):
        super(TurtlebotExplorer,
              self).__init__(node_name=node_name)
        self._subscriber_scan = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self._scan_callback,
            qos_profile=10,
        )
        self._subscriber_odom = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self._odom_callback,
            qos_profile=10,
        )
        self._publisher = self.create_publisher(
            msg_type=Twist,
            topic="/cmd_vel",
            qos_profile=10,
        )
        self.update_timer = self.create_timer(
            timer_period_sec=1,
            callback=self.update_callback,
        )
        self.view_angle = 90
        self._odom_init = False
        self._scan_init = False
        self.count = 0
        self.multiplier = 1.0
        self._update = threading.Lock()

        self.state = State.UNKOWN
        self.get_logger().info("Init turtlebot_explorer")

    def _scan_callback(self, message: LaserScan):
        if not self._update.locked():
            self._scan = message
        self._scan_init = True

    def _odom_callback(self, message: Odometry):
        self._odom = message
        if not self._update.locked():
            self._odom = message
        self._odom_init = True

    def update_callback(self):
        if self._scan_init and self._odom_init:
            if self._update.locked():
                return
            else:
                self._update.acquire()
                self.get_logger().info("updating...")
                self.detect_obstacle()
                self._update.release()

    def detect_obstacle(self):
        # unit: m
        safety_distance = 0.5

        _, _, theta = euler_from_quaternion(self._odom.pose.pose.orientation)

        min_ang = (-140) * (np.pi/180)
        max_ang = (140) * (np.pi/180)

        increm = self._scan.angle_increment
        size = len(self._scan.ranges)

        start = int(size/2 + min_ang/increm)
        end = int(size/2 + max_ang/increm)
        
        # scan_range = [self.scan.ranges[i] for i in range(90, 270)]
        scan_range = [self._scan.ranges[i] for i in range(end, len(self._scan.ranges))]
        for i in range(0, start):
            scan_range.append(self._scan.ranges[i])

        obstacle_distance = min(scan_range)

        if self.state == State.UNKOWN:
            if obstacle_distance < safety_distance:
                if self.state != State.ROTATE:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.25*self.multiplier
                    if self.count >= 3:
                        self.count = 0
                        self.multiplier *= -1.0
                    self.count += 1
                    self.state = State.ROTATE
                    self._publisher.publish(twist)
            else:
                self.state = State.FORWARD
                twist = Twist()
                twist.linear.x = 0.25
                twist.angular.z = 0.0
                self._publisher.publish(twist)

        elif self.state == State.FORWARD:
            if obstacle_distance < safety_distance:
                if self.state != State.ROTATE:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.25*self.multiplier
                    if self.count >= 3:
                        self.count = 0
                        self.multiplier *= -1.0
                    self.count += 1
                    self.state = State.ROTATE
                    self._publisher.publish(twist)
            else:
                twist = Twist()
                twist.linear.x = 0.25
                twist.angular.z = 0.0
                self.state = State.FORWARD
                self._publisher.publish(twist)

        else:
            if obstacle_distance < safety_distance:
                pass
            else:
                twist = Twist()
                twist.linear.x = 0.25
                twist.angular.z = 0.0
                self.state = State.FORWARD
                self._publisher.publish(twist)


def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    node = TurtlebotExplorer()
    try:
        rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
