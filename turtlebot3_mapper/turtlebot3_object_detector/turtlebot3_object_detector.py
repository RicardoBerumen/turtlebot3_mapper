from typing import List

import cv2
import numpy as np
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from turtlebot3_mapper.utils import occupancy_grid_to_numpy


class Turtlebot3ObjectDetector(Node):

    def parameter_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        for param in parameters:
            if param.name == "threshold":
                self.threshold = param.value
                self.get_logger().info("Updated parameter threshold=%.2f" % self.threshold)
            elif param.name == "connectivity":
                self.connectivity = param.value
                self.get_logger().info("Updated parameter connectivity=%.2f" % self.connectivity)
            elif param.name == "min_area":
                self.min_area = param.value
                self.get_logger().info("Updated parameter min_area=%.2f" % self.min_area)
            elif param.name == "max_area":
                self.max_area = param.value
                self.get_logger().info("Updated parameter max_area=%.2f" % self.max_area)
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def __init__(self, node_name: str = 'turtlebot3_object_detector'):

        super(Turtlebot3ObjectDetector, self).__init__(node_name=node_name)
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "threshold",
                    150,
                    ParameterDescriptor(description="Threshold to binary image"),
                ),
                (
                    "connectivity",
                    4,
                    ParameterDescriptor(description="4-way or 8-way connectivity"),
                ),
                (
                    "min_area",
                    30,
                    ParameterDescriptor(description="Min area of each connected component"),
                ),
                (
                    "max_area",
                    100,
                    ParameterDescriptor(description="Max area of each connected component"),
                ),
            ],
        )
        self.threshold = float(self.get_parameter("threshold").value)
        self.connectivity = float(self.get_parameter("connectivity").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.max_area = float(self.get_parameter("max_area").value)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self._map_subscriber = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/custom_map",
            callback=self._map_callback,
            qos_profile=10,
        )
        self._detections_publisher = self.create_publisher(
            msg_type=BoundingBox2DArray,
            topic="/detections",
            qos_profile=10,
        )
        self._result_publisher = self.create_publisher(
            msg_type=Image,
            topic="/rendered",
            qos_profile=10,
        )
        self._bridge = CvBridge()
        self.get_logger().info(f"Init {node_name}")

    def _map_callback(self, message: OccupancyGrid):
        # get occupancy grid as a numpy array int8
        array = occupancy_grid_to_numpy(message)
        # convert int8 array to float32
        input = array.astype("float32")
        # remap to 0-255 interval
        input = 255 * (input - np.min(input)) / (np.max(input) - np.min(input))
        # if it is unkown or free, mark as 0
        input[input <= self.threshold] = 0
        # if it is occupied, mark as 255
        input[input > self.threshold] = 255
        # get array as uint8
        input = input.astype('uint8')
        # get component connected information
        analysis = cv2.connectedComponentsWithStats(input, self.connectivity, cv2.CV_32S)
        # unpack information
        (totalLabels, label_ids, values, centroid) = analysis
        # get RGB image to show
        output = cv2.cvtColor(input, cv2.COLOR_GRAY2BGR)
        # Loop through each component
        detections = BoundingBox2DArray()
        detections.header = message.header
        for i in range(1, totalLabels):
            x = values[i, cv2.CC_STAT_LEFT]
            y = values[i, cv2.CC_STAT_TOP]
            w = values[i, cv2.CC_STAT_WIDTH]
            h = values[i, cv2.CC_STAT_HEIGHT]
            area = values[i, cv2.CC_STAT_AREA]
            (cX, cY) = x + (w // 2), y + (h // 2)
            if (area > self.min_area) and (area < self.max_area):
                bbox = BoundingBox2D()
                bbox.center.position.x = float(cX)
                bbox.center.position.y = float(cY)
                bbox.size_x = float(w)
                bbox.size_y = float(h)
                detections.boxes.append(bbox)
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 1)
                cv2.circle(output, (int(cX), int(cY)), 1, (0, 0, 255), -1)
        self._detections_publisher.publish(detections)
        image_ros = self._bridge.cv2_to_imgmsg(cvim=np.flip(output, axis=0))
        image_ros.header = message.header
        self._result_publisher.publish(image_ros)
