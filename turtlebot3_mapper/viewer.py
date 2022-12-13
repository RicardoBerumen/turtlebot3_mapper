import sys
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import OccupancyGrid

class MapViewer(Node):

    def __init__(self,
                 node_name: str = 'map_viewer'):

        super(MapViewer, self).__init__(node_name=node_name)

        self._map_subscriber = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/custom_map",
            callback=self._map_callback,
            qos_profile=10,
        )
        self._init = False
        self.get_logger().info(f"Init {node_name}")

    def _map_callback(self, message: OccupancyGrid):
        # get occupancy grid as a numpy array int8
        array = self.occupancygrid_to_numpy(message)
        # convert int8 array to float32
        input = array.astype("float32")
        # remap to 0-255 interval
        input = 255*(input - np.min(input))/(np.max(input) - np.min(input))
        # if it is unkown or free, mark as 0
        input[input <= 150] = 0
        # if it is occupied, mark as 255
        input[input > 150] = 255
        # get array as uint8
        input = input.astype('uint8')
        # kernel to apply morphological transformation
        kernel = np.ones((5,5),np.uint8)
        # apply closing transformation
        closing = cv2.morphologyEx(input, cv2.MORPH_CLOSE, kernel)
        # get component connected information
        analysis = cv2.connectedComponentsWithStats(closing, 8, cv2.CV_32S)
        # unpack information
        (totalLabels, label_ids, values, centroid) = analysis
        # get RGB image to show
        output = cv2.cvtColor(closing, cv2.COLOR_GRAY2BGR)
        # Loop through each component
        for i in range(1, totalLabels):
            x = values[i, cv2.CC_STAT_LEFT]
            y = values[i, cv2.CC_STAT_TOP]
            w = values[i, cv2.CC_STAT_WIDTH]
            h = values[i, cv2.CC_STAT_HEIGHT]
            area = values[i, cv2.CC_STAT_AREA]
            (cX, cY) = x+(w//2), y+(h//2)
            if (area > 30) and (area < 300):
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 1)
                cv2.circle(output, (int(cX), int(cY)), 1, (0, 0, 255), -1)   
        # rotate just to plot equal gazebo
        rotated_image = cv2.rotate(src = output, rotateCode = cv2.ROTATE_180)
        # show image
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.imshow("output", rotated_image)
        if cv2.waitKey(1) == ord('q'):
            sys.exit(1)

    @staticmethod
    def occupancygrid_to_numpy(msg, info=None):
        """
        Source: http://docs.ros.org/en/jade/api/ros_numpy/html/occupancy__grid_8py_source.html
        """
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return data



def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    node = MapViewer()
    try:
        rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
