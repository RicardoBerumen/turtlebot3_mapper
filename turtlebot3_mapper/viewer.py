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
        self._init = True
        array = self.occupancygrid_to_numpy(message)
        array[array == -1] = 57
        array[array == 0] = 127
        array[array == 100] = 0
        input = array.astype('uint8')
        # Applying 7x7 Gaussian Blur
        blurred = cv2.GaussianBlur(input, (7, 7), 0)
        # Applying threshold
        threshold = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        # Apply the Component analysis function
        analysis = cv2.connectedComponentsWithStats(threshold, 4, cv2.CV_32S)
        (totalLabels, label_ids, values, centroid) = analysis

        # Initialize a new image to store all the output components
        input = array.astype('uint8')
        output = cv2.cvtColor(input, cv2.COLOR_GRAY2BGR)


        # Loop through each component
        count = 0
        for i in range(1, totalLabels):
            # Area of the component
            x = values[i, cv2.CC_STAT_LEFT]
            y = values[i, cv2.CC_STAT_TOP]
            w = values[i, cv2.CC_STAT_WIDTH]
            h = values[i, cv2.CC_STAT_HEIGHT]
            area = values[i, cv2.CC_STAT_AREA]
            (cX, cY) = centroid[i]
            if (area > 100) and (area < 1000):
                count += 1
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 1)
                cv2.circle(output, (int(cX), int(cY)), 1, (0, 0, 255), -1)   
                # componentMask = (label_ids == i).astype("uint8") * 255
                # output = cv2.bitwise_or(output, componentMask)

        # resized = cv2.resize(output, (500, 500))
        image = cv2.flip(output, 0)
        rotated_image = cv2.rotate(src = image, rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

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
