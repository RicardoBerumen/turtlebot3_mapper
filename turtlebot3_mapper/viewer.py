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

        # resized = cv2.resize(array.astype(np.uint8), (720, 720))
        image = cv2.flip(array, 0)
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
