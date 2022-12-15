import numpy as np
from geometry_msgs.msg import Quaternion


def euler_from_quaternion(quaternion: Quaternion):
    """ Convert quaternion (w in last place) to euler roll, pitch, yaw.
    
    Parameters
    ----------
    quaternion: :py:class:`Quarternion <geometry_msgs.msg._quaternion.Quaternion>`
        Orientation as a quaternion.         

    Returns
    -------
    Tuple[float,float,float]
        Return roll, pitch, yaw euler angles.

    References
    ----------
    Source: https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_example
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
