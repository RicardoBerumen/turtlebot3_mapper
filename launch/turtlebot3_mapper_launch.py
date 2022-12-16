import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    turtlebot3_mapper_rviz_dir = os.path.join(
        get_package_share_directory('turtlebot3_mapper'),
        'rviz',
    )
    turtlebot3_gazebo_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    turtlebot3_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_launch_dir,
                'turtlebot3_world.launch.py',
            ),
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=os.path.join(
            turtlebot3_mapper_rviz_dir,
            'config.rviz',
        ),
    )
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_launch_file_dir,
            'rviz_launch.py',
        )),
        launch_arguments={'rviz_config': rviz_config_file}.items(),
    )
    turtlebot3_occupancy_grid_node = Node(
        package='turtlebot3_mapper',
        namespace='',
        executable='turtlebot3_occupancy_grid',
        name='turtlebot3_occupancy_grid',
        parameters=[
            {
                "width": 8.0
            },
            {
                "height": 8.0
            },
            {
                "resolution": 0.03,
            },
            {
                "min_prob": 0.01
            },
            {
                "max_prob": 0.99,
            },
            {
                "prob_occupied": 0.6,
            },
            {
                "prob_free": 0.4,
            },
            {
                "prob_priori": 0.5,
            },
        ],
    )
    turtlebot3_object_detector_node = Node(
        package='turtlebot3_mapper',
        namespace='',
        executable='turtlebot3_object_detector',
        name='turtlebot3_object_detector',
        parameters=[
            {
                "threshold": 150,
            },
            {
                "min_area": 30,
            },
            {
                "max_area": 100,
            },
            {
                "connectivity": 4,
            },
        ],
    )
    turtlebot3_explorer_node = Node(
        package='turtlebot3_mapper',
        namespace='',
        executable='turtlebot3_explorer',
        name='turtlebot3_explorer',
        parameters=[
            {
                "view_angle": 30.0,
            },
            {
                "min_rand": 10,
            },
            {
                "max_rand": 20,
            },
            {
                "safety_distance": 0.5,
            },
            {
                "linear_speed": 0.15,
            },
            {
                "angular_speed": 0.25,
            },
            {
                "update_rate": 0.5,
            },
        ],
    )
    ld = LaunchDescription()
    ld.add_action(rviz_cmd)
    ld.add_action(turtlebot3_world_cmd)
    ld.add_action(turtlebot3_explorer_node)
    ld.add_action(turtlebot3_occupancy_grid_node)
    ld.add_action(turtlebot3_object_detector_node)
    return ld