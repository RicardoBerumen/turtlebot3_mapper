from os import path
from glob import glob
from setuptools import setup

package_name = 'turtlebot3_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        path.join(package_name, "turtlebot3_explorer"),
        path.join(package_name, "turtlebot3_occupancy_grid"),
        path.join(package_name, "turtlebot3_object_detector"),
        path.join(package_name, "turtlebot3_mission_controller"),
        path.join(package_name, "turtlebot3_mission_client"),
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=["Luiz Carlos Cosmi Filho"],
    author_email=["luizcarloscosmifilho@gmail.com"],
    maintainer="Luiz Carlos Cosmi Filho",
    maintainer_email="luizcarloscosmifilho@gmail.com",
    keywords=['ROS', 'ROS2', 'mapping', 'perception', 'kinematics'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Set of packages developed to explore, map and detect obstacles in an \
        enviromment',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_explorer = \
                turtlebot3_mapper.turtlebot3_explorer.main:main',
            'turtlebot3_occupancy_grid = \
                turtlebot3_mapper.turtlebot3_occupancy_grid.main:main',
            'turtlebot3_object_detector = \
                turtlebot3_mapper.turtlebot3_object_detector.main:main',
            'turtlebot3_mission_controller = \
                turtlebot3_mapper.turtlebot3_mission_controller.main:main',
            'turtlebot3_mission_client = \
                turtlebot3_mapper.turtlebot3_mission_client.main:main',
        ],
    },
)
