from setuptools import setup

package_name = 'turtlebot3_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autonomous-robots',
    description='Simple implementation of an occupancy map using laser scan',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapper=turtlebot3_mapper.mapper:main',
            'viewer=turtlebot3_mapper.viewer:main',
        ],
    },
)
