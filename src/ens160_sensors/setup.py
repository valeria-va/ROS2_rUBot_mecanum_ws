from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ens160_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Standard ROS 2 Metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Other source files (like launch files)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        
    ],
    
    install_requires=['setuptools', 'rclpy', 'ens160_interfaces'], 
    
    zip_safe=True,
    maintainer='root',    
    entry_points={
        'console_scripts': [
            'ens160_sensors_hw_exec = ens160_sensors.sensor_pose_node_hw:main',
            'ens160_sensors_sw_exec = ens160_sensors.sensor_pose_node_sw:main',

        ],
    },
)