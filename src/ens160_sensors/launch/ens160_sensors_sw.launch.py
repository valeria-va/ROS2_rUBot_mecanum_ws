from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    sensor_pose_node = Node(
        package="ens160_sensors",
        executable="ens160_sensors_sw_exec",
        name="sensor_pose_node_sw",
      
        parameters=[
          
        {"timer_period": 1} 
          
        ]
      
    )
    ld.add_action(sensor_pose_node)
    return ld
