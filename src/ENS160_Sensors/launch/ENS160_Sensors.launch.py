from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    sensor_pose_node = Node(
        package="ENS160_Sensors",
        executable="ENS160_Sensors_exec",
        name="sensor_pose_node",
      
        parameters=[
          
        {"serial_port": "/dev/ttyACM0"}, # Placeholder 
        {"baud_rate": 9600},
        {"timer_period": 0.5} # Placeholder, publishes fused data at 2 Hz 
          
        ]
      
    )
    ld.add_action(sensor_pose_node)
    return ld
