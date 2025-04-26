from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_launch_arguments
import ast

def generate_launch_description():
    """
    Generates a launch description for the navigation task node,
    optionally accepting waypoints as a launch argument.
    """

    # Declare the launch argument for waypoints.  We use a default value.
    declare_waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value='[(2.0, -2.0, 1.57), (4.0, 0.8, 0.0), (8.0, 1.0, -1.57), (8.0, -0.5, 1.57), (5.0, 5.0, 3.14), (3.0, 4.0, 1.57), (4.0, 5.0, 0.0), (5.0, 3.0, -1.57), (4.0, 0.8, 3.14), (-4.0, 3.5, -1.57), (-4.0, 0.0, 1.57)]',
        description='A list of waypoints, each as a tuple (x, y, yaw_in_radians).  Example: "[(1.0, 2.0, 0.0), (3.0, 4.0, 1.57)]"',
    )

    # Create the node action, passing the waypoints as a parameter.
    navigation_node = Node(
        package='my_robot_nav_control',
        executable='nav_waypoints_exec',  # Make sure this matches your executable name
        name='nav_waypoints_node',
        parameters=[
            {
                'waypoints': LaunchConfiguration('waypoints'),
            },
        ],
        output='screen',  #  Good for seeing the output in the terminal
    )

    # Create the launch description and add the actions.
    ld = LaunchDescription()
    ld.add_action(declare_waypoints_arg)
    ld.add_action(navigation_node)
    return ld
