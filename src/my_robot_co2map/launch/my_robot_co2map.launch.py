from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os

"""
1st launch the trajectory generator file to obtain the trajectory, then run launch file:

ros2 launch my_robot_co2map my_robot_co2map.launch.py trajectory_csv:=trajectories/sweep.csv
"""


def generate_launch_description():

    # --- 1. ARGUMENTS ---
    traj_csv_arg = DeclareLaunchArgument(
        'trajectory_csv',
        default_value='sweep.csv',
        description='CSV file with waypoints to send'
    )

    # --- 2. TRAJECTORY EXECUTOR NODE ---
    executor_node = Node(
        package='my_robot_co2map',
        executable='trajectory_executor',
        name='trajectory_executor',
        output='screen',
        parameters=[
            {'use_sim_time': False},   # set True if running in Gazebo
            {'tolerance': 0.5},        # Nav2 goal tolerance
        ]
    )

    # --- 3. TRAJECTORY SENDER NODE ---
    sender_node = Node(
        package='my_robot_co2map',
        executable='trajectory_sender',
        name='trajectory_sender',
        output='screen',
        arguments=['--csv', traj_csv_arg.default_value]
    )

    # --- 4. OPTIONAL: RViz preview node ---
    rviz_preview_node = Node(
        package='my_robot_co2map',
        executable='rviz_preview',
        name='rviz_preview',
        output='screen'
    )

    # --- 5. RETURN LAUNCH DESCRIPTION ---
    return LaunchDescription([
        traj_csv_arg,
        executor_node,
        sender_node,
        rviz_preview_node
    ])
