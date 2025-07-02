# Copyright 2019 Open Source Robotics Foundation, Inc.
# ... (llicència)
# Author: Darby Lim
# Modifier: [Your Name]

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments de Llançament ---
    nav_pkg_dir = get_package_share_directory('my_robot_navigation2')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_pkg_dir, 'map', 'map_square3m_wall.yaml'),
        description='Full path to map file')

    declare_params_arg = DeclareLaunchArgument(
        'params_file', # Canviat a 'params_file' per coincidir amb l'estàndard de Nav2
        default_value=os.path.join(nav_pkg_dir, 'param', 'rubot_sw.yaml'),
        description='Full path to param file for nav2')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # --- Obtenim els valors dels arguments ---
    map_path = LaunchConfiguration('map')
    params_file_path = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # --- Configuració dels Nodes ---

    # Node per llançar RViz2 amb la configuració de navegació
    rviz_config_path = os.path.join(nav_pkg_dir, 'rviz', 'my_robot_navigation2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Inclusió del fitxer de llançament principal de Nav2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': params_file_path
        }.items(),
    )

    # --- Creació de la Descripció de Llançament ---
    
    return LaunchDescription([
        # Declaració de tots els arguments
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,

        # Llançament de Nav2
        nav2_bringup_launch,

        # Llançament de RViz2
        rviz_node,
    ])