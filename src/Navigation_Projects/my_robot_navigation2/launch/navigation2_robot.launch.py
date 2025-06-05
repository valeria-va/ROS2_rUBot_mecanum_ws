# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution # Importem PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare Arguments amb noms de fitxer per defecte
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value='map_square4m_sig.yaml',  # Nom del fitxer del mapa per defecte
        description='Name of the map file (e.g., my_map.yaml)')

    declare_params_arg = DeclareLaunchArgument(
        'params',
        default_value='limo_sw.yaml',  # Nom del fitxer de paràmetres per defecte
        description='Name of the parameter file (e.g., limo_sw.yaml)')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Assignar les configuracions de llançament des dels arguments declarats
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file_name = LaunchConfiguration('map_file_name')
    params_file_name = LaunchConfiguration('params_file_name')

    # Construir les rutes completes dels fitxers
    # Utilitzem PathJoinSubstitution per construir la ruta de manera més robusta
    map_path = PathJoinSubstitution([
        get_package_share_directory('my_robot_navigation2'),
        'map',
        map_file_name
    ])

    param_path = PathJoinSubstitution([
        get_package_share_directory('my_robot_navigation2'),
        'param',
        params_file_name
    ])

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('my_robot_navigation2'),
        'rviz',
        'my_robot_navigation2.rviz')

    return LaunchDescription([
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,             # Passem la ruta completa construïda
                'use_sim_time': use_sim_time,
                'params_file': param_path    # Passem la ruta completa construïda
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])