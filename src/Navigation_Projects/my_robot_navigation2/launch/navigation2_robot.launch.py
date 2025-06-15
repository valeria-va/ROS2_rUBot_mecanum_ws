# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim
# Modifier: [Your Name]

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Arguments de Llançament ---

    # Argument per seleccionar el fitxer del mapa
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value='map_square4m_sig.yaml',
        description='Name of the map file (e.g., my_map.yaml)')

    # Argument per seleccionar el fitxer de paràmetres de navegació
    declare_params_arg = DeclareLaunchArgument(
        'params',
        default_value='limo_sw.yaml',
        description='Name of the parameter file (e.g., limo_sw.yaml)')

    # Argument per utilitzar el temps de simulació
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # NOU: Argument per seleccionar el model URDF del robot
    declare_robot_model_arg = DeclareLaunchArgument(
        'robot',
        default_value='robot_arm/my_simple_robot.urdf', # Nom del teu fitxer URDF per defecte
        description='Name of the robot URDF file')

    # --- Configuracions i Rutes ---

    # Obtenim els valors dels arguments
    map_file_name = LaunchConfiguration('map')
    params_file_name = LaunchConfiguration('params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_urdf_file = LaunchConfiguration('robot')

    # Ruta al paquet de navegació
    nav_pkg_dir = get_package_share_directory('my_robot_navigation2')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # NOU: Ruta al paquet que conté la descripció del robot (URDF)
    # !!! CANVIA 'my_robot_description' PEL NOM REAL DEL TEU PAQUET DE DESCRIPCIÓ !!!
    robot_desc_pkg_dir = get_package_share_directory('my_robot_description')
    
    # Construcció de les rutes completes als fitxers
    map_path = PathJoinSubstitution([nav_pkg_dir, 'map', map_file_name])
    param_path = PathJoinSubstitution([nav_pkg_dir, 'param', params_file_name])
    rviz_config_path = PathJoinSubstitution([nav_pkg_dir, 'rviz', 'my_robot_navigation2.rviz'])
    robot_urdf_path = PathJoinSubstitution([robot_desc_pkg_dir, 'urdf', robot_urdf_file])

    # Node per publicar l'estat de les unions (joints) del robot
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node per publicar les transformades (TF) del robot a partir del URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Processa el fitxer XACRO en temps d'execució
            'robot_description': Command(['xacro ', robot_urdf_path]) 
        }]
    )

    # Node per llançar RViz2 amb la configuració especificada
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
            'params_file': param_path
        }.items(),
    )

    # --- Creació de la Descripció de Llançament ---
    
    return LaunchDescription([
        # Declaració de tots els arguments
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        declare_robot_model_arg,

        # Llançament de Nav2
        nav2_bringup_launch,

        # NOU: Llançament dels nodes per al model del robot
        joint_state_publisher_node,
        robot_state_publisher_node,

        # Llançament de RViz2
        rviz_node,
    ])
