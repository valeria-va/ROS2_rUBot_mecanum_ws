import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ===================================================================================
    #   Declaració dels arguments configurables per a tot el robot
    # ===================================================================================
    
    # Arguments per al driver del robot
    declare_mecanum_serial_port_arg = DeclareLaunchArgument(
        'mecanum_serial_port', default_value='/dev/ttyACM0',
        description='Port sèrie per a la placa dels motors'
    )

    # Arguments per al LiDAR
    declare_rplidar_serial_port_arg = DeclareLaunchArgument(
        'rplidar_serial_port', default_value='/dev/ttyUSB0',
        description='Port sèrie per al RPLiDAR'
    )
    # NOU: Argument pel frame_id del LiDAR, amb 'base_link' per defecte
    declare_rplidar_frame_id_arg = DeclareLaunchArgument(
        'rplidar_frame_id', default_value='base_link',
        description='Frame ID per a les dades del LiDAR'
    )
    
    # Arguments per a la Càmera
    declare_camera_width_arg = DeclareLaunchArgument(
        'camera_width', default_value='160',
        description='Amplada de la imatge de la càmera'
    )
    declare_camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='120',
        description='Alçada de la imatge de la càmera'
    )

    # Arguments per al model del robot
    declare_robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='robot_arm/my_simple_robot.urdf',
        description='Nom del fitxer URDF/XACRO dins del paquet de descripció'
    )
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Indica si es vol llançar la GUI de les articulacions'
    )

    # ===================================================================================
    #   Obtenció dels directoris dels paquets necessaris
    # ===================================================================================
    my_robot_driver_pkg = get_package_share_directory('my_robot_driver')
    my_robot_bringup_pkg = get_package_share_directory('my_robot_bringup')
    rplidar_ros_pkg = get_package_share_directory('rplidar_ros')

    # ===================================================================================
    #   Preparació de la configuració dels nodes
    # ===================================================================================
    
    robot_model = LaunchConfiguration('robot_model')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'), 'urdf', robot_model
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Eliminem el joint_state_publisher_gui perquè el driver del robot ja publica /joint_states
    # Si en algun moment el necessites per depurar sense el driver, pots descomentar-lo.
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=IfCondition(use_rviz)
    # )

    start_mecanum_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_driver_pkg, 'launch', 'rubot_nano_driver_mecanum.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('mecanum_serial_port')
        }.items()
    )
    
    start_rplidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_a1_launch.py')
        ),
        # Utilitzem l'argument declarat a dalt
        launch_arguments={
            'serial_port': LaunchConfiguration('rplidar_serial_port'),
            'frame_id': LaunchConfiguration('rplidar_frame_id')
        }.items()
    )

    start_usb_cam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_bringup_pkg, 'launch', 'usb_cam_custom.launch.py')
        ),
        launch_arguments={
            'image_width': LaunchConfiguration('camera_width'),
            'image_height': LaunchConfiguration('camera_height')
        }.items()
    )

    # ===================================================================================
    #   Creació de la descripció del llançament final
    # ===================================================================================
    ld = LaunchDescription()

    # Afegir tots els arguments
    ld.add_action(declare_mecanum_serial_port_arg)
    ld.add_action(declare_rplidar_serial_port_arg)
    ld.add_action(declare_rplidar_frame_id_arg) # NOU
    ld.add_action(declare_camera_width_arg)
    ld.add_action(declare_camera_height_arg)
    ld.add_action(declare_robot_model_arg)
    ld.add_action(declare_use_rviz_arg)
    
    # Afegir tots els nodes i llançadors
    ld.add_action(start_mecanum_driver_cmd)
    ld.add_action(start_rplidar_cmd)
    ld.add_action(start_usb_cam_cmd)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_gui_node) # Comentat perquè és redundant

    return ld