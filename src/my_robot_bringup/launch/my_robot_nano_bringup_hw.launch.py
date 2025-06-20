import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # ===================================================================================
    #   Declaració dels arguments configurables per a tot el robot
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

    # ===================================================================================
    #   Obtenció dels directoris dels paquets necessaris
    # ===================================================================================
    my_robot_driver_pkg = get_package_share_directory('my_robot_driver')
    my_robot_bringup_pkg = get_package_share_directory('my_robot_bringup')
    rplidar_ros_pkg = get_package_share_directory('rplidar_ros')

    # ===================================================================================
    #   Definició de les accions d'inclusió dels llançadors
    # ===================================================================================

    # 1. Llançador per al driver dels motors Mecanum (és un fitxer .launch.xml)
    start_mecanum_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_driver_pkg, 'launch', 'rubot_nano_driver_mecanum.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('mecanum_serial_port')
        }.items()
    )
    
    # 2. Llançador per al LiDAR RPLiDAR (és un fitxer .launch.py)
    start_rplidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('rplidar_serial_port'),
            'frame_id': LaunchConfiguration('rplidar_frame_id')
        }.items()
    )

    # 3. Llançador per a la càmera USB (és un fitxer .launch.py)
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
    #   Creació de la descripció del llançament final
    # ===================================================================================
    ld = LaunchDescription()

    # Afegir tots els arguments
    ld.add_action(declare_mecanum_serial_port_arg)
    ld.add_action(declare_rplidar_serial_port_arg)
    ld.add_action(declare_rplidar_frame_id_arg)
    ld.add_action(declare_camera_width_arg)
    ld.add_action(declare_camera_height_arg)
    
    # Afegir totes les accions d'inclusió
    ld.add_action(start_mecanum_driver_cmd)
    ld.add_action(start_rplidar_cmd)
    ld.add_action(start_usb_cam_cmd)

    return ld