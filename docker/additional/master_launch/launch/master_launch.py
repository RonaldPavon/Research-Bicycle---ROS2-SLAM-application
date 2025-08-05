from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments to enable/disable sensors
    use_front_camera = LaunchConfiguration('use_front_camera')
    use_rear_camera = LaunchConfiguration('use_rear_camera')
    use_front_lidar = LaunchConfiguration('use_front_lidar')
    use_rear_lidar = LaunchConfiguration('use_rear_lidar')
    enable_gnss = LaunchConfiguration('enable_gnss')    

    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('use_front_camera', default_value='false', description='Launch front camera'))
    ld.add_action(DeclareLaunchArgument('use_rear_camera', default_value='false', description='Launch rear camera'))
    ld.add_action(DeclareLaunchArgument('use_front_lidar', default_value='false', description='Launch front lidar'))
    ld.add_action(DeclareLaunchArgument('use_rear_lidar', default_value='false', description='Launch rear lidar'))
    ld.add_action(DeclareLaunchArgument('enable_gnss', default_value='true', description='Launch gnss'))

    # Paths to sensor launch files
    camera_launch_path = os.path.join(
        get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
    multi_camera_launch_path = os.path.join(
        get_package_share_directory('zed_multi_camera'), 'launch', 'zed_multi_camera.launch.py')
    lidar_launch_path = os.path.join(
        get_package_share_directory('livox_ros2_driver'), 'launch', 'dynamic_lidar_launch.py')
    gnss_launch_path = os.path.join(
        get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node_zedf9p-launch.py')
    
    #Paths to lidar config files

    front_lidar_config = os.path.join(
        get_package_share_directory('livox_ros2_driver'), 'config', 'front_lidar_config.json')
    rear_lidar_config = os.path.join(
        get_package_share_directory('livox_ros2_driver'), 'config', 'rear_lidar_config.json')

    front_camera_expression = PythonExpression([
        '"', use_front_camera, '" == "true" and ',
        '"', use_rear_camera, '" == "false" '
    ])

    rear_camera_expression = PythonExpression([
        '"', use_rear_camera, '" == "true" and ',
        '"', use_front_camera, '" == "false"'
    ])

    all_camera_expression = PythonExpression([
        '"', use_rear_camera, '" == "true" and ',
        '"', use_front_camera, '" == "true"'
    ])

    front_lidar_expression = PythonExpression([
        '"', use_front_lidar, '" == "true" and ',
        '"', use_rear_lidar, '" == "false"'
    ])

    rear_lidar_expression = PythonExpression([
        '"', use_rear_lidar, '" == "true" and ',
        '"', use_front_lidar, '" == "false"'
    ])

    all_lidar_expression = PythonExpression([
        '"', use_rear_lidar, '" == "true" and ',
        '"', use_front_lidar, '" == "true"'
    ])


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gnss_launch_path),
        condition=IfCondition(enable_gnss),
        
    ))


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        condition=IfCondition(front_camera_expression),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'front_camera',
            'namespace' : 'zed_multi',
            'camera_id' : '0'
        }.items()
    ))


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        condition=IfCondition(rear_camera_expression),
        launch_arguments={
            'camera_model': 'zedx',
            'camera_name': 'rear_camera',
            'namespace' : 'zed_multi',
            'camera_id' : '1'
        }.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multi_camera_launch_path),
        condition=IfCondition(all_camera_expression),
        launch_arguments={
            'cam_models': '[zedx,zedx]',
            'cam_names': '[front_camera,rear_camera]',
            'cam_ids' : '[0,1]'
        }.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        condition=IfCondition(front_lidar_expression),
        launch_arguments={
            'frame_id': 'front_lidar_frame',
            'user_config_path': front_lidar_config,
            'multi_topic': '1',            
        }.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        condition=IfCondition(rear_lidar_expression),
        launch_arguments={
            'frame_id': 'rear_lidar_frame',
            'user_config_path': rear_lidar_config,
            'multi_topic': '1',            
        }.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        condition=IfCondition(all_lidar_expression),
        launch_arguments={
            'multi_topic': '1',            
        }.items()
    ))

    return ld


