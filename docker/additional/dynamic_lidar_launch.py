import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch

def generate_launch_description():
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', 
        default_value='livox_frame', 
        description='Frame ID for LiDAR'
    )
    
    user_config_arg = DeclareLaunchArgument(
        'user_config_path',
        default_value=os.path.join(os.path.split(os.path.realpath(__file__))[0] + '/',
                                 '../config', 'livox_lidar_config.json'),
        description='Path to LiDAR configuration file'
    )
    
    lvx_file_arg = DeclareLaunchArgument(
        'lvx_file_path',
        default_value='/home/livox/livox_test.lvx',
        description='Path to LVX file'
    )
    
    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value='1',
        description='0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format'
    )
    
    multi_topic_arg = DeclareLaunchArgument(
        'multi_topic',
        default_value='1',
        description='0-All LiDARs share the same topic, 1-One LiDAR one topic'
    )
    
    data_src_arg = DeclareLaunchArgument(
        'data_src',
        default_value='0',
        description='0-lidar,1-hub'
    )
    
    publish_freq_arg = DeclareLaunchArgument(
        'publish_freq',
        default_value='10.0',
        description='Frequency of publish'
    )
    
    output_type_arg = DeclareLaunchArgument(
        'output_type',
        default_value='0',
        description='Output type'
    )
    
    cmdline_bd_code_arg = DeclareLaunchArgument(
        'cmdline_bd_code',
        default_value='livox0000000001',
        description='Broadcast code'
    )
    
    # Get LaunchConfiguration objects
    frame_id = LaunchConfiguration('frame_id')
    user_config_path = LaunchConfiguration('user_config_path')
    lvx_file_path = LaunchConfiguration('lvx_file_path')
    xfer_format = LaunchConfiguration('xfer_format')
    multi_topic = LaunchConfiguration('multi_topic')
    data_src = LaunchConfiguration('data_src')
    publish_freq = LaunchConfiguration('publish_freq')
    output_type = LaunchConfiguration('output_type')
    cmdline_bd_code = LaunchConfiguration('cmdline_bd_code')
    
    # Set parameters for the LiDAR driver node
    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    
    livox_driver = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    
    return LaunchDescription([
        frame_id_arg,
        user_config_arg,
        lvx_file_arg,
        xfer_format_arg,
        multi_topic_arg,
        data_src_arg,
        publish_freq_arg,
        output_type_arg,
        cmdline_bd_code_arg,
        livox_driver
    ])
