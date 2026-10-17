from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_controller').find('gazebo_controller')    
    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', default_value='ros_gz_bridge', description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value=os.path.join(pkg_share, 'config', 'bridge.yaml'), description='YAML config file'
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        declare_bridge_name_cmd,
        declare_config_file_cmd,             
        RosGzBridge(
            bridge_name=LaunchConfiguration('bridge_name'),
            config_file=LaunchConfiguration('config_file'),
        ),
    ])

    return ld