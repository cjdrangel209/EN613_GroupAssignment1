import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gazebo_controller')


    world_path = os.path.join(pkg_dir, 'worlds', 'building_robot.sdf')


    rviz_config_path = os.path.join(pkg_dir, 'launch', 'default.rviz')


    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/vehicle_blue/odometry', '/odom'),
            ('/model/vehicle_blue/tf', '/tf'),
        ],
        output='screen'
    )

    pid_controller = Node(
        package='gazebo_controller',
        executable='pid_controller',
        output='screen'
    )

    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        pid_controller,
        rviz
    ])
