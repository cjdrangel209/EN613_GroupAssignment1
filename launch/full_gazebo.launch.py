"""Launch Gazebo maze, ROS bridges, PID goal controller, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """Create the full maze simulation launch description."""
    pkg_dir = get_package_share_directory('gazebo_controller')
    world_path = os.path.join(pkg_dir, 'worlds', 'maze.sdf')
    rviz_config_path = os.path.join(pkg_dir, 'launch', 'group_project.rviz')
    robot_description_path = os.path.join(pkg_dir, 'urdf', 'vehicle_blue.urdf')

    with open(robot_description_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/model/vehicle_blue/odometry', '/odom'),
            ('/model/vehicle_blue/tf', '/tf'),
        ],
        output='screen',
    )

    # Publishes the URDF-backed RobotModel and fixed chassis-to-wheel transforms.
    # Gazebo's DiffDrive plugin remains the sole publisher of odom -> chassis.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    pid_controller = Node(
        package='gazebo_controller',
        executable='pid_controller',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([gz_sim, bridge, robot_state_publisher, pid_controller, rviz])
