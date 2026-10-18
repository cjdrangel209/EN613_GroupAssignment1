import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DiffDrivePID(Node):
    """Drive the differential-drive robot toward RViz /goal_pose goals.

    The robot pose is read from the odom -> chassis transform and commands are
    published as geometry_msgs/Twist on /cmd_vel.
    """

    def __init__(self):
        super().__init__('diffdrive_pid')

        # Listener for tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to /goal_pose
        self.subscription_goal_pose = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )

        # Publish to /cmd_vel
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_marker = self.create_publisher(
            Marker, '/robot_visualization', 10
        )

        # Controller parameters
        self.Kp = 0.8       # Proportional gain
        self.Kd = 0.05      # Derivative gain
        self.l = 0.4        # Look-ahead point distance along heading
        self.goal_tolerance = 0.15  # Stop within 15 cm of target

        # Previous position for derivative calculation
        self.prev_x = None
        self.prev_y = None

        self.goal_pose = None

        # Control loop at 30 Hz
        self.dt = 1.0 / 30.0
        self.timer = self.create_timer(self.dt, self.update_robot_state)
        self.get_logger().info('DiffDrive PID Controller ready. Send a 2D Goal Pose in RViz.')

    def quaternion_to_euler(self, q):
        """Convert a ROS quaternion to roll, pitch, and yaw angles."""
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def update_robot_state(self):
        """Read the latest transform and publish a velocity command if active."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'chassis', now)
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            current_theta = self.quaternion_to_euler(trans.transform.rotation)[2]
            current_pose = (current_x, current_y, current_theta)
            self.publish_robot_marker(trans.transform)
        except TransformException as ex:
            return

        if self.goal_pose is not None:
            v_f, omega_f = self.PDController(current_pose, self.goal_pose)
            twist_msg = Twist()
            twist_msg.linear.x = float(v_f)
            twist_msg.angular.z = float(omega_f)
            self.publisher_cmd_vel.publish(twist_msg)

    def publish_robot_marker(self, transform):
        """Publish a simple RViz cube at the robot's current odometry pose."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = transform.translation.x
        marker.pose.position.y = transform.translation.y
        marker.pose.position.z = 0.16
        marker.pose.orientation = transform.rotation
        marker.scale.x = 0.8
        marker.scale.y = 0.55
        marker.scale.z = 0.25
        marker.color.r = 0.1
        marker.color.g = 0.55
        marker.color.b = 0.95
        marker.color.a = 0.9
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.publisher_marker.publish(marker)

    def PDController(self, current_pose, goal_pose):
        """Compute bounded linear/angular velocities using look-ahead PD control."""
        x_r = current_pose[0]
        y_r = current_pose[1]
        theta_r = current_pose[2]

        x_g = goal_pose.pose.position.x
        y_g = goal_pose.pose.position.y

        # Distance error to target
        distance_to_goal = math.hypot(x_g - x_r, y_g - y_r)
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached! Holding position.')
            self.goal_pose = None
            self.prev_x = None
            self.prev_y = None
            return 0.0, 0.0

        # Look-ahead point
        x_p = x_r + self.l * np.cos(theta_r)
        y_p = y_r + self.l * np.sin(theta_r)

        if self.prev_x is None or self.prev_y is None:
            self.prev_x = x_p
            self.prev_y = y_p
            x_p_dot = 0.0
            y_p_dot = 0.0
        else:
            x_p_dot = (x_p - self.prev_x) / self.dt
            y_p_dot = (y_p - self.prev_y) / self.dt
            self.prev_x = x_p
            self.prev_y = y_p

        # Feedback-linearized control law
        u_x = -self.Kp * (x_p - x_g) - self.Kd * x_p_dot
        u_y = -self.Kp * (y_p - y_g) - self.Kd * y_p_dot

        v_f = np.cos(theta_r) * u_x + np.sin(theta_r) * u_y
        omega_f = (-np.sin(theta_r) / self.l) * u_x + (np.cos(theta_r) / self.l) * u_y

        # Safety clamps on velocities
        v_f = max(min(v_f, 0.6), -0.6)
        omega_f = max(min(omega_f, 1.2), -1.2)

        return v_f, omega_f

    def goal_pose_callback(self, msg):
        """Store a newly selected RViz goal pose and reset derivative history."""
        self.prev_x = None
        self.prev_y = None
        self.goal_pose = msg
        self.get_logger().info(
            f'New goal set: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    diffdrive_pid = DiffDrivePID()
    try:
        rclpy.spin(diffdrive_pid)
    except KeyboardInterrupt:
        pass
    finally:
        diffdrive_pid.publisher_cmd_vel.publish(Twist())
        diffdrive_pid.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
