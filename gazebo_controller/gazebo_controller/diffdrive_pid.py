import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class DiffDrivePID(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('diffdrive_pid')
        # Listener for tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to goal_pose
        self.subscription_goal_pose = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)

        # publish cmd_vel
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize PID controller parameters
        self.Kp = 1.0  # Proportional gain
        self.Kd = 0.1  # Derivative gain
        self.l = 0.5

        # Initialize previous position for derivative calculation
        self.prev_x = None
        self.prev_y = None  # Current y position

        self.goal_pose = None  # Initialize goal_pose to None

        # create a timer that runs at 30 hz to update robot's position and joint states
        self.dt = 1/30  # Time step
        self.timer = self.create_timer(self.dt, self.update_robot_state)

    def quaternion_to_euler(self, q):
        # Convert quaternion to euler angles (roll, pitch, yaw)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    def update_robot_state(self):
        try:
            # Get the current pose of the robot in the 'odom' frame
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'chassis', now)
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            current_theta = self.quaternion_to_euler(trans.transform.rotation)[2]
            current_pose = (current_x, current_y, current_theta)

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return

        # Compute control command using PID controller
        if self.goal_pose is not None:
            v_f, omega_f = self.PDController(current_pose, self.goal_pose)
            # Create Twist message and publish it
            twist_msg = Twist()
            twist_msg.linear.x = v_f
            twist_msg.angular.z = omega_f
            self.publisher_cmd_vel.publish(twist_msg)

    def PDController(self, current_pose, goal_pose):
        x_r = current_pose[0]
        y_r = current_pose[1]
        theta_r = current_pose[2]

        x_g = goal_pose.pose.position.x
        y_g = goal_pose.pose.position.y

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

            # Update previous position for next derivative calculation
            self.prev_x = x_p
            self.prev_y = y_p

        v_f = np.cos(theta_r) * (-self.Kp * (x_p - x_g) - self.Kd * x_p_dot) + np.sin(theta_r) * (-self.Kp * (y_p - y_g) - self.Kd * y_p_dot)
        omega_f = -np.sin(theta_r)/self.l * (-self.Kp * (x_p - x_g) - self.Kd * x_p_dot) + np.cos(theta_r)/self.l * (-self.Kp * (y_p - y_g) - self.Kd * y_p_dot)

        return v_f, omega_f

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

def main(args=None):
    rclpy.init(args=args)

    diffdrive_pid = DiffDrivePID()

    rclpy.spin(diffdrive_pid)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    diffdrive_pid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
