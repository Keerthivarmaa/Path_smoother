# smooth_path_controller/trajectory_tracker.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan # Import LaserScan message
import math
from tf_transformations import euler_from_quaternion
from builtin_interfaces.msg import Time

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')
        self.subscription = self.create_subscription(Path, 'trajectory', self.trajectory_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10) # Subscribe to LaserScan
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Declare ROS parameters for controller gains and look-ahead distance
        self.declare_parameter('kp_linear', 0.8) # Proportional gain for linear velocity
        self.declare_parameter('kp_angular', 1.5) # Proportional gain for angular velocity
        self.declare_parameter('look_ahead_time', 0.5) # Seconds to look ahead on the trajectory
        self.declare_parameter('max_linear_velocity', 0.5) # Max linear velocity
        self.declare_parameter('max_angular_velocity', 1.0) # Max angular velocity (rad/s)
        self.declare_parameter('goal_tolerance_distance', 0.1) # Distance to consider goal reached
        self.declare_parameter('goal_tolerance_angle', 0.1) # Angle to consider goal orientation reached

        # Obstacle Avoidance Parameters - Adjusted for more aggressive avoidance
        self.declare_parameter('obstacle_min_distance', 0.6) # Increased: React earlier (was 0.4)
        self.declare_parameter('obstacle_detection_angle_range', 0.8) # Increased: Look wider (was 0.5 rad = ~28 deg, now 0.8 rad = ~45 deg)
        self.declare_parameter('avoidance_angular_speed', 0.8) # Increased: Turn faster (was 0.5)
        self.declare_parameter('avoidance_linear_speed_reduction', 0.4) # Increased: Slow down more (was 0.2)
        self.declare_parameter('stop_distance', 0.09) # New parameter: Very close distance to force stop

        self.timer = self.create_timer(0.05, self.track_trajectory) # Faster control loop (e.g., 20 Hz)
        
        self.trajectory = []
        self.trajectory_start_time = None # Timestamp when the trajectory was received
        self.current_pose = None # Current robot pose from odometry
        self.last_scan_data = None # Store the latest laser scan data

        self.get_logger().info("Trajectory Tracker Initialized")

    def trajectory_callback(self, msg: Path):
        self.trajectory = msg.poses
        if self.trajectory:
            self.trajectory_start_time = self.trajectory[0].header.stamp 
            self.get_logger().info(f"Trajectory received with {len(self.trajectory)} poses. Start time: {self.trajectory_start_time.sec}.{self.trajectory_start_time.nanosec}")
        else:
            self.trajectory_start_time = None
            self.get_logger().warn("Received empty trajectory.")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        self.last_scan_data = msg

    def get_current_robot_yaw(self):
        if self.current_pose is None:
            return 0.0
        orientation_q = self.current_pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw

    def find_target_pose_on_trajectory(self, current_time_ns):
        if not self.trajectory or self.trajectory_start_time is None:
            self.get_logger().warn("find_target_pose_on_trajectory: No trajectory or start time.")
            return None

        traj_start_ns = self.trajectory_start_time.sec * 1e9 + self.trajectory_start_time.nanosec
        elapsed_time_ns = current_time_ns - traj_start_ns
        elapsed_time_s = elapsed_time_ns / 1e9

        look_ahead_time = self.get_parameter('look_ahead_time').get_parameter_value().double_value
        target_relative_time_s = elapsed_time_s + look_ahead_time

        # self.get_logger().info(f"--- find_target_pose_on_trajectory ---")
        # self.get_logger().info(f"Current sim time (ns): {current_time_ns}")
        # self.get_logger().info(f"Trajectory start time (ns): {traj_start_ns}")
        # self.get_logger().info(f"Elapsed time (s): {elapsed_time_s:.2f}")
        # self.get_logger().info(f"Look-ahead time (s): {look_ahead_time:.2f}")
        # self.get_logger().info(f"Target RELATIVE time (s): {target_relative_time_s:.2f}")


        for i in range(len(self.trajectory) - 1):
            p1 = self.trajectory[i]
            p2 = self.trajectory[i+1]

            t1_relative_s = ((p1.header.stamp.sec * 1e9 + p1.header.stamp.nanosec) - traj_start_ns) / 1e9
            t2_relative_s = ((p2.header.stamp.sec * 1e9 + p2.header.stamp.nanosec) - traj_start_ns) / 1e9

            if t1_relative_s <= target_relative_time_s <= t2_relative_s:
                if (t2_relative_s - t1_relative_s) > 1e-6:
                    alpha = (target_relative_time_s - t1_relative_s) / (t2_relative_s - t1_relative_s)
                else:
                    alpha = 0.0

                target_pose = PoseStamped().pose
                target_pose.position.x = p1.pose.position.x + alpha * (p2.pose.position.x - p1.pose.position.x)
                target_pose.position.y = p1.pose.position.y + alpha * (p2.pose.position.y - p1.pose.position.y)
                
                # self.get_logger().info(f"Interpolated target pose: ({target_pose.position.x:.2f}, {target_pose.position.y:.2f}) at relative time {target_relative_time_s:.2f}")
                return target_pose

        if self.trajectory:
            # self.get_logger().info(f"Target relative time beyond trajectory end. Aiming for final point: ({self.trajectory[-1].pose.position.x:.2f}, {self.trajectory[-1].pose.position.y:.2f})")
            return self.trajectory[-1].pose
        
        self.get_logger().warn("find_target_pose_on_trajectory: Should not reach here if trajectory exists.")
        return None

    def check_for_obstacles(self):
        if self.last_scan_data is None:
            return None # No scan data yet

        min_dist = self.get_parameter('obstacle_min_distance').get_parameter_value().double_value
        angle_range = self.get_parameter('obstacle_detection_angle_range').get_parameter_value().double_value # e.g., 0.5 rad = ~28 degrees
        stop_dist = self.get_parameter('stop_distance').get_parameter_value().double_value # Get the new stop distance parameter

        # Calculate indices for the front sector
        # The laser scan ranges from angle_min to angle_max
        # 0 is directly in front of the robot
        angle_min_scan = self.last_scan_data.angle_min
        angle_max_scan = self.last_scan_data.angle_max
        angle_increment = self.last_scan_data.angle_increment

        # Find indices corresponding to the detection angle range
        # Start from -angle_range/2 to +angle_range/2 around 0 (front)
        start_angle = -angle_range / 2.0
        end_angle = angle_range / 2.0

        # Convert angles to array indices
        # Ensure indices are within valid range [0, len(ranges)-1]
        start_idx = math.floor((start_angle - angle_min_scan) / angle_increment)
        end_idx = math.ceil((end_angle - angle_min_scan) / angle_increment)

        # Clamp indices to valid range
        start_idx = max(0, min(start_idx, len(self.last_scan_data.ranges) - 1))
        end_idx = max(0, min(end_idx, len(self.last_scan_data.ranges) - 1))

        # Check for obstacles in the front sector
        obstacle_detected = False
        min_front_distance = float('inf')
        
        # Determine if obstacle is more to the left or right
        left_obstacle_dist = float('inf')
        right_obstacle_dist = float('inf')
        
        # Divide the front sector into left, center, right for simple steering
        # This helps decide which way to turn
        # The angles are relative to the robot's heading.
        # Negative angles are to the left, positive to the right
        left_turn_sector_end_angle = -0.1 # Example: up to -0.1 rad from center
        right_turn_sector_start_angle = 0.1 # Example: from 0.1 rad from center


        for i in range(start_idx, end_idx + 1):
            if i < 0 or i >= len(self.last_scan_data.ranges):
                continue

            distance = self.last_scan_data.ranges[i]
            # Filter out invalid readings (inf for max range, nan for no data)
            if not math.isinf(distance) and not math.isnan(distance) and distance < min_dist:
                obstacle_detected = True
                min_front_distance = min(min_front_distance, distance)

                # Get the actual angle of this laser beam relative to robot's front
                current_angle_relative_to_robot = angle_min_scan + i * angle_increment
                
                # Assign to left/right based on angle
                if current_angle_relative_to_robot < 0: # Left side of robot's front
                    left_obstacle_dist = min(left_obstacle_dist, distance)
                else: # Right side of robot's front
                    right_obstacle_dist = min(right_obstacle_dist, distance)
        
        if obstacle_detected:
            self.get_logger().warn(f"Obstacle detected! Min front dist: {min_front_distance:.2f}m")
            # Return steering direction: -1 for left turn, 1 for right turn, 0 for stop
            if min_front_distance < stop_dist: # Use the new stop_distance parameter
                self.get_logger().warn(f"Obstacle too close ({min_front_distance:.2f}m < {stop_dist:.2f}m). Forcing stop.")
                return {'action': 'stop'}
            elif left_obstacle_dist < right_obstacle_dist: # Obstacle more to the left, turn right
                return {'action': 'turn_right'}
            else: # Obstacle more to the right, turn left
                return {'action': 'turn_left'}
        
        return None # No obstacle detected within range

    def track_trajectory(self):
        cmd = Twist()
        # If no current pose or trajectory, publish zero velocities and return
        if self.current_pose is None or not self.trajectory:
            self.publisher.publish(cmd)
            return

        # Get controller parameters
        kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        goal_tolerance_distance = self.get_parameter('goal_tolerance_distance').get_parameter_value().double_value
        goal_tolerance_angle = self.get_parameter('goal_tolerance_angle').get_parameter_value().double_value

        # Get obstacle avoidance parameters
        avoidance_angular_speed = self.get_parameter('avoidance_angular_speed').get_parameter_value().double_value
        avoidance_linear_speed_reduction = self.get_parameter('avoidance_linear_speed_reduction').get_parameter_value().double_value
        stop_dist = self.get_parameter('stop_distance').get_parameter_value().double_value # Get the new stop_distance parameter

        # --- Obstacle Avoidance Logic ---
        # Check for obstacles first. If detected, override trajectory tracking.
        obstacle_action = self.check_for_obstacles()
        if obstacle_action:
            self.get_logger().warn(f"Executing obstacle avoidance: {obstacle_action['action']}")
            if obstacle_action['action'] == 'stop':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            elif obstacle_action['action'] == 'turn_right':
                cmd.linear.x = max_linear_velocity * avoidance_linear_speed_reduction
                cmd.angular.z = -avoidance_angular_speed # Negative for right turn
            elif obstacle_action['action'] == 'turn_left':
                cmd.linear.x = max_linear_velocity * avoidance_linear_speed_reduction
                cmd.angular.z = avoidance_angular_speed # Positive for left turn
            
            self.publisher.publish(cmd)
            return # Skip the rest of the function (trajectory tracking) if avoiding obstacle

        # --- Trajectory Tracking Logic (only executes if no obstacle detected) ---
        current_time_ns = self.get_clock().now().nanoseconds
        target_pose = self.find_target_pose_on_trajectory(current_time_ns)

        if target_pose is None:
            self.get_logger().info("End of trajectory or no target pose found. Stopping robot.")
            self.publisher.publish(cmd)
            return

        # Calculate errors for P-controller
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance_error = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        current_heading = self.get_current_robot_yaw()

        heading_error = desired_heading - current_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error)) # Normalize angle

        # Calculate linear and angular velocities based on P-controller
        linear_vel = kp_linear * distance_error
        angular_vel = kp_angular * heading_error

        # Clamp velocities to max values
        cmd.linear.x = min(max(linear_vel, -max_linear_velocity), max_linear_velocity)
        cmd.angular.z = min(max(angular_vel, -max_angular_velocity), max_angular_velocity)

        # Optional: Uncomment these lines for debugging controller behavior
        # self.get_logger().info(f"--- track_trajectory ---")
        # self.get_logger().info(f"Robot: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, {current_heading:.2f})")
        # self.get_logger().info(f"Target: ({target_pose.position.x:.2f}, {target_pose.position.y:.2f})")
        # self.get_logger().info(f"Errors: Dist={distance_error:.2f}, Heading={heading_error:.2f}")
        # self.get_logger().info(f"Cmd: Lin={cmd.linear.x:.2f}, Ang={cmd.angular.z:.2f}")

        # Check if robot has reached the end of the trajectory
        last_point_pose = self.trajectory[-1].pose
        final_dx = last_point_pose.position.x - self.current_pose.position.x
        final_dy = last_point_pose.position.y - self.current_pose.position.y
        final_distance = math.hypot(final_dx, final_dy)

        if final_distance < goal_tolerance_distance:
            self.get_logger().info("Reached final goal. Stopping robot.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.trajectory = [] # Clear trajectory to stop further tracking
            self.trajectory_start_time = None

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

