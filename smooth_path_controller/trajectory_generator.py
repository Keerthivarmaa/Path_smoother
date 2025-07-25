# smooth_path_controller/trajectory_generator.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
# Import the Time message type
from builtin_interfaces.msg import Time # <--- ADD THIS LINE

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.subscription = self.create_subscription(
            PoseArray,
            'smoothed_path',
            self.smoothed_path_callback,
            10)
        self.publisher = self.create_publisher(Path, 'trajectory', 10)
        self.declare_parameter('desired_linear_velocity', 0.5) # meters/second
        self.get_logger().info("Trajectory Generator Initialized")

    def smoothed_path_callback(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().warn("Received empty smoothed path.")
            return

        desired_velocity = self.get_parameter('desired_linear_velocity').get_parameter_value().double_value
        self.get_logger().info(f"Generating trajectory with desired velocity: {desired_velocity} m/s")

        path_msg = Path()
        path_msg.header = msg.header # Keep the frame_id from the smoothed path

        current_time_offset = 0.0 # Time elapsed along the path
        last_pose = None

        for i, pose in enumerate(msg.poses):
            if last_pose is not None:
                # Calculate distance from last point
                dx = pose.position.x - last_pose.position.x
                dy = pose.position.y - last_pose.position.y
                segment_distance = math.hypot(dx, dy)

                # Calculate time increment for this segment
                if desired_velocity > 0:
                    time_increment = segment_distance / desired_velocity
                else:
                    time_increment = 0.0 # Avoid division by zero if velocity is 0

                current_time_offset += time_increment

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = path_msg.header.frame_id
            
            # Calculate the target time for this pose
            target_rclpy_time = self.get_clock().now() + rclpy.duration.Duration(seconds=current_time_offset)
            
            # Convert rclpy.time.Time to builtin_interfaces.msg.Time
            pose_stamped.header.stamp = Time(sec=target_rclpy_time.seconds_nanoseconds()[0],
                                             nanosec=target_rclpy_time.seconds_nanoseconds()[1])
            
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)

            last_pose = pose # Store current pose for next iteration's distance calculation

        self.publisher.publish(path_msg)
        self.get_logger().info(f"Published time-parameterized trajectory with {len(path_msg.poses)} poses.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node() # Use destroy_node() for proper cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
