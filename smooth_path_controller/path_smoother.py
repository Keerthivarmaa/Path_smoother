# smooth_path_controller/path_smoother.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from scipy.interpolate import CubicSpline

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')
        self.publisher_ = self.create_publisher(PoseArray, 'smoothed_path', 10)
        self.subscription_ = self.create_subscription(
            PoseArray,
            'raw_waypoints',
            self.raw_callback,
            10
        )
        self.get_logger().info("Path Smoother Initialized and waiting for raw waypoints...")

    def raw_callback(self, msg: PoseArray):
        self.get_logger().info(f"Received {len(msg.poses)} raw waypoints.")
        if len(msg.poses) < 3:
            self.get_logger().warn("Need at least 3 waypoints to smooth.")
            return

        # Convert PoseArray to numpy array
        waypoints = np.array([[pose.position.x, pose.position.y] for pose in msg.poses])
        t = np.linspace(0, 1, len(waypoints))
        cs_x = CubicSpline(t, waypoints[:, 0])
        cs_y = CubicSpline(t, waypoints[:, 1])
        ts = np.linspace(0, 1, 50)

        # Create smoothed PoseArray
        path = PoseArray()
        path.header = msg.header
        for ti in ts:
            pose = Pose()
            pose.position.x = float(cs_x(ti))
            pose.position.y = float(cs_y(ti))
            path.poses.append(pose)

        self.publisher_.publish(path)
        self.get_logger().info("Published smoothed path")

def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    rclpy.spin(node)
    rclpy.shutdown()

