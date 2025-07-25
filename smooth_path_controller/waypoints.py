# smooth_path_controller/raw_waypoints_publisher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
# Import Time message type
from builtin_interfaces.msg import Time # <--- ADD THIS LINE

class WaypointsPublisher(Node):
    def __init__(self):
        super().__init__('raw_waypoints_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/raw_waypoints', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.publish_waypoints)

    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.frame_id = "odom"
        # Set the timestamp to the current ROS time
        now = self.get_clock().now()
        msg.header.stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1]) # <--- ADD THIS LINE

        def create_pose(x, y):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0
            return pose

        #waypoints = [(0.0, 0.0), (1.0, 3.0), (2.0, 0.0), (1.0, 2.0)]
        waypoints = [
		    (0.0, 0.0),
		    (2.0, 1.0),
		    (1.0, 3.0),
		    (3.0, 4.0),
		    (4.0, 2.0),
		    (6.0, 3.0),
		    (5.0, 0.0),
		    (7.0, -1.0),
		    (8.0, 1.0)
		]
        msg.poses = [create_pose(x, y) for x, y in waypoints]

        self.publisher_.publish(msg)
        self.get_logger().info('Published raw waypoints')
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
