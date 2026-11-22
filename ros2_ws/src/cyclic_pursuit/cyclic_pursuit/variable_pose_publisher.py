"""Synthetic PoseStamped publishers for isolated, non-flight topic tests."""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GradualPosePublisher(Node):
    def __init__(self):
        super().__init__('gradual_pose_publisher')
        # Match the three example Crazyflie IDs in config/cyclic.yaml.
        self.start_time = time.monotonic()
        self.initial_positions = {
            'cf231': (4.0, 6.0),
            'cf123': (5.0, 6.0),
            'cf232': (6.0, 6.0),
        }
        self.publishers = {
            name: self.create_publisher(PoseStamped, f'/{name}/pose', 10)
            for name in self.initial_positions
        }
        self.timer = self.create_timer(0.05, self.publish_poses)

    def publish_poses(self):
        elapsed = time.monotonic() - self.start_time
        for name, (x0, y0) in self.initial_positions.items():
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = x0 + elapsed * 0.1
            msg.pose.position.y = y0 + elapsed * 0.05
            msg.pose.orientation.w = 1.0  # Unit quaternion (zero yaw).
            self.publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = GradualPosePublisher()
    try:
        rclpy.spin(publisher)
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
