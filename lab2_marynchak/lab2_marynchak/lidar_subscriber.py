#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):

    def __init__(self):

        super().__init__('lidar_subscriber')

        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )

        self.get_logger().info("Lidar subscriber started")

    def lidar_callback(self, msg):

        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]

        if len(valid) > 0:

            min_dist = min(valid)
            avg_dist = sum(valid)/len(valid)

            self.get_logger().info(
                f"Min distance: {min_dist:.2f}  Avg: {avg_dist:.2f}"
            )

            if min_dist < 0.8:
                self.get_logger().warn("Obstacle detected!")


def main(args=None):

    rclpy.init(args=args)

    node = LidarSubscriber()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()