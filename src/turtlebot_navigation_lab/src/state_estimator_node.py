#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose2D

import math
from tf_transformations import euler_from_quaternion


class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10
        )

        # Publishers
        self.state_pub = self.create_publisher(
            Pose2D,
            '/robot_state',
            10
        )

        self.goal_pub = self.create_publisher(
            Pose2D,
            '/goal',
            10
        )

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.goal_x = None
        self.goal_y = None

        # Timer to publish state at fixed rate
        self.timer = self.create_timer(0.05, self.publish_state)  # 20 Hz

        self.get_logger().info("State Estimator Node Started")

    # ----------------------------------
    # ODOM CALLBACK
    # ----------------------------------
    def odom_callback(self, msg: Odometry):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        _, _, self.theta = euler_from_quaternion(quaternion)

    # ----------------------------------
    # GOAL CALLBACK (RViz clicked point)
    # ----------------------------------
    def goal_callback(self, msg: PointStamped):

        self.goal_x = msg.point.x
        self.goal_y = msg.point.y

        goal_msg = Pose2D()
        goal_msg.x = self.goal_x
        goal_msg.y = self.goal_y
        goal_msg.theta = 0.0

        self.goal_pub.publish(goal_msg)

        self.get_logger().info(
            f"New Goal Received: x={self.goal_x:.2f}, y={self.goal_y:.2f}"
        )

    # ----------------------------------
    # PUBLISH STATE
    # ----------------------------------
    def publish_state(self):

        state_msg = Pose2D()
        state_msg.x = self.x
        state_msg.y = self.y
        state_msg.theta = self.theta

        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()