#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan

import math
import numpy as np


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # Subscribers
        self.state_sub = self.create_subscription(
            Pose2D,
            '/robot_state',
            self.state_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            Pose2D,
            '/goal',
            self.goal_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.goal_x = None
        self.goal_y = None

        self.scan_data = None

        # Gains
        self.k_att = 1.0
        self.k_rep = 0.8
        self.d0 = 0.6

        # LQR-like gains
        self.k_theta = 2.0
        self.k_damp = 0.5

        self.prev_omega = 0.0

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Controller Node Started")

    # ---------------------------
    def state_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    # ---------------------------
    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y

    # ---------------------------
    def scan_callback(self, msg):
        self.scan_data = msg

    # ---------------------------
    def compute_attractive_force(self):

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        return self.k_att * dx, self.k_att * dy

    # ---------------------------
    def compute_repulsive_force(self):

        if self.scan_data is None:
            return 0.0, 0.0

        fx_rep = 0.0
        fy_rep = 0.0

        angle = self.scan_data.angle_min

        for r in self.scan_data.ranges:

            if r < self.d0 and r > 0.05:

                force_mag = self.k_rep * (1.0/r - 1.0/self.d0) / (r**2)

                # Obstacle position in robot frame
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)

                # Convert to world frame
                ox_w = self.x + ox * math.cos(self.theta) - oy * math.sin(self.theta)
                oy_w = self.y + ox * math.sin(self.theta) + oy * math.cos(self.theta)

                dx = self.x - ox_w
                dy = self.y - oy_w

                norm = math.sqrt(dx**2 + dy**2)

                if norm > 0.001:
                    fx_rep += force_mag * dx / norm
                    fy_rep += force_mag * dy / norm

            angle += self.scan_data.angle_increment

        return fx_rep, fy_rep

    # ---------------------------
    def control_loop(self):

        if self.goal_x is None:
            return

        # Attractive
        fx_att, fy_att = self.compute_attractive_force()

        # Repulsive
        fx_rep, fy_rep = self.compute_repulsive_force()

        # Total force
        fx = fx_att + fx_rep
        fy = fy_att + fy_rep

        # Desired heading
        theta_d = math.atan2(fy, fx)

        # Heading error
        e_theta = self.normalize_angle(theta_d - self.theta)

        # Linear velocity
        v = 0.2

        # LQR-like angular velocity
        omega = self.k_theta * e_theta - self.k_damp * self.prev_omega

        self.prev_omega = omega

        # Stop near goal
        dist_goal = math.sqrt((self.goal_x - self.x)**2 +
                              (self.goal_y - self.y)**2)

        if dist_goal < 0.1:
            v = 0.0
            omega = 0.0

        # Publish
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega

        self.cmd_pub.publish(cmd)

    # ---------------------------
    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi

        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()