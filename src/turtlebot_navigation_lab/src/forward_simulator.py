#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan

import math
import numpy as np


class ForwardSimulator(Node):

    def __init__(self):
        super().__init__('forward_simulator')

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

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.goal_x = None
        self.goal_y = None

        self.scan_data = None

        # Simulation parameters
        self.dt = 0.1
        self.H = 15

        # Cost weights
        self.w_goal = 5.0
        self.w_obs = 3.0
        self.w_theta = 1.0

        self.timer = self.create_timer(0.1, self.simulation_step)

        self.get_logger().info("Forward Simulator Node Started")

    # -----------------------------
    def state_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    # -----------------------------
    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y

    # -----------------------------
    def scan_callback(self, msg):
        self.scan_data = msg

    # -----------------------------
    def simulate_trajectory(self, v, omega):

        x = self.x
        y = self.y
        theta = self.theta

        min_obs_dist = float('inf')

        for _ in range(self.H):

            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += omega * self.dt

            d_obs = self.min_obstacle_distance(x, y)

            if d_obs < min_obs_dist:
                min_obs_dist = d_obs

        return x, y, theta, min_obs_dist

    # -----------------------------
    def min_obstacle_distance(self, x, y):

        if self.scan_data is None:
            return 10.0

        min_dist = float('inf')
        angle = self.scan_data.angle_min

        for r in self.scan_data.ranges:

            if r > 0.05:

                ox = self.x + r * math.cos(angle + self.theta)
                oy = self.y + r * math.sin(angle + self.theta)

                d = math.sqrt((x - ox)**2 + (y - oy)**2)

                if d < min_dist:
                    min_dist = d

            angle += self.scan_data.angle_increment

        return min_dist

    # -----------------------------
    def compute_cost(self, x, y, theta, min_obs_dist):

        d_goal = math.sqrt((x - self.goal_x)**2 +
                           (y - self.goal_y)**2)

        theta_d = math.atan2(self.goal_y - y,
                             self.goal_x - x)

        e_theta = abs(self.normalize_angle(theta_d - theta))

        if min_obs_dist < 0.2:
            return float('inf')

        cost = (self.w_goal * d_goal +
                self.w_obs * (1.0 / (min_obs_dist + 1e-3)) +
                self.w_theta * e_theta)

        return cost

    # -----------------------------
    def simulation_step(self):

        if self.goal_x is None:
            return

        candidates_v = [0.1, 0.2]
        candidates_w = [-1.0, 0.0, 1.0]

        best_cost = float('inf')
        best_cmd = (0.0, 0.0)

        for v in candidates_v:
            for w in candidates_w:

                x_f, y_f, theta_f, min_obs = \
                    self.simulate_trajectory(v, w)

                cost = self.compute_cost(x_f, y_f,
                                         theta_f, min_obs)

                if cost < best_cost:
                    best_cost = cost
                    best_cmd = (v, w)

        cmd = Twist()
        cmd.linear.x = best_cmd[0]
        cmd.angular.z = best_cmd[1]

        self.cmd_pub.publish(cmd)

    # -----------------------------
    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ForwardSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()