#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

import math
import time


class Planner(Node):

    def __init__(self):
        super().__init__('planner')

        # Subscribers
        self.goal_sub = self.create_subscription(
            Pose2D,
            '/goal',
            self.goal_callback,
            10
        )

        self.state_sub = self.create_subscription(
            Pose2D,
            '/robot_state',
            self.state_callback,
            10
        )

        # Publishers
        self.planner_goal_pub = self.create_publisher(
            Pose2D,
            '/planner_goal',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/planner_status',
            10
        )

        # Internal state
        self.global_goal = None
        self.current_state = None

        self.last_distance = None
        self.last_progress_time = time.time()

        self.goal_tolerance = 0.15
        self.subgoal_fraction = 0.4

        self.timer = self.create_timer(0.2, self.planner_loop)

        self.get_logger().info("Planner Node Started")

    # ----------------------------
    def goal_callback(self, msg):
        self.global_goal = msg
        self.get_logger().info("New Global Goal Received")

    # ----------------------------
    def state_callback(self, msg):
        self.current_state = msg

    # ----------------------------
    def planner_loop(self):

        if self.global_goal is None or self.current_state is None:
            return

        dx = self.global_goal.x - self.current_state.x
        dy = self.global_goal.y - self.current_state.y

        dist = math.sqrt(dx**2 + dy**2)

        # Check goal reached
        if dist < self.goal_tolerance:

            status = String()
            status.data = "GOAL_REACHED"
            self.status_pub.publish(status)

            self.publish_goal(self.global_goal)
            return

        # Detect progress
        if self.last_distance is not None:

            if abs(dist - self.last_distance) > 0.01:
                self.last_progress_time = time.time()

            # If stuck for 5 seconds
            if time.time() - self.last_progress_time > 5.0:
                self.get_logger().warn("Robot appears stuck. Replanning...")
                self.perturb_goal()
                self.last_progress_time = time.time()

        self.last_distance = dist

        # Generate subgoal
        subgoal = Pose2D()
        subgoal.x = self.current_state.x + self.subgoal_fraction * dx
        subgoal.y = self.current_state.y + self.subgoal_fraction * dy
        subgoal.theta = 0.0

        self.publish_goal(subgoal)

        status = String()
        status.data = "MOVING_TO_SUBGOAL"
        self.status_pub.publish(status)

    # ----------------------------
    def publish_goal(self, goal):

        self.planner_goal_pub.publish(goal)

    # ----------------------------
    def perturb_goal(self):

        if self.global_goal is None:
            return

        perturbed = Pose2D()
        perturbed.x = self.global_goal.x + 0.2
        perturbed.y = self.global_goal.y - 0.2
        perturbed.theta = 0.0

        self.planner_goal_pub.publish(perturbed)


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()