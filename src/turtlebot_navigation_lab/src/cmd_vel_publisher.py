#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')

        # Crear publisher al tópico /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para publicar cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.get_logger().info("Nodo cmd_vel_publisher iniciado")

    def publish_cmd_vel(self):
        msg = Twist()

        # Velocidad lineal (m/s)
        msg.linear.x = 0.2
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Velocidad angular (rad/s)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()