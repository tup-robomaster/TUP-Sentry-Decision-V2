#!/usr/bin/env python
from ast import arg
import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import Uart as UartMsg
import random


class Lier(Node):

    def __init__(self):
        super().__init__('fake_msg_publisher')
        self.publisher_Uart = self.create_publisher(
            UartMsg, '/uart', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.make_fake()
        self.publisher_Uart.publish(self.uart_msg)
        self.get_logger().info("Publish Fake Msgs")

    def make_fake(self):
        self.uart_msg = UartMsg()
        self.uart_msg.remain_time = 419
        self.uart_msg.outpost_hp = 1000
        self.uart_msg.sentry_hp = 600
        self.uart_msg.game_stage = 4
        self.uart_msg.header.stamp = self.get_clock().now().to_msg()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Lier()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
