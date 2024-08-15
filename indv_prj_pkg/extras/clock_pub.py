#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(Time, 'clock', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Time()
        msg = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing current time: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    clock_publisher = ClockPublisher()
    try:
        rclpy.spin(clock_publisher)
    except KeyboardInterrupt:
        pass
    clock_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

