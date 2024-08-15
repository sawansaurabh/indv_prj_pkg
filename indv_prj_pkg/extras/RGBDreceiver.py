#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import socket

class RGBDReceiver(Node):
    def __init__(self):
        super().__init__('rgbd_receiver')
        self.publisher_ = self.create_publisher(Image, 'rgb_image', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))
        self.listen_for_data()

    def listen_for_data(self):
        while rclpy.ok():
            data, _ = self.sock.recvfrom(65507)
            if not data:
                continue
            self.publish_rgb_image(data)

    def publish_rgb_image(self, data):
        # Print the size of the received data
        print(f"Received data size: {len(data)}")

        # Decode image data
        image = self.decode_image_data(data)
        if image is not None:
            ros_image = self.convert_to_ros_image(image, encoding='rgb8')
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().error("Failed to decode image data")

    def decode_image_data(self, data):
        try:
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().error("Decoded image is None")
            return image
        except Exception as e:
            self.get_logger().error(f"Failed to decode image data: {e}")
            return None

    def convert_to_ros_image(self, cv_image, encoding):
        ros_image = Image()
        ros_image.height, ros_image.width = cv_image.shape[:2]
        ros_image.encoding = encoding
        ros_image.data = cv_image.tobytes()
        ros_image.step = len(ros_image.data) // ros_image.height
        return ros_image

def main(args=None):
    rclpy.init(args=args)
    node = RGBDReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
