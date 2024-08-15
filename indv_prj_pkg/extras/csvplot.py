#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

class OdomToCSV(Node):
    def __init__(self):
        super().__init__('odom_to_csv_node')
        
        # Get the output file name from the parameter server or use a default
        self.declare_parameter('csv_file', 'odom_data.csv')
        self.csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        
        # Open the CSV file in write mode
        self.file = open(self.csv_file, 'w')
        self.writer = csv.writer(self.file)
        
        # Write the header row
        self.writer.writerow(['Time', 'x', 'y', 'z'])
        
        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/unity_odom',
            self.odom_callback,
            10  # QoS history depth
        )
        
        self.get_logger().info(f"Writing odom data to {self.csv_file}")

    def odom_callback(self, msg):
        # Extract position data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Get the current time
        time = self.get_clock().now().to_msg().sec
        
        # Write the data to the CSV file
        self.writer.writerow([time, x, y, z])
    
    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        self.get_logger().info("Shutting down, closing CSV file.")
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the OdomToCSV class
    node = OdomToCSV()
    
    # Run the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
