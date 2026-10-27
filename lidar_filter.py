# lidar_filter_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )

        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

    def callback(self, msg):
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header

        filtered_msg.angle_min = 0.0
        filtered_msg.angle_max = math.pi
        filtered_msg.angle_increment = msg.angle_increment

        start_index = int((0.0 - msg.angle_min) / msg.angle_increment)
        end_index = int((math.pi - msg.angle_min) / msg.angle_increment)

        filtered_msg.ranges = msg.ranges[start_index:end_index]
        filtered_msg.intensities = msg.intensities[start_index:end_index]

        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        self.pub.publish(filtered_msg)


def main():
    rclpy.init()
    node = LidarFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()