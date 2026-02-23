#!/usr/bin/env python3
import csv
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class UtmOriginShiftNode(Node):
    def __init__(self):
        super().__init__('utm_origin_shift_node')

        self.declare_parameter('origin_package', 'erp42_racing_planning')
        self.declare_parameter('origin_csv', 'utm_0218.csv')
        self.declare_parameter('origin_x_column', 'UTM_X')
        self.declare_parameter('origin_y_column', 'UTM_Y')
        self.declare_parameter('input_topic', '/utm')
        self.declare_parameter('output_topic', '/utm_tm')
        self.declare_parameter('output_frame_id', 'map')

        origin_package = self.get_parameter('origin_package').value
        origin_csv = self.get_parameter('origin_csv').value
        origin_x_column = self.get_parameter('origin_x_column').value
        origin_y_column = self.get_parameter('origin_y_column').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.origin_x, self.origin_y = self._load_origin(
            origin_package, origin_csv, origin_x_column, origin_y_column
        )

        self.pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.sub = self.create_subscription(PoseStamped, input_topic, self.utm_callback, 10)

        self.get_logger().info(
            f'UtmOriginShiftNode ready: in={input_topic}, out={output_topic}, '
            f'origin=({self.origin_x:.3f}, {self.origin_y:.3f})'
        )

    def _load_origin(self, package_name, csv_name, x_column, y_column):
        try:
            share_dir = get_package_share_directory(package_name)
        except Exception as exc:
            self.get_logger().error(f'Failed to find package {package_name}: {exc}')
            raise

        csv_path = os.path.join(share_dir, 'resource', csv_name)

        try:
            with open(csv_path, newline='') as f:
                reader = csv.DictReader(f)
                first_row = next(reader)
        except StopIteration:
            self.get_logger().error(f'CSV is empty: {csv_path}')
            raise
        except Exception as exc:
            self.get_logger().error(f'Failed to read origin csv {csv_path}: {exc}')
            raise

        try:
            origin_x = float(first_row[x_column])
            origin_y = float(first_row[y_column])
        except KeyError as exc:
            self.get_logger().error(f'Invalid CSV columns in {csv_path}: {exc}')
            raise

        return origin_x, origin_y

    def utm_callback(self, msg):
        shifted = PoseStamped()
        shifted.header = msg.header
        shifted.header.frame_id = self.output_frame_id
        shifted.pose.position.x = msg.pose.position.x - self.origin_x
        shifted.pose.position.y = msg.pose.position.y - self.origin_y
        shifted.pose.position.z = msg.pose.position.z
        shifted.pose.orientation = msg.pose.orientation

        self.pub.publish(shifted)


def main(args=None):
    rclpy.init(args=args)
    node = UtmOriginShiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
