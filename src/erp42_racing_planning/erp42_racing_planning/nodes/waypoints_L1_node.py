import csv
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class WaypointLoader(Node):
    def __init__(self):
        super().__init__('waypoints_node')

        self.declare_parameter('csv_name', 'fix_utm_v2.csv')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('lane_prefix', 'L1')
        self.declare_parameter('origin_lane_prefix', 'L1')

        csv_name = self.get_parameter('csv_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.lane_prefix = self.get_parameter('lane_prefix').value
        self.origin_lane_prefix = self.get_parameter('origin_lane_prefix').value

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Path, 'waypoints', path_qos)
        self.publish_once_timer = None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        share_directory = get_package_share_directory('erp42_racing_planning')
        csv_path = os.path.join(share_directory, 'resource', csv_name)
        self._load_waypoints(csv_path)

        self.publish_once_timer = self.create_timer(0.1, self.publish_path_once)

    def _select_columns(self, fieldnames):
        if not fieldnames:
            raise ValueError('CSV has no header')

        if 'UTM_X' in fieldnames and 'UTM_Y' in fieldnames:
            path_x_col = 'UTM_X'
            path_y_col = 'UTM_Y'
        else:
            path_x_col = f'{self.lane_prefix}_UTM_X'
            path_y_col = f'{self.lane_prefix}_UTM_Y'

        if 'UTM_X' in fieldnames and 'UTM_Y' in fieldnames:
            origin_x_col = 'UTM_X'
            origin_y_col = 'UTM_Y'
        else:
            origin_x_col = f'{self.origin_lane_prefix}_UTM_X'
            origin_y_col = f'{self.origin_lane_prefix}_UTM_Y'

        missing = [
            col for col in (path_x_col, path_y_col, origin_x_col, origin_y_col)
            if col not in fieldnames
        ]
        if missing:
            raise KeyError(f'Missing CSV columns: {missing}')

        return path_x_col, path_y_col, origin_x_col, origin_y_col

    def _load_waypoints(self, csv_path):
        origin_x = None
        origin_y = None

        try:
            with open(csv_path, newline='') as f:
                reader = csv.DictReader(f)
                path_x_col, path_y_col, origin_x_col, origin_y_col = self._select_columns(
                    reader.fieldnames
                )

                for row in reader:
                    if origin_x is None:
                        origin_x = float(row[origin_x_col])
                        origin_y = float(row[origin_y_col])

                    utm_x = float(row[path_x_col])
                    utm_y = float(row[path_y_col])

                    pose = PoseStamped()
                    pose.header.frame_id = self.frame_id
                    pose.pose.position.x = utm_x - origin_x
                    pose.pose.position.y = utm_y - origin_y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    self.path_msg.poses.append(pose)

            self.get_logger().info(
                f'Loaded {len(self.path_msg.poses)} waypoints from CSV: {csv_path} '
                f'(lane={self.lane_prefix}, origin_lane={self.origin_lane_prefix})'
            )
        except FileNotFoundError:
            self.get_logger().error(f'CSV file not found at: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')

    def publish_path_once(self):
        if self.path_msg.poses:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.path_msg)
            self.get_logger().info('Published waypoints once with TRANSIENT_LOCAL durability.')
            self.publish_once_timer.cancel()
            self.destroy_timer(self.publish_once_timer)
            self.publish_once_timer = None
        else:
            self.get_logger().warn('No waypoints loaded, not publishing path.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
