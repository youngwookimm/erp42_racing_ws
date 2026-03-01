#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker


class VehiclePoseVizNode(Node):
    def __init__(self):
        super().__init__('vehicle_pose_viz_node')

        self.declare_parameter('pose_topic', '/utm_tm')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('output_pose_topic', '/vehicle_pose')
        self.declare_parameter('output_marker_topic', '/vehicle_pose_origin_marker')
        self.declare_parameter('output_frame_id', 'map')
        self.declare_parameter('origin_marker_scale', 1.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.output_pose_topic = self.get_parameter('output_pose_topic').value
        self.output_marker_topic = self.get_parameter('output_marker_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.origin_marker_scale = self.get_parameter('origin_marker_scale').value

        self.latest_pose = None
        self.latest_orientation = None
        self.warned_invalid_imu = False

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.output_marker_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )

        self.get_logger().info(
            f'VehiclePoseVizNode ready: pose={self.pose_topic}, imu={self.imu_topic}, '
            f'out={self.output_pose_topic}, marker={self.output_marker_topic}, '
            f'frame={self.output_frame_id}'
        )

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.publish_vehicle_pose(msg.header.stamp)

    def imu_callback(self, msg):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            if not self.warned_invalid_imu:
                self.get_logger().warn('Received invalid IMU orientation quaternion; skipping publish.')
                self.warned_invalid_imu = True
            return

        self.warned_invalid_imu = False
        self.latest_orientation = q
        self.publish_vehicle_pose(msg.header.stamp)

    def publish_vehicle_pose(self, stamp):
        if self.latest_pose is None or self.latest_orientation is None:
            return

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame_id
        msg.pose.position.x = self.latest_pose.pose.position.x
        msg.pose.position.y = self.latest_pose.pose.position.y
        msg.pose.position.z = self.latest_pose.pose.position.z
        msg.pose.orientation = self.latest_orientation

        self.pose_pub.publish(msg)
        self.publish_origin_marker(msg)

    def publish_origin_marker(self, msg):
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = self.output_frame_id
        marker.ns = 'vehicle_pose_origin'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.pose.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.origin_marker_scale
        marker.scale.y = self.origin_marker_scale
        marker.scale.z = self.origin_marker_scale
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.1
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VehiclePoseVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
