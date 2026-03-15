import math

import rclpy
from rclpy.node import Node

from erp42_racing_msgs.msg import ControlCommand
from obstacle_tracking.msg import Track2DArray


class AebNode(Node):
    def __init__(self):
        super().__init__('aeb_node')

        self.declare_parameter('input_topic', '/tracked/objects')
        self.declare_parameter('output_topic', '/vehicle_cmd_gate/aeb_cmd')
        self.declare_parameter('partial_ttc_sec', 1.4)
        self.declare_parameter('full_ttc_sec', 0.9)
        self.declare_parameter('partial_brake', 50)
        self.declare_parameter('full_brake', 100)
        self.declare_parameter('ego_width_m', 1.16)
        self.declare_parameter('ego_front_offset_m', 1.09)
        self.declare_parameter('lateral_margin_m', 1.0)
        self.declare_parameter('min_closing_speed_mps', 0.1)
        self.declare_parameter('hold_distance_m', 2.5)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.partial_ttc_sec = self.get_parameter('partial_ttc_sec').value
        self.full_ttc_sec = self.get_parameter('full_ttc_sec').value
        self.partial_brake = self.get_parameter('partial_brake').value
        self.full_brake = self.get_parameter('full_brake').value
        self.ego_width_m = self.get_parameter('ego_width_m').value
        self.ego_front_offset_m = self.get_parameter('ego_front_offset_m').value
        self.lateral_margin_m = self.get_parameter('lateral_margin_m').value
        self.min_closing_speed_mps = self.get_parameter('min_closing_speed_mps').value
        self.hold_distance_m = self.get_parameter('hold_distance_m').value

        self.hold_active = False
        self.hold_brake = 0
        self.hold_track_id = -1

        self.sub_tracks = self.create_subscription(
            Track2DArray,
            self.input_topic,
            self.tracks_callback,
            10,
        )
        self.pub_cmd = self.create_publisher(ControlCommand, self.output_topic, 10)

        self.get_logger().info(
            f'AEB Ready: in={self.input_topic}, out={self.output_topic}, '
            f'partial_ttc={self.partial_ttc_sec:.2f}, full_ttc={self.full_ttc_sec:.2f}, '
            f'hold_distance={self.hold_distance_m:.2f}'
        )

    def tracks_callback(self, msg):
        best_ttc = math.inf
        best_gap = math.inf
        best_track_id = -1

        for track in msg.tracks:
            if track.pose.position.x <= 0.0:
                continue

            half_object_width = max(0.05, 0.5 * float(track.width))
            lateral_limit = 0.5 * self.ego_width_m + half_object_width + self.lateral_margin_m
            if abs(track.pose.position.y) > lateral_limit:
                continue

            gap = (
                float(track.pose.position.x)
                - max(0.1, 0.5 * float(track.length))
                - self.ego_front_offset_m
            )

            if gap < best_gap:
                best_gap = gap
                best_track_id = track.track_id

            if gap <= 0.0:
                best_ttc = 0.0
                break

            closing_speed = max(-float(track.twist.linear.x), 0.0)
            if closing_speed < self.min_closing_speed_mps:
                continue

            ttc = gap / closing_speed
            if ttc < best_ttc:
                best_ttc = ttc
                best_track_id = track.track_id

        cmd = ControlCommand()
        cmd.speed = 0.0
        cmd.steering = 0.0
        cmd.brake = 0

        state = 'NONE'
        if best_ttc <= self.full_ttc_sec:
            cmd.brake = int(self.full_brake)
            state = 'FULL'
        elif best_ttc <= self.partial_ttc_sec:
            cmd.brake = int(self.partial_brake)
            state = 'PARTIAL'
        elif self.hold_active and best_gap <= self.hold_distance_m:
            cmd.brake = int(self.hold_brake)
            state = 'HOLD'

        if state in ('FULL', 'PARTIAL'):
            self.hold_active = True
            self.hold_brake = cmd.brake
            self.hold_track_id = best_track_id
        elif state == 'HOLD':
            self.hold_active = True
        else:
            self.hold_active = False
            self.hold_brake = 0
            self.hold_track_id = -1

        self.pub_cmd.publish(cmd)

        if state in ('FULL', 'PARTIAL'):
            self.get_logger().warn(
                f'AEB {state}: track_id={best_track_id}, min_ttc={best_ttc:.3f}s, '
                f'gap={best_gap:.3f}m, brake={cmd.brake}'
            )
        elif state == 'HOLD':
            self.get_logger().warn(
                f'AEB HOLD: track_id={self.hold_track_id}, gap={best_gap:.3f}m, brake={cmd.brake}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
