import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

from erp42_racing_msgs.msg import ControlCommand
from erp42_racing_msgs.srv import ModeCommand


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.L = 1.2

        self.declare_parameter('Ld_min', 2.0)
        self.declare_parameter('Ld_max', 5.0)
        self.declare_parameter('G_v', 1.2)
        self.declare_parameter('G_k', 0.5)
        self.declare_parameter('V_max', 1.5)
        self.declare_parameter('V_min', 1.0)
        self.declare_parameter('G_avg', 1.48)
        self.declare_parameter('D_v_look', 10.0)
        self.declare_parameter('max_steering_deg', 20.0)
        self.declare_parameter('fallback_min_dist_threshold', 1.5)
        self.declare_parameter('curvature_window_distance', 0.15)

        self.declare_parameter('path_topic', '/L1/waypoints')
        self.declare_parameter('pose_topic', '/utm_tm')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('vel_topic', '/vel')

        self.Ld_min = self.get_parameter('Ld_min').get_parameter_value().double_value
        self.Ld_max = self.get_parameter('Ld_max').get_parameter_value().double_value
        self.G_v = self.get_parameter('G_v').get_parameter_value().double_value
        self.G_k = self.get_parameter('G_k').get_parameter_value().double_value
        self.V_max = self.get_parameter('V_max').get_parameter_value().double_value
        self.V_min = self.get_parameter('V_min').get_parameter_value().double_value
        self.G_avg = self.get_parameter('G_avg').get_parameter_value().double_value
        self.D_v_look = self.get_parameter('D_v_look').get_parameter_value().double_value
        self.max_steering_deg = (
            self.get_parameter('max_steering_deg').get_parameter_value().double_value
        )
        self.fallback_min_dist_threshold = (
            self.get_parameter('fallback_min_dist_threshold').get_parameter_value().double_value
        )
        self.curvature_window_distance = (
            self.get_parameter('curvature_window_distance').get_parameter_value().double_value
        )
        self.max_steering_rad = math.radians(self.max_steering_deg)

        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.vel_topic = self.get_parameter('vel_topic').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.v_est = 0.0
        self.path = []
        self.path_curvatures = []
        self.path_segment_lengths = []
        self.target_idx = 0
        self.path_is_closed = False

        self.have_pose = False
        self.have_yaw = False
        self.have_vel = False

        self.mode_client = self.create_client(ModeCommand, '/erp42_racing/mode_command')
        self._init_vehicle()

        self.pub_cmd = self.create_publisher(ControlCommand, '/erp42_racing/control_command', 10)
        self.pub_target_marker = self.create_publisher(Marker, '/target_waypoint_marker', 10)
        self.pub_lookahead_marker = self.create_publisher(Marker, '/lookahead_waypoint_marker', 10)
        self.pub_virtual_target_marker = self.create_publisher(Marker, '/virtual_target_point_marker', 10)
        self.pub_kappa_tilda = self.create_publisher(Float64, '/kappa_tilda', 10)
        self.pub_kappa = self.create_publisher(Float64, '/kappa', 10)
        self.pub_lateral_error = self.create_publisher(Float64, '/lateral_error', 10)

        self.sub_path = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.sub_pose = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.sub_vel = self.create_subscription(
            TwistWithCovarianceStamped,
            self.vel_topic,
            self.vel_callback,
            10,
        )

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(
            f'Pure Pursuit Ready: path={self.path_topic}, pose={self.pose_topic}, '
            f'imu={self.imu_topic}, vel={self.vel_topic}'
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'Ld_min':
                self.Ld_min = param.value
            elif param.name == 'Ld_max':
                self.Ld_max = param.value
            elif param.name == 'G_v':
                self.G_v = param.value
            elif param.name == 'G_k':
                self.G_k = param.value
            elif param.name == 'V_max':
                self.V_max = param.value
            elif param.name == 'V_min':
                self.V_min = param.value
            elif param.name == 'G_avg':
                self.G_avg = param.value
            elif param.name == 'D_v_look':
                self.D_v_look = param.value
            elif param.name == 'max_steering_deg':
                self.max_steering_deg = param.value
                self.max_steering_rad = math.radians(self.max_steering_deg)
            elif param.name == 'fallback_min_dist_threshold':
                self.fallback_min_dist_threshold = param.value
            elif param.name == 'curvature_window_distance':
                self.curvature_window_distance = param.value
            elif param.name == 'path_topic':
                self.path_topic = param.value
            elif param.name == 'pose_topic':
                self.pose_topic = param.value
            elif param.name == 'imu_topic':
                self.imu_topic = param.value
            elif param.name == 'vel_topic':
                self.vel_topic = param.value

            self.get_logger().info(f'Parameter {param.name} updated to {param.value}')

        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def _init_vehicle(self):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ModeCommand Service...')

        req = ModeCommand.Request()
        req.manual_mode = False
        req.emergency_stop = False
        req.gear = 0
        self.mode_client.call_async(req)

    def get_yaw(self, q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def calculate_path_curvatures(self):
        num_waypoints = len(self.path)
        self.path_curvatures = [0.0] * num_waypoints
        self.path_segment_lengths = [0.0] * num_waypoints

        if num_waypoints < 3:
            return

        first_point = self.path[0].pose.position
        last_point = self.path[-1].pose.position
        first_last_dist = math.hypot(last_point.x - first_point.x, last_point.y - first_point.y)
        self.path_is_closed = first_last_dist < 0.5

        for i in range(num_waypoints):
            p_curr = self.path[i].pose.position
            if i < num_waypoints - 1:
                p_next = self.path[i + 1].pose.position
                self.path_segment_lengths[i] = math.hypot(
                    p_next.x - p_curr.x,
                    p_next.y - p_curr.y,
                )
            elif self.path_is_closed:
                p_next = self.path[0].pose.position
                self.path_segment_lengths[i] = math.hypot(
                    p_next.x - p_curr.x,
                    p_next.y - p_curr.y,
                )
            else:
                self.path_segment_lengths[i] = 0.0

        total_path_length = sum(self.path_segment_lengths)
        if total_path_length <= 1e-6:
            return

        window_distance = min(self.curvature_window_distance, total_path_length * 0.25)
        if window_distance <= 1e-6:
            return

        for i in range(num_waypoints):
            curr = self.path[i].pose.position
            mid_x, mid_y = self.interpolate_forward_point_along_path(i, window_distance)
            next_x, next_y = self.interpolate_forward_point_along_path(i, 2.0 * window_distance)

            v1_x = mid_x - curr.x
            v1_y = mid_y - curr.y
            v2_x = next_x - mid_x
            v2_y = next_y - mid_y
            v3_x = next_x - curr.x
            v3_y = next_y - curr.y

            a = math.hypot(v1_x, v1_y)
            b = math.hypot(v2_x, v2_y)
            c = math.hypot(v3_x, v3_y)
            cross = abs(v1_x * v2_y - v1_y * v2_x)
            denom = a * b * c

            if denom > 1e-9:
                self.path_curvatures[i] = 2.0 * cross / denom

    def interpolate_forward_point_along_path(self, start_idx, target_distance):
        num_waypoints = len(self.path)
        point = self.path[start_idx].pose.position

        if num_waypoints == 0 or target_distance <= 1e-9:
            return point.x, point.y

        remaining_distance = target_distance
        current_idx = start_idx

        for _ in range(num_waypoints):
            seg_idx = current_idx
            if seg_idx >= num_waypoints - 1 and not self.path_is_closed:
                final_point = self.path[-1].pose.position
                return final_point.x, final_point.y

            next_idx = (seg_idx + 1) % num_waypoints
            start_point = self.path[seg_idx].pose.position
            end_point = self.path[next_idx].pose.position
            seg_len = self.path_segment_lengths[seg_idx]
            advance_idx = next_idx

            if seg_len <= 1e-9:
                current_idx = advance_idx
                continue

            if remaining_distance <= seg_len:
                ratio = remaining_distance / seg_len
                x = start_point.x + ratio * (end_point.x - start_point.x)
                y = start_point.y + ratio * (end_point.y - start_point.y)
                return x, y

            remaining_distance -= seg_len
            current_idx = advance_idx

        point = self.path[current_idx].pose.position
        return point.x, point.y

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.have_pose = True

    def imu_callback(self, msg):
        self.current_yaw = self.get_yaw(msg.orientation)
        self.have_yaw = True

    def vel_callback(self, msg):
        self.v_est = msg.twist.twist.linear.x
        self.have_vel = True

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return

        self.path = msg.poses
        self.target_idx = min(self.target_idx, len(self.path) - 1)
        self.calculate_path_curvatures()

    def publish_marker(self, publisher, x, y, r, g, b, marker_id):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        publisher.publish(marker)

    def find_local_nearest_index(self):
        num_waypoints = len(self.path)
        min_dist = float('inf')
        near_idx = self.target_idx

        for i in range(self.target_idx, self.target_idx + 50):
            idx = i % num_waypoints
            d = math.hypot(
                self.path[idx].pose.position.x - self.current_x,
                self.path[idx].pose.position.y - self.current_y,
            )
            if d < min_dist:
                min_dist = d
                near_idx = idx

        return near_idx, min_dist

    def find_global_nearest_index(self):
        min_dist = float('inf')
        near_idx = self.target_idx

        for idx, pose in enumerate(self.path):
            d = math.hypot(
                pose.pose.position.x - self.current_x,
                pose.pose.position.y - self.current_y,
            )
            if d < min_dist:
                min_dist = d
                near_idx = idx

        return near_idx, min_dist

    def find_closest_waypoint_to_point(self, x, y, start_idx, search_count):
        num_waypoints = len(self.path)
        min_dist = float('inf')
        best_idx = start_idx

        for i in range(start_idx, start_idx + min(search_count, num_waypoints)):
            idx = i % num_waypoints
            d = math.hypot(
                self.path[idx].pose.position.x - x,
                self.path[idx].pose.position.y - y,
            )
            if d < min_dist:
                min_dist = d
                best_idx = idx

        return best_idx

    def find_index_at_path_distance(self, start_idx, target_distance):
        num_waypoints = len(self.path)

        if target_distance <= 0.0:
            point = self.path[start_idx].pose.position
            return start_idx, point.x, point.y

        accumulated_distance = 0.0
        current_idx = start_idx

        for step in range(num_waypoints):
            seg_idx = (start_idx + step) % num_waypoints
            next_idx = (seg_idx + 1) % num_waypoints
            seg_len = self.path_segment_lengths[seg_idx]
            start_point = self.path[seg_idx].pose.position
            end_point = self.path[next_idx].pose.position

            if seg_len <= 1e-6:
                current_idx = next_idx
                continue

            if accumulated_distance + seg_len >= target_distance:
                ratio = (target_distance - accumulated_distance) / seg_len
                x = start_point.x + ratio * (end_point.x - start_point.x)
                y = start_point.y + ratio * (end_point.y - start_point.y)
                return next_idx, x, y

            accumulated_distance += seg_len
            current_idx = next_idx

        point = self.path[current_idx].pose.position
        return current_idx, point.x, point.y

    def compute_weighted_curvature_average(self, start_idx, end_idx):
        if start_idx == end_idx:
            return self.path_curvatures[start_idx]

        num_waypoints = len(self.path)
        weighted_sum = 0.0
        total_length = 0.0
        idx = start_idx

        for _ in range(num_waypoints):
            seg_len = self.path_segment_lengths[idx]
            if seg_len > 0.0:
                weighted_sum += self.path_curvatures[idx] * seg_len
                total_length += seg_len

            idx = (idx + 1) % num_waypoints
            if idx == end_idx:
                break

        if total_length > 0.0:
            return weighted_sum / total_length

        return self.path_curvatures[start_idx]

    def control_loop(self):
        if len(self.path) < 2:
            return
        if not (self.have_pose and self.have_yaw and self.have_vel):
            return

        near_idx, min_dist = self.find_local_nearest_index()
        if min_dist > self.fallback_min_dist_threshold:
            near_idx, min_dist = self.find_global_nearest_index()

        self.target_idx = near_idx
        kappa_here = self.path_curvatures[near_idx]

        kappa_msg = Float64()
        kappa_msg.data = kappa_here
        self.pub_kappa.publish(kappa_msg)

        dx = self.current_x - self.path[near_idx].pose.position.x
        dy = self.current_y - self.path[near_idx].pose.position.y
        lateral_error = -math.sin(self.current_yaw) * dx + math.cos(self.current_yaw) * dy

        error_msg = Float64()
        error_msg.data = lateral_error
        self.pub_lateral_error.publish(error_msg)

        ld_raw = (abs(self.v_est) * self.G_v) - (kappa_here * self.G_k)
        L_d = max(self.Ld_min, min(self.Ld_max, ld_raw))

        lookahead_x = self.current_x + L_d * math.cos(self.current_yaw)
        lookahead_y = self.current_y + L_d * math.sin(self.current_yaw)
        target_idx = self.find_closest_waypoint_to_point(
            lookahead_x,
            lookahead_y,
            near_idx,
            800,
        )
        tx = self.path[target_idx].pose.position.x
        ty = self.path[target_idx].pose.position.y

        alpha = math.atan2(ty - self.current_y, tx - self.current_x) - self.current_yaw
        steering_rad = math.atan2(2.0 * self.L * math.sin(alpha), L_d)
        steering_rad = max(-self.max_steering_rad, min(self.max_steering_rad, steering_rad))

        look_idx_look, _, _ = self.find_index_at_path_distance(
            near_idx,
            self.D_v_look,
        )
        kappa_tilda = self.compute_weighted_curvature_average(near_idx, look_idx_look)

        kappa_tilda_msg = Float64()
        kappa_tilda_msg.data = kappa_tilda
        self.pub_kappa_tilda.publish(kappa_tilda_msg)

        v_curv = 1.0 / (self.G_avg * (0.1 + kappa_tilda))
        v_cmd = max(self.V_min, min(self.V_max, v_curv))

        cmd = ControlCommand()
        cmd.speed = v_cmd
        cmd.steering = steering_rad
        cmd.brake = 0

        self.pub_cmd.publish(cmd)

        self.publish_marker(self.pub_target_marker, tx, ty, 1.0, 0.0, 0.0, 0)
        self.publish_marker(
            self.pub_virtual_target_marker,
            lookahead_x,
            lookahead_y,
            1.0,
            1.0,
            0.0,
            2,
        )
        lx = self.path[look_idx_look].pose.position.x
        ly = self.path[look_idx_look].pose.position.y
        self.publish_marker(self.pub_lookahead_marker, lx, ly, 0.0, 0.0, 1.0, 1)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
