import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from rclpy.node import Node
from visualization_msgs.msg import Marker

from erp42_racing_msgs.msg import ControlCommand


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Vehicle parameters
        self.wheelbase = 1.212

        # Initialization
        self._declare_parameters()
        self._load_parameters()
        self._init_state()

        self.add_on_set_parameters_callback(self.parameter_callback)

        self._create_publishers()
        self._create_subscribers()
        self._create_timers() # control_loop, debug_publish_loop

        self.get_logger().info(
            f'Pure Pursuit Ready: path={self.path_topic}, cmd={self.command_topic}'
        )

    # =========================================================
    # Initialization
    # =========================================================
    def _declare_parameters(self):
        # Pure pursuit / speed parameters
        self.declare_parameter('Ld_min', 2.0)
        self.declare_parameter('Ld_max', 5.0)
        self.declare_parameter('G_v', 1.0)
        self.declare_parameter('G_k', 3.0)
        \
        self.declare_parameter('V_max', 2.0)
        self.declare_parameter('V_min', 1.0)
        self.declare_parameter('G_avg', 1.40)
        self.declare_parameter('D_v_look', 20.0)
        self.declare_parameter('max_steering_deg', 20.0)

        # Path processing parameters
        self.declare_parameter('curvature_window_distance', 1.2)
        self.declare_parameter('resample_ds', 0.4)

        # Topic parameters
        self.declare_parameter('path_topic', '/planning/local_path')
        self.declare_parameter('pose_topic', '/utm_tm')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('vel_topic', '/vel')
        self.declare_parameter('command_topic', '/control/pp_cmd')

    def _load_parameters(self):
        # Pure pursuit / speed parameters
        self.Ld_min = self.get_parameter('Ld_min').value
        self.Ld_max = self.get_parameter('Ld_max').value
        self.G_v = self.get_parameter('G_v').value
        self.G_k = self.get_parameter('G_k').value
        self.V_max = self.get_parameter('V_max').value
        self.V_min = self.get_parameter('V_min').value
        self.G_avg = self.get_parameter('G_avg').value
        self.D_v_look = self.get_parameter('D_v_look').value
        self.max_steering_deg = self.get_parameter('max_steering_deg').value

        # Path processing parameters
        self.curvature_window_distance = self.get_parameter('curvature_window_distance').value
        self.resample_ds = self.get_parameter('resample_ds').value

        # Topics
        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.vel_topic = self.get_parameter('vel_topic').value
        self.command_topic = self.get_parameter('command_topic').value

        # Derived values
        self.max_steering_rad = math.radians(self.max_steering_deg)

    def _init_state(self):
        # Ego state
        self.current_x =    0.0
        self.current_y =    0.0
        self.current_yaw =  0.0
        self.v_est =        0.0
        self.have_pose =    False
        self.have_yaw =    False
        self.have_vel =    False

        # Path state
        self.path =                 []
        self.path_curvatures =      []
        self.path_segment_lengths = []
        self.path_frame_id =        'map'

        # Debug snapshot
        self.debug_snapshot_valid       = False
        self.debug_target_x             = 0.0
        self.debug_target_y             = 0.0
        self.debug_speed_lookahead_x    = 0.0
        self.debug_speed_lookahead_y    = 0.0

    def _create_publishers(self):
        self.pub_cmd = self.create_publisher(
            ControlCommand,
            self.command_topic,
            10,
        )
        self.pub_target_marker = self.create_publisher(
            Marker,
            '/target_waypoint_marker',
            10,
        )
        self.pub_lookahead_marker = self.create_publisher(
            Marker,
            '/lookahead_waypoint_marker',
            10,
        )

    def _create_subscribers(self):
        self.sub_path = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10,
        )
        self.sub_pose = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.sub_imu = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )
        self.sub_vel = self.create_subscription(
            TwistWithCovarianceStamped,
            self.vel_topic,
            self.vel_callback,
            10,
        )

    def _create_timers(self):
        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.debug_timer = self.create_timer(0.1, self.debug_publish_loop)

    # =========================================================
    # Parameter callback
    # =========================================================
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

            elif param.name == 'curvature_window_distance':
                self.curvature_window_distance = param.value

            elif param.name == 'resample_ds':
                self.resample_ds = param.value

            self.get_logger().info(f'Parameter {param.name} updated to {param.value}')

        return SetParametersResult(successful=True)

    # =========================================================
    # ROS callbacks
    # =========================================================
    def path_callback(self, msg):
        if len(msg.poses) < 2:
            self._clear_path_data()
            return

        self.path_frame_id = msg.header.frame_id or 'map'
        self.path = self._resample_path(msg.poses)
        self._calculate_path_curvatures()

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.have_pose = True

    def imu_callback(self, msg):
        self.current_yaw = self._get_yaw_from_quaternion(msg.orientation)
        self.have_yaw = True

    def vel_callback(self, msg):
        self.v_est = msg.twist.twist.linear.x
        self.have_vel = True

    def control_loop(self):
        if not self._is_ready_for_control():
            
            return

        near_idx, _ = self._find_nearest_index()

        kappa_here = self._get_current_curvature(near_idx)

        lookahead_distance = self._compute_lookahead_distance(kappa_here)
        _, target_x, target_y = self._find_point_at_path_distance(
            near_idx,
            lookahead_distance,
        )

        steering_rad = self._compute_steering_command(
            target_x,
            target_y,
            lookahead_distance,
        )

        speed_idx, speed_lookahead_x, speed_lookahead_y = self._find_point_at_path_distance(
            near_idx,
            self.D_v_look,
        )
        kappa_tilda = self._compute_weighted_curvature_average(near_idx, speed_idx)
        speed_cmd = self._compute_speed_command(kappa_tilda)

        self._publish_control_command(speed_cmd, steering_rad)

        self._update_debug_snapshot(
            target_x=target_x,
            target_y=target_y,
            speed_lookahead_x=speed_lookahead_x,
            speed_lookahead_y=speed_lookahead_y,
        )

    def debug_publish_loop(self):
        if not self.debug_snapshot_valid:
            return

        self.publish_marker(
            publisher=self.pub_target_marker,
            x=self.debug_target_x,
            y=self.debug_target_y,
            r=1.0,
            g=0.0,
            b=0.0,
            marker_id=0,
        )

        self.publish_marker(
            publisher=self.pub_lookahead_marker,
            x=self.debug_speed_lookahead_x,
            y=self.debug_speed_lookahead_y,
            r=0.0,
            g=0.0,
            b=1.0,
            marker_id=1,
        )

    # =========================================================
    # Control helpers
    # =========================================================
    def _is_ready_for_control(self):
        if len(self.path) < 2:
            return False

        if not self.have_pose:
            return False
        if not self.have_yaw:
            return False
        if not self.have_vel:
            return False

        return True

    def _get_current_curvature(self, near_idx):
        if not self.path_curvatures:
            return 0.0

        return self.path_curvatures[near_idx]

    def _compute_lookahead_distance(self, kappa_here):
        ld_raw = (abs(self.v_est) * self.G_v) - (kappa_here * self.G_k)

        if ld_raw < self.Ld_min:
            return self.Ld_min

        if ld_raw > self.Ld_max:
            return self.Ld_max

        return ld_raw

    def _compute_steering_command(self, target_x, target_y, lookahead_distance):
        alpha = math.atan2(
            target_y - self.current_y,
            target_x - self.current_x,
        ) - self.current_yaw

        steering_rad = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            lookahead_distance,
        )

        if steering_rad > self.max_steering_rad:
            steering_rad = self.max_steering_rad
        elif steering_rad < -self.max_steering_rad:
            steering_rad = -self.max_steering_rad

        return steering_rad

    def _compute_speed_command(self, kappa_tilda):
        v_curv = 1.0 / (self.G_avg * (0.1 + kappa_tilda))

        if v_curv < self.V_min:
            return self.V_min

        if v_curv > self.V_max:
            return self.V_max

        return v_curv

    def _publish_control_command(self, speed, steering):
        cmd = ControlCommand()
        cmd.speed = float(speed)
        cmd.steering = float(steering)
        cmd.brake = 0
        self.pub_cmd.publish(cmd)

    def _update_debug_snapshot(
        self,
        target_x,
        target_y,
        speed_lookahead_x,
        speed_lookahead_y,
    ):
        self.debug_target_x = target_x
        self.debug_target_y = target_y
        self.debug_speed_lookahead_x = speed_lookahead_x
        self.debug_speed_lookahead_y = speed_lookahead_y
        self.debug_snapshot_valid = True

    # =========================================================
    # Path processing helpers
    # =========================================================
    def _clear_path_data(self):
        self.path = []
        self.path_curvatures = []
        self.path_segment_lengths = []

    def _build_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = self.path_frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def _resample_path(self, poses):
        """
        Resample input path with constant arc-length spacing.
        This makes nearest-point search and preview distance handling more stable.
        """
        if len(poses) < 2:
            return []

        path_xy = []
        for pose in poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            path_xy.append((x, y))

        cumulative_distances = [0.0]
        total_length = 0.0

        for i in range(len(path_xy) - 1):
            x0, y0 = path_xy[i]
            x1, y1 = path_xy[i + 1]

            seg_len = math.hypot(x1 - x0, y1 - y0)
            total_length += seg_len
            cumulative_distances.append(total_length)

        if total_length <= 1e-6:
            x0, y0 = path_xy[0]
            return [self._build_pose(x0, y0)]

        sample_step = max(float(self.resample_ds), 1e-3)

        sample_distances = [0.0]
        current_distance = sample_step
        while current_distance < total_length:
            sample_distances.append(current_distance)
            current_distance += sample_step
        sample_distances.append(total_length)

        resampled_path = []
        seg_idx = 0

        for target_distance in sample_distances:
            while (
                seg_idx < len(cumulative_distances) - 2
                and cumulative_distances[seg_idx + 1] < target_distance
            ):
                seg_idx += 1

            seg_start_dist = cumulative_distances[seg_idx]
            seg_end_dist = cumulative_distances[seg_idx + 1]

            x0, y0 = path_xy[seg_idx]
            x1, y1 = path_xy[seg_idx + 1]

            seg_len = seg_end_dist - seg_start_dist
            if seg_len <= 1e-9:
                ratio = 0.0
            else:
                ratio = (target_distance - seg_start_dist) / seg_len

            x = x0 + ratio * (x1 - x0)
            y = y0 + ratio * (y1 - y0)

            resampled_path.append(self._build_pose(x, y))

        return resampled_path

    def _calculate_path_curvatures(self):
        num_points = len(self.path)

        self.path_curvatures = [0.0] * num_points
        self.path_segment_lengths = [0.0] * num_points

        if num_points < 3:
            return

        for i in range(num_points - 1):
            p0 = self.path[i].pose.position
            p1 = self.path[i + 1].pose.position

            seg_len = math.hypot(p1.x - p0.x, p1.y - p0.y)
            self.path_segment_lengths[i] = seg_len

        self.path_segment_lengths[-1] = 0.0

        total_path_length = sum(self.path_segment_lengths)
        if total_path_length <= 1e-6:
            return

        window_distance = min(
            self.curvature_window_distance,
            total_path_length * 0.25,
        )
        if window_distance <= 1e-6:
            return

        for i in range(num_points):
            curr = self.path[i].pose.position

            mid_x, mid_y = self._interpolate_forward_point_along_path(i, window_distance)
            next_x, next_y = self._interpolate_forward_point_along_path(i, 2.0 * window_distance)

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

    def _interpolate_forward_point_along_path(self, start_idx, target_distance):
        point = self.path[start_idx].pose.position

        if target_distance <= 1e-9:
            return point.x, point.y

        remaining_distance = target_distance
        current_idx = start_idx
        num_points = len(self.path)

        for _ in range(num_points):
            if current_idx >= num_points - 1:
                final_point = self.path[-1].pose.position
                return final_point.x, final_point.y

            next_idx = current_idx + 1

            start_point = self.path[current_idx].pose.position
            end_point = self.path[next_idx].pose.position
            seg_len = self.path_segment_lengths[current_idx]

            if seg_len <= 1e-9:
                current_idx = next_idx
                continue

            if remaining_distance <= seg_len:
                ratio = remaining_distance / seg_len
                x = start_point.x + ratio * (end_point.x - start_point.x)
                y = start_point.y + ratio * (end_point.y - start_point.y)
                return x, y

            remaining_distance -= seg_len
            current_idx = next_idx

        final_point = self.path[-1].pose.position
        return final_point.x, final_point.y

    def _find_nearest_index(self):
        min_dist = float('inf')
        nearest_idx = 0

        for idx, pose in enumerate(self.path):
            px = pose.pose.position.x
            py = pose.pose.position.y

            dist = math.hypot(px - self.current_x, py - self.current_y)

            if dist < min_dist:
                min_dist = dist
                nearest_idx = idx

        return nearest_idx, min_dist

    def _find_point_at_path_distance(self, start_idx, target_distance):
        """
        Move forward along the path by target_distance starting from start_idx.
        Return:
            end_index, x, y
        """
        if target_distance <= 0.0:
            point = self.path[start_idx].pose.position
            return start_idx, point.x, point.y

        accumulated_distance = 0.0
        current_idx = start_idx
        num_points = len(self.path)

        for seg_idx in range(start_idx, num_points - 1):
            next_idx = seg_idx + 1

            seg_len = self.path_segment_lengths[seg_idx]
            start_point = self.path[seg_idx].pose.position
            end_point = self.path[next_idx].pose.position

            if seg_len <= 1e-9:
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

    def _compute_weighted_curvature_average(self, start_idx, end_idx):
        if start_idx == end_idx:
            return self.path_curvatures[start_idx]

        weighted_sum = 0.0
        total_length = 0.0

        for idx in range(start_idx, end_idx):
            seg_len = self.path_segment_lengths[idx]

            if seg_len > 0.0:
                weighted_sum += self.path_curvatures[idx] * seg_len
                total_length += seg_len

        if total_length > 0.0:
            return weighted_sum / total_length

        return self.path_curvatures[start_idx]

    # =========================================================
    # Visualization / utility helpers
    # =========================================================
    def publish_marker(self, publisher, x, y, r, g, b, marker_id):
        marker = Marker()
        marker.header.frame_id = self.path_frame_id
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

    def _get_yaw_from_quaternion(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )


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
