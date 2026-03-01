import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from erp42_racing_msgs.msg import ControlCommand
from erp42_racing_msgs.srv import ModeCommand


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node_')

        self.L = 1.2

        self.declare_parameter('Ld_min', 1.5)
        self.declare_parameter('Ld_max', 5.0)
        self.declare_parameter('G_v', 1.2)
        self.declare_parameter('G_k', 0.5)
        self.declare_parameter('V_max', 1.4)
        self.declare_parameter('V_min', 1.0)
        self.declare_parameter('G_avg', 1.2)
        self.declare_parameter('D_v_look', 15.0)
        self.declare_parameter('max_steering_deg', 20.0)

        self.declare_parameter('nearest_search_window', 50)
        self.declare_parameter('fallback_min_dist_threshold', 1.5)
        self.declare_parameter('fallback_lateral_error_threshold', 1.0)
        self.declare_parameter('fallback_heading_threshold_deg', 45.0)
        self.declare_parameter('fallback_min_speed', 0.5)
        self.declare_parameter('stuck_idx_epsilon', 2)
        self.declare_parameter('stuck_frame_threshold', 15)

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
        self.nearest_search_window = (
            self.get_parameter('nearest_search_window').get_parameter_value().integer_value
        )
        self.fallback_min_dist_threshold = (
            self.get_parameter('fallback_min_dist_threshold').get_parameter_value().double_value
        )
        self.fallback_lateral_error_threshold = (
            self.get_parameter('fallback_lateral_error_threshold')
            .get_parameter_value()
            .double_value
        )
        self.fallback_heading_threshold_deg = (
            self.get_parameter('fallback_heading_threshold_deg')
            .get_parameter_value()
            .double_value
        )
        self.fallback_heading_threshold_rad = math.radians(
            self.fallback_heading_threshold_deg
        )
        self.fallback_min_speed = (
            self.get_parameter('fallback_min_speed').get_parameter_value().double_value
        )
        self.stuck_idx_epsilon = (
            self.get_parameter('stuck_idx_epsilon').get_parameter_value().integer_value
        )
        self.stuck_frame_threshold = (
            self.get_parameter('stuck_frame_threshold').get_parameter_value().integer_value
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
        self.path_yaws = []
        self.target_idx = 0
        self.stuck_counter = 0

        self.have_pose = False
        self.have_yaw = False
        self.have_vel = False

        self.mode_client = self.create_client(ModeCommand, '/erp42_racing/mode_command')
        self._init_vehicle()

        self.pub_cmd = self.create_publisher(ControlCommand, '/erp42_racing/control_command', 10)

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
            elif param.name == 'nearest_search_window':
                self.nearest_search_window = param.value
            elif param.name == 'fallback_min_dist_threshold':
                self.fallback_min_dist_threshold = param.value
            elif param.name == 'fallback_lateral_error_threshold':
                self.fallback_lateral_error_threshold = param.value
            elif param.name == 'fallback_heading_threshold_deg':
                self.fallback_heading_threshold_deg = param.value
                self.fallback_heading_threshold_rad = math.radians(param.value)
            elif param.name == 'fallback_min_speed':
                self.fallback_min_speed = param.value
            elif param.name == 'stuck_idx_epsilon':
                self.stuck_idx_epsilon = param.value
            elif param.name == 'stuck_frame_threshold':
                self.stuck_frame_threshold = param.value
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

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def circular_index_delta(self, idx_a, idx_b, num_waypoints):
        raw_delta = abs(idx_a - idx_b)
        return min(raw_delta, num_waypoints - raw_delta)

    def calculate_path_metadata(self):
        step = 5
        num_waypoints = len(self.path)
        self.path_curvatures = [0.0] * num_waypoints
        self.path_yaws = [0.0] * num_waypoints

        if num_waypoints < 2:
            return

        for i in range(num_waypoints):
            p1 = self.path[(i - step) % num_waypoints].pose.position
            p2 = self.path[i].pose.position
            p3 = self.path[(i + step) % num_waypoints].pose.position

            v1 = (p2.x - p1.x, p2.y - p1.y)
            v2 = (p3.x - p2.x, p3.y - p2.y)
            tangent = (p3.x - p1.x, p3.y - p1.y)

            angle1 = math.atan2(v1[1], v1[0])
            angle2 = math.atan2(v2[1], v2[0])
            d_theta = math.atan2(math.sin(angle2 - angle1), math.cos(angle2 - angle1))
            d_dist = math.hypot(v1[0], v1[1])
            if d_dist > 0.1:
                self.path_curvatures[i] = abs(d_theta / d_dist)

            self.path_yaws[i] = math.atan2(tangent[1], tangent[0])

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
        self.calculate_path_metadata()

    def find_local_nearest_index(self):
        num_waypoints = len(self.path)
        min_dist = float('inf')
        near_idx = self.target_idx

        for i in range(self.target_idx, self.target_idx + self.nearest_search_window):
            idx = i % num_waypoints
            d = math.hypot(
                self.path[idx].pose.position.x - self.current_x,
                self.path[idx].pose.position.y - self.current_y,
            )
            if d < min_dist:
                min_dist = d
                near_idx = idx

        return near_idx, min_dist

    def compute_lateral_error(self, idx):
        dx = self.current_x - self.path[idx].pose.position.x
        dy = self.current_y - self.path[idx].pose.position.y
        return -math.sin(self.current_yaw) * dx + math.cos(self.current_yaw) * dy

    def should_trigger_fallback(self, near_idx, min_dist, lateral_error):
        num_waypoints = len(self.path)
        idx_delta = self.circular_index_delta(near_idx, self.target_idx, num_waypoints)

        if abs(self.v_est) > self.fallback_min_speed and idx_delta <= self.stuck_idx_epsilon:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        trigger_dist = min_dist > self.fallback_min_dist_threshold
        trigger_lat = abs(lateral_error) > self.fallback_lateral_error_threshold
        trigger_stuck = self.stuck_counter >= self.stuck_frame_threshold

        return trigger_dist or (trigger_lat and trigger_stuck)

    def find_global_nearest_with_heading_gate(self):
        min_dist = float('inf')
        near_idx = None

        for idx, pose in enumerate(self.path):
            heading_error = self.wrap_angle(self.path_yaws[idx] - self.current_yaw)
            if abs(heading_error) > self.fallback_heading_threshold_rad:
                continue

            d = math.hypot(
                pose.pose.position.x - self.current_x,
                pose.pose.position.y - self.current_y,
            )
            if d < min_dist:
                min_dist = d
                near_idx = idx

        return near_idx, min_dist

    def control_loop(self):
        if len(self.path) < 2:
            return
        if not (self.have_pose and self.have_yaw and self.have_vel):
            return

        num_waypoints = len(self.path)
        near_idx, min_dist = self.find_local_nearest_index()
        lateral_error = self.compute_lateral_error(near_idx)

        if self.should_trigger_fallback(near_idx, min_dist, lateral_error):
            idx_global, min_dist_global = self.find_global_nearest_with_heading_gate()
            if idx_global is not None:
                near_idx = idx_global
                min_dist = min_dist_global
                lateral_error = self.compute_lateral_error(near_idx)

        self.target_idx = near_idx
        kappa_here = self.path_curvatures[near_idx]

        kappa_msg = Float64()
        kappa_msg.data = kappa_here
        self.pub_kappa.publish(kappa_msg)

        error_msg = Float64()
        error_msg.data = lateral_error
        self.pub_lateral_error.publish(error_msg)

        ld_raw = (abs(self.v_est) * self.G_v) - (kappa_here * self.G_k)
        L_d = max(self.Ld_min, min(self.Ld_max, ld_raw))

        tx = self.current_x
        ty = self.current_y
        for i in range(near_idx, near_idx + num_waypoints):
            idx = i % num_waypoints
            if (
                math.hypot(
                    self.path[idx].pose.position.x - self.current_x,
                    self.path[idx].pose.position.y - self.current_y,
                )
                > L_d
            ):
                tx = self.path[idx].pose.position.x
                ty = self.path[idx].pose.position.y
                break

        alpha = math.atan2(ty - self.current_y, tx - self.current_x) - self.current_yaw
        alpha = self.wrap_angle(alpha)
        steering_rad = math.atan2(2.0 * self.L * math.sin(alpha), L_d)
        steering_rad = max(-self.max_steering_rad, min(self.max_steering_rad, steering_rad))

        look_idx_look = near_idx
        for i in range(near_idx, near_idx + num_waypoints):
            idx = i % num_waypoints
            dist_to_wp = math.hypot(
                self.path[idx].pose.position.x - self.current_x,
                self.path[idx].pose.position.y - self.current_y,
            )
            look_idx_look = idx
            if dist_to_wp > self.D_v_look:
                break

        if near_idx <= look_idx_look:
            segment = self.path_curvatures[near_idx : look_idx_look + 1]
        else:
            segment = self.path_curvatures[near_idx:] + self.path_curvatures[: look_idx_look + 1]

        kappa_tilda = sum(segment) / len(segment) if segment else 0.0

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
