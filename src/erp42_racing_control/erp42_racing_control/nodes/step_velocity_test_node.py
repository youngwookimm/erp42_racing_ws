import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu

from erp42_racing_msgs.msg import ControlCommand, Feedback
from erp42_racing_msgs.srv import ModeCommand


class TestPhase(Enum):
    WAIT_FOR_DATA = auto()
    WAIT_FOR_AUTO = auto()
    HOLD_HIGH = auto()
    STEP_DOWN = auto()
    FINISH = auto()


class StepVelocityTestNode(Node):
    def __init__(self):
        super().__init__('step_velocity_test_node')

        self.declare_parameter('pose_topic', '/utm')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('vel_topic', '/vel')
        self.declare_parameter('feedback_topic', '/erp42_racing/feedback')
        self.declare_parameter('control_topic', '/erp42_racing/control_command')
        self.declare_parameter('mode_service', '/erp42_racing/mode_command')

        self.declare_parameter('command_rate_hz', 50.0) # 제어 명령 주기
        self.declare_parameter('v_high', 2.0) # 고속 구간 유지 목표 속도
        self.declare_parameter('v_low', 1.0) # 감속 구간 유지 목표 속도
        self.declare_parameter('hold_high_sec', 4.0) # 고속 구간 시간
        self.declare_parameter('hold_low_sec', 4.0) # 감속 구간 시간
        self.declare_parameter('steering_cmd', 0.0)
        self.declare_parameter('brake_high', 0) 
        self.declare_parameter('brake_low', 0) # 감속 구간에서 brake 얼마나 넣을지
        self.declare_parameter('finish_brake', 10)
        self.declare_parameter('settle_error_threshold', 0.1)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.vel_topic = self.get_parameter('vel_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.mode_service_name = self.get_parameter('mode_service').value

        self.command_rate_hz = self.get_parameter('command_rate_hz').value
        self.v_high = self.get_parameter('v_high').value
        self.v_low = self.get_parameter('v_low').value
        self.hold_high_sec = self.get_parameter('hold_high_sec').value
        self.hold_low_sec = self.get_parameter('hold_low_sec').value
        self.steering_cmd = self.get_parameter('steering_cmd').value
        self.brake_high = int(self.get_parameter('brake_high').value)
        self.brake_low = int(self.get_parameter('brake_low').value)
        self.finish_brake = int(self.get_parameter('finish_brake').value)
        self.settle_error_threshold = self.get_parameter('settle_error_threshold').value

        self.have_pose = False
        self.have_yaw = False
        self.have_vel = False
        self.have_feedback = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_velocity = 0.0
        self.feedback_speed = 0.0
        self.manual_mode = True
        self.emergency_stop = False
        self.current_gear = Feedback.GEAR_NEUTRAL

        self.phase = TestPhase.WAIT_FOR_DATA
        self.phase_start_sec = None
        self.mode_request_sent = False
        self.mode_future = None
        self.finish_logged = False

        self.step_start_sec = None
        self.step_response_sec = None
        self.step_max_abs_error = 0.0
        self.last_step_error = 0.0

        self.pub_cmd = self.create_publisher(ControlCommand, self.control_topic, 10)
        self.mode_client = self.create_client(ModeCommand, self.mode_service_name)

        self.sub_pose = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.sub_vel = self.create_subscription(
            TwistWithCovarianceStamped,
            self.vel_topic,
            self.vel_callback,
            10,
        )
        self.sub_feedback = self.create_subscription(
            Feedback,
            self.feedback_topic,
            self.feedback_callback,
            10,
        )

        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            'Step velocity test ready: '
            f'v_high={self.v_high}, v_low={self.v_low}, '
            f'hold_high={self.hold_high_sec}s, hold_low={self.hold_low_sec}s'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def get_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

    def transition_to(self, phase, message):
        self.phase = phase
        self.phase_start_sec = self.now_sec()
        self.get_logger().info(f'[{phase.name}] {message}')

    def publish_command(self, speed, steering, brake):
        cmd = ControlCommand()
        cmd.speed = float(max(speed, 0.0))
        cmd.steering = float(steering)
        cmd.brake = int(max(0, min(100, brake)))
        self.pub_cmd.publish(cmd)

    def send_mode_request(self):
        req = ModeCommand.Request()
        req.manual_mode = False
        req.emergency_stop = False
        req.gear = ModeCommand.Request.GEAR_DRIVE
        self.mode_future = self.mode_client.call_async(req)
        self.mode_request_sent = True
        self.get_logger().info('ModeCommand request sent: auto, estop off, gear drive')

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.have_pose = True

    def imu_callback(self, msg):
        self.current_yaw = self.get_yaw(msg.orientation)
        self.have_yaw = True

    def vel_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x
        self.have_vel = True

    def feedback_callback(self, msg):
        self.feedback_speed = msg.speed
        self.manual_mode = msg.manual_mode
        self.emergency_stop = msg.emergency_stop
        self.current_gear = msg.gear
        self.have_feedback = True

    def all_inputs_ready(self):
        return self.have_pose and self.have_yaw and self.have_vel and self.have_feedback

    def handle_wait_for_data(self):
        self.publish_command(0.0, 0.0, 0)
        if self.all_inputs_ready():
            self.transition_to(TestPhase.WAIT_FOR_AUTO, 'All required topics received')

    def handle_wait_for_auto(self):
        self.publish_command(0.0, 0.0, 0)

        if self.manual_mode:
            return

        if not self.mode_client.service_is_ready():
            self.get_logger().warn('ModeCommand service is not available yet')
            return

        if not self.mode_request_sent:
            self.send_mode_request()
            return

        if self.mode_future is None or not self.mode_future.done():
            return

        response = self.mode_future.result()
        if response is None or not response.success:
            self.get_logger().warn('ModeCommand failed, retrying')
            self.mode_request_sent = False
            self.mode_future = None
            return

        self.transition_to(TestPhase.HOLD_HIGH, 'Auto confirmed and mode command accepted')

    def handle_hold_high(self):
        self.publish_command(self.v_high, self.steering_cmd, self.brake_high)

        if self.manual_mode or self.emergency_stop:
            self.transition_to(TestPhase.FINISH, 'Experiment aborted by manual mode or estop')
            return

        elapsed = self.now_sec() - self.phase_start_sec
        if elapsed >= self.hold_high_sec:
            self.step_start_sec = self.now_sec()
            self.step_response_sec = None
            self.step_max_abs_error = 0.0
            self.last_step_error = 0.0
            self.transition_to(TestPhase.STEP_DOWN, 'Applying step-down velocity command')

    def handle_step_down(self):
        self.publish_command(self.v_low, self.steering_cmd, self.brake_low)

        if self.manual_mode or self.emergency_stop:
            self.transition_to(TestPhase.FINISH, 'Experiment aborted by manual mode or estop')
            return

        error = self.v_low - self.current_velocity
        abs_error = abs(error)
        self.last_step_error = error
        self.step_max_abs_error = max(self.step_max_abs_error, abs_error)

        if self.step_response_sec is None and abs_error <= self.settle_error_threshold:
            self.step_response_sec = self.now_sec() - self.step_start_sec

        elapsed = self.now_sec() - self.phase_start_sec
        if elapsed >= self.hold_low_sec:
            self.transition_to(TestPhase.FINISH, 'Step-down measurement complete')

    def handle_finish(self):
        self.publish_command(0.0, 0.0, self.finish_brake)

        if self.finish_logged:
            return

        if self.step_start_sec is None:
            self.get_logger().info('Experiment finished before step-down phase')
        else:
            response_text = (
                f'{self.step_response_sec:.3f}s'
                if self.step_response_sec is not None
                else 'not reached'
            )
            self.get_logger().info(
                'Step-down summary: '
                f'current_vel={self.current_velocity:.3f} m/s, '
                f'feedback_speed={self.feedback_speed:.3f} m/s, '
                f'last_error={self.last_step_error:.3f} m/s, '
                f'max_abs_error={self.step_max_abs_error:.3f} m/s, '
                f'response_time={response_text}'
            )

        self.finish_logged = True

    def control_loop(self):
        if self.phase == TestPhase.WAIT_FOR_DATA:
            self.handle_wait_for_data()
        elif self.phase == TestPhase.WAIT_FOR_AUTO:
            self.handle_wait_for_auto()
        elif self.phase == TestPhase.HOLD_HIGH:
            self.handle_hold_high()
        elif self.phase == TestPhase.STEP_DOWN:
            self.handle_step_down()
        elif self.phase == TestPhase.FINISH:
            self.handle_finish()

    def send_stop_once(self):
        self.publish_command(0.0, 0.0, self.finish_brake)


def main(args=None):
    rclpy.init(args=args)
    node = StepVelocityTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop_once()
        node.destroy_node()
        rclpy.shutdown()
