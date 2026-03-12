from enum import Enum, auto

import rclpy
from rclpy.node import Node

from erp42_racing_msgs.msg import ControlCommand, Feedback
from erp42_racing_msgs.srv import ModeCommand


class TestPhase(Enum):
    WAIT_FOR_FEEDBACK = auto()
    WAIT_FOR_AUTO = auto()
    HOLD_HIGH = auto()
    FINISH = auto()


class BrakingInputTestNode(Node):
    def __init__(self):
        super().__init__('braking_input_test_node')

        self.declare_parameter('feedback_topic', '/erp42_racing/feedback')
        self.declare_parameter('control_topic', '/erp42_racing/control_command')
        self.declare_parameter('mode_service', '/erp42_racing/mode_command')

        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('hold_speed', 1.0)
        self.declare_parameter('steering_cmd', 0.0)
        self.declare_parameter('hold_brake', 0)
        self.declare_parameter('finish_brake', 10)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.mode_service_name = self.get_parameter('mode_service').value

        self.command_rate_hz = self.get_parameter('command_rate_hz').value
        self.hold_speed = self.get_parameter('hold_speed').value
        self.steering_cmd = self.get_parameter('steering_cmd').value
        self.hold_brake = int(self.get_parameter('hold_brake').value)
        self.finish_brake = int(self.get_parameter('finish_brake').value)

        self.have_feedback = False
        self.manual_mode = True
        self.prev_manual_mode = True
        self.emergency_stop = False

        self.phase = TestPhase.WAIT_FOR_FEEDBACK
        self.mode_request_sent = False
        self.mode_future = None
        self.finish_logged = False

        self.pub_cmd = self.create_publisher(ControlCommand, self.control_topic, 10)
        self.mode_client = self.create_client(ModeCommand, self.mode_service_name)

        self.sub_feedback = self.create_subscription(
            Feedback,
            self.feedback_topic,
            self.feedback_callback,
            10,
        )

        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            'Braking input test ready: '
            f'hold_speed={self.hold_speed}, steering={self.steering_cmd}, hold_brake={self.hold_brake}'
        )

    def transition_to(self, phase, message):
        self.phase = phase
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

    def feedback_callback(self, msg):
        self.prev_manual_mode = self.manual_mode
        self.manual_mode = msg.manual_mode
        self.emergency_stop = msg.emergency_stop
        self.have_feedback = True

    def handle_wait_for_feedback(self):
        self.publish_command(0.0, 0.0, 0)
        if self.have_feedback:
            self.transition_to(TestPhase.WAIT_FOR_AUTO, 'Feedback received')

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
        self.publish_command(self.hold_speed, self.steering_cmd, self.hold_brake)

        manual_triggered = self.prev_manual_mode is False and self.manual_mode is True
        stop_triggered = manual_triggered or self.emergency_stop
        if stop_triggered:
            self.transition_to(TestPhase.FINISH, 'Manual switch or estop detected')

    def handle_finish(self):
        self.publish_command(0.0, 0.0, self.finish_brake)
        if self.finish_logged:
            return
        self.get_logger().info('Finish state: sending stop command with finish brake')
        self.finish_logged = True

    def control_loop(self):
        if self.phase == TestPhase.WAIT_FOR_FEEDBACK:
            self.handle_wait_for_feedback()
        elif self.phase == TestPhase.WAIT_FOR_AUTO:
            self.handle_wait_for_auto()
        elif self.phase == TestPhase.HOLD_HIGH:
            self.handle_hold_high()
        elif self.phase == TestPhase.FINISH:
            self.handle_finish()

    def send_stop_once(self):
        self.publish_command(0.0, 0.0, self.finish_brake)


def main(args=None):
    rclpy.init(args=args)
    node = BrakingInputTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop_once()
        node.destroy_node()
        rclpy.shutdown()
