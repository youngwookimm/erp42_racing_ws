import copy

import rclpy
from rclpy.node import Node

from erp42_racing_msgs.msg import ControlCommand
from erp42_racing_msgs.srv import ModeCommand


class VehicleCmdGateNode(Node):
    def __init__(self):
        super().__init__('vehicle_cmd_gate_node')

        self.declare_parameter('pp_topic', '/vehicle_cmd_gate/pp_cmd')
        self.declare_parameter('aeb_topic', '/vehicle_cmd_gate/aeb_cmd')
        self.declare_parameter('output_topic', '/erp42_racing/control_command')
        self.declare_parameter('mode_service', '/erp42_racing/mode_command')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('pp_timeout_sec', 0.3)
        self.declare_parameter('aeb_timeout_sec', 0.3)

        self.pp_topic = self.get_parameter('pp_topic').value
        self.aeb_topic = self.get_parameter('aeb_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.mode_service = self.get_parameter('mode_service').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.pp_timeout_sec = self.get_parameter('pp_timeout_sec').value
        self.aeb_timeout_sec = self.get_parameter('aeb_timeout_sec').value

        self.latest_pp_cmd = None
        self.latest_pp_time = None
        self.latest_aeb_cmd = None
        self.latest_aeb_time = None
        self.mode_initialized = False
        self.last_gate_state = 'IDLE'

        self.mode_client = self.create_client(ModeCommand, self.mode_service)
        self.sub_pp = self.create_subscription(
            ControlCommand,
            self.pp_topic,
            self.pp_callback,
            10,
        )
        self.sub_aeb = self.create_subscription(
            ControlCommand,
            self.aeb_topic,
            self.aeb_callback,
            10,
        )
        self.pub_cmd = self.create_publisher(ControlCommand, self.output_topic, 10)

        period = 1.0 / max(1.0, float(self.publish_rate_hz))
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f'Vehicle Cmd Gate Ready: pp={self.pp_topic}, aeb={self.aeb_topic}, out={self.output_topic}'
        )

    def pp_callback(self, msg):
        self.latest_pp_cmd = copy.deepcopy(msg)
        self.latest_pp_time = self.get_clock().now()

    def aeb_callback(self, msg):
        self.latest_aeb_cmd = copy.deepcopy(msg)
        self.latest_aeb_time = self.get_clock().now()

    def init_mode(self):
        if self.mode_initialized:
            return
        if not self.mode_client.service_is_ready():
            return

        req = ModeCommand.Request()
        req.manual_mode = False
        req.emergency_stop = False
        req.gear = ModeCommand.Request.GEAR_DRIVE
        self.mode_client.call_async(req)
        self.mode_initialized = True
        self.get_logger().info('ModeCommand initialized by gate node')

    def recent(self, stamp, timeout_sec):
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= timeout_sec

    def control_loop(self):
        self.init_mode()

        pp_recent = self.recent(self.latest_pp_time, self.pp_timeout_sec)
        aeb_recent = self.recent(self.latest_aeb_time, self.aeb_timeout_sec)
        aeb_active = aeb_recent and self.latest_aeb_cmd is not None and self.latest_aeb_cmd.brake > 0

        if not pp_recent and not aeb_active:
            return

        if aeb_active:
            cmd = ControlCommand()
            cmd.speed = 0.0
            cmd.brake = int(self.latest_aeb_cmd.brake)
            if pp_recent and self.latest_pp_cmd is not None:
                cmd.steering = float(self.latest_pp_cmd.steering)
                cmd.brake = max(int(self.latest_pp_cmd.brake), int(self.latest_aeb_cmd.brake))
            else:
                cmd.steering = 0.0
            gate_state = 'FULL_AEB' if cmd.brake >= 100 else 'PARTIAL_AEB'
        elif pp_recent and self.latest_pp_cmd is not None:
            cmd = copy.deepcopy(self.latest_pp_cmd)
            gate_state = 'PP'
        else:
            return

        self.pub_cmd.publish(cmd)

        if gate_state != self.last_gate_state:
            self.last_gate_state = gate_state
            self.get_logger().info(f'Gate state -> {gate_state}')


def main(args=None):
    rclpy.init(args=args)
    node = VehicleCmdGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
