import copy

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from erp42_racing_msgs.msg import ControlCommand
from erp42_racing_msgs.srv import ModeCommand

class VehicleCmdGateNode(Node):
    def __init__(self):
        super().__init__('vehicle_cmd_gate_node')

        # Initialization
        self._declare_parameters()
        self._load_parameters()
        self._init_state()

        # ROS interfaces
        self.mode_client = self.create_client(ModeCommand, self.mode_service)
        self._init_vehicle_mode()

        self._create_publishers()
        self._create_subscribers()
        self._create_timers()

        self.get_logger().info(
            f'Vehicle Cmd Gate Ready: pp={self.pp_topic}, aeb={self.aeb_topic}, out={self.output_topic}'
        )

    # =========================================================
    # Initialization
    # =========================================================
    def _declare_parameters(self):
        self.declare_parameter('pp_topic', '/control/pp_cmd')
        self.declare_parameter('aeb_topic', '/control/aeb_cmd')
        self.declare_parameter('output_topic', '/erp42_racing/control_command')
        self.declare_parameter('mode_service', '/erp42_racing/mode_command')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('pp_timeout_sec', 0.3)
        self.declare_parameter('aeb_timeout_sec', 0.3)

    def _load_parameters(self):
        self.pp_topic = self.get_parameter('pp_topic').value
        self.aeb_topic = self.get_parameter('aeb_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.mode_service = self.get_parameter('mode_service').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.pp_timeout_sec = self.get_parameter('pp_timeout_sec').value
        self.aeb_timeout_sec = self.get_parameter('aeb_timeout_sec').value

    def _init_state(self):
        self.latest_pp_cmd = None
        self.latest_pp_time = None

        self.latest_aeb_cmd = None
        self.latest_aeb_time = None

        self.current_mode = None

    def _init_vehicle_mode(self):
        while not self.mode_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for ModeCommand Service...')
        
        req = ModeCommand.Request()
        req.manual_mode = False
        req.emergency_stop = False
        req.gear = 0

        self.mode_client.call_async(req)
        self.get_logger().info('ModeCommand initialized by gate node')

    def _create_publishers(self):
        self.pub_cmd = self.create_publisher(
            ControlCommand,
            self.output_topic,
            10,
        )
    
    def _create_subscribers(self):
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
    
    def _create_timers(self):
        timer_period = 1.0 / max(1.0, float(self.publish_rate_hz))
        self.timer = self.create_timer(timer_period, self.control_loop) # timer_period 마다 control_loop 자동호출

    # =========================================================
    # ROS callbacks
    # =========================================================
    def pp_callback(self, msg):
        self.latest_pp_cmd = copy.deepcopy(msg)
        self.latest_pp_time = self.get_clock().now()

    def aeb_callback(self, msg):
        self.latest_aeb_cmd = copy.deepcopy(msg)
        self.latest_aeb_time = self.get_clock().now()

    def control_loop(self):
        pp_valid = self._is_pp_valid()
        aeb_valid = self._is_aeb_valid()

        if aeb_valid:
            cmd = self._build_aeb_command(pp_valid)
            self.pub_cmd.publish(cmd)
            self._update_mode('AEB')
            return
        
        if pp_valid:
            cmd = copy.deepcopy(self.latest_pp_cmd)
            self.pub_cmd.publish(cmd)
            self._update_mode('PURE PURSUIT')
            return
        
        cmd = self._build_stop_command()
        self.pub_cmd.publish(cmd)
        self._update_mode('STOP')


    # =========================================================
    # Gate logic helpers
    # =========================================================
    def _is_recent(self, stamp, timeout_sec):
        if stamp is None:
            return False

        elapsed_sec = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return elapsed_sec <= timeout_sec

    def _is_pp_valid(self):
        if self.latest_pp_cmd is None:
            return False
        
        if not self._is_recent(self.latest_pp_time, self.pp_timeout_sec):
            return False

        return True
    
    def _is_aeb_valid(self):
        if self.latest_aeb_cmd is None:
            return False

        if not self._is_recent(self.latest_aeb_time, self.aeb_timeout_sec):
            return False

        if self.latest_aeb_cmd.brake <= 0:
            return False
        
        return True
    
    def _update_mode(self, mode):
        if mode == self.current_mode:
            return

        self.current_mode = mode
        msg = f'Gate mode -> {mode}'

        if mode == 'AEB' or mode == 'STOP':
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)
    
    # =========================================================
    # Command builders
    # =========================================================  
    def _build_aeb_command(self, pp_valid):
        cmd = ControlCommand()

        cmd.speed = 0.0
        cmd.brake = int(self.latest_aeb_cmd.brake)

        if pp_valid:
            cmd.steering = float(self.latest_pp_cmd.steering)
        else:
            cmd.steering = 0.0
        
        return cmd

    def _build_stop_command(self):
        cmd = ControlCommand()

        cmd.speed = 0.0
        cmd.steering = 0.0
        cmd.brake = 0

        return cmd


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