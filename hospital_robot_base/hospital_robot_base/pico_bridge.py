#!/usr/bin/env python3
"""
pico_bridge.py  --  ROS 2 Jazzy node that connects ROS to the Pico W.

    /cmd_vel  ──►  [pico_bridge]  ──►  "v <left> <right>"  ──►  Pico
    /odom     ◄──  [pico_bridge]  ◄──  "e <ticks...>"      ◄──  Pico
    TF odom -> base_footprint
    /wheel_states (sensor_msgs/JointState, merged into /joint_states)
    /pico/status, /pico/battery, /robot_state
    Head-column lift (Phase 8, wiring rev 2, firmware 1.1.0):
    /lift/command (std_msgs/String: home | park | stop | goto <mm> [speed])  ──►  "l ..."
    /lift/state   (std_msgs/String: the raw 'l' status line from the Pico)
    /column_states (sensor_msgs/JointState, merged into /joint_states, once homed)

Services:
    /pico_bridge/arm           std_srvs/SetBool   true = arm, false = disarm
    /pico_bridge/clear_faults  std_srvs/Trigger

SAFETY DESIGN
  * The bridge NEVER arms the motors by itself. A human must call the arm
    service. (No automatic restart - same rule as your firmware.)
  * If /cmd_vel stops arriving for cmd_timeout seconds, it sends zero speed.
  * If this node dies, it stops sending, and the Pico's own 0.5 s watchdog
    stops the wheels. Two independent layers.
  * On a clean shutdown (Ctrl+C) it sends 'v 0 0' and 'a 0'.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from hospital_robot_base.ultrasonic import UltrasonicPublisher  # Phase 7

import serial

from hospital_robot_base.pico_protocol import (
    EncoderMsg, StatusMsg, LiftMsg, lift_text, lift_command_line,
    parse_line, status_summary, status_text,
    body_to_wheels, wheels_to_body, DiffDriveOdometry,
    micros_delta, yaw_to_quaternion, TickScaleChecker,
)


class PicoBridge(Node):

    def __init__(self):
        super().__init__('pico_bridge')

        # ---------------- parameters (values come from config/base.yaml) ----
        p = self.declare_parameter
        self.port = p('port', '/dev/pico').value
        self.baud = p('baud', 115200).value
        self.wheel_radius = p('wheel_radius', 0.072).value
        self.wheel_separation = p('wheel_separation', 0.26).value
        self.ticks_per_rev = p('ticks_per_wheel_rev', 15360.0).value
        self.max_wheel_speed = p('max_wheel_speed', 9.0).value
        self.cmd_timeout = p('cmd_timeout', 0.6).value
        self.send_rate = p('send_rate', 50.0).value
        self.use_stamped = p('use_stamped', True).value
        self.publish_tf = p('publish_tf', True).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.left_joint = p('left_wheel_joint', 'left_wheel_joint').value
        self.right_joint = p('right_wheel_joint', 'right_wheel_joint').value
        self.disarm_on_exit = p('disarm_on_exit', True).value
        self.echo_pico_text = p('echo_pico_text', True).value
        # Phase 7 (KNOWN_ISSUE_ODOM_FALSE_SPEED): /odom speed from tick
        # differences, not from the Pico's noisy speed readings.
        self.twist_from_ticks = p('twist_from_ticks', True).value
        self.column_joint = p('column_joint', 'column_joint').value   # Phase 8 lift
        self.last_lift_key = None

        # --------------- odometry state -------------------------------------
        self.odom = DiffDriveOdometry(self.wheel_radius, self.wheel_separation,
                                      self.ticks_per_rev)
        self.checker = TickScaleChecker()
        # A wheel can't physically turn 2 full revolutions in one 20 ms step,
        # so a bigger jump means the Pico restarted.
        self.max_jump = int(2 * self.ticks_per_rev)
        self.wheel_pos_l = 0.0
        self.wheel_pos_r = 0.0

        # --------------- command state --------------------------------------
        self.target_l = 0.0
        self.target_r = 0.0
        self.last_cmd_time = 0.0
        self.was_commanding = False

        # --------------- serial state ---------------------------------------
        self.ser = None
        self.rx_buffer = b''
        self.last_rx_time = 0.0
        self.last_open_attempt = 0.0
        self.last_status_key = None
        self.info_requested_at = None      # when we sent 'i' after connecting
        self.info_checked = False
        self.not_armed_msgs = 0

        # --------------- publishers -----------------------------------------
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'wheel_states', 10)
        self.status_pub = self.create_publisher(String, 'pico/status', 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        self.batt_pub = self.create_publisher(BatteryState, 'pico/battery', 10)
        self.lift_pub = self.create_publisher(String, 'lift/state', 10)
        self.column_pub = self.create_publisher(JointState, 'column_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        # Phase 7: 'u <n> <mm>' lines -> /ultrasonic/* (sensor_msgs/Range)
        self.ultrasonic = UltrasonicPublisher(self)

        # --------------- subscriber -----------------------------------------
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        if self.use_stamped:
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self.on_cmd(m.twist), qos)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, qos)
        self.create_subscription(String, 'lift/command', self.on_lift_cmd, 10)

        # --------------- services -------------------------------------------
        self.create_service(SetBool, '~/arm', self.on_arm)
        self.create_service(Trigger, '~/clear_faults', self.on_clear)

        # --------------- timers ---------------------------------------------
        self.create_timer(1.0 / self.send_rate, self.send_tick)
        self.create_timer(0.005, self.read_tick)          # 200 Hz serial poll
        self.create_timer(1.0, self.health_tick)

        self.get_logger().info(
            f'pico_bridge started. port={self.port} r={self.wheel_radius} m '
            f'L={self.wheel_separation} m ticks/rev={self.ticks_per_rev} '
            f'cmd_vel type={"TwistStamped" if self.use_stamped else "Twist"}')
        self.get_logger().info(
            'Motors are DISARMED. Arm with:  '
            'ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"')
        self.try_open()

    # ======================================================================
    # Serial port handling
    # ======================================================================
    def try_open(self):
        self.last_open_attempt = time.monotonic()
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0,
                                     write_timeout=0.05)
            self.ser.reset_input_buffer()
            self.rx_buffer = b''
            self.odom.last_l = None          # re-baseline ticks after (re)connect
            self.last_rx_time = time.monotonic()
            self.info_requested_at = time.monotonic()   # ask for 'i' shortly
            self.info_checked = False
            self.get_logger().info(f'Connected to Pico on {self.port}')
        except (serial.SerialException, OSError) as err:
            self.ser = None
            self.get_logger().warn(f'Cannot open {self.port}: {err}. Retrying...',
                                   throttle_duration_sec=5.0)

    def lost_connection(self, err):
        self.get_logger().error(f'Lost the Pico ({err}). Will keep retrying.')
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def write(self, text: str):
        if self.ser is None:
            return
        try:
            self.ser.write((text + '\n').encode('ascii'))
        except (serial.SerialException, OSError) as err:
            self.lost_connection(err)

    # ======================================================================
    # Commands from ROS
    # ======================================================================
    def on_cmd(self, twist):
        self.target_l, self.target_r = body_to_wheels(
            twist.linear.x, twist.angular.z, self.wheel_radius,
            self.wheel_separation, self.max_wheel_speed)
        self.last_cmd_time = time.monotonic()

    def send_tick(self):
        """Runs at send_rate (50 Hz). Always sends something, so the Pico's
        watchdog only fires if this node or the cable dies."""
        if self.ser is None:
            if time.monotonic() - self.last_open_attempt > 1.0:
                self.try_open()
            return

        if (self.info_requested_at is not None
                and time.monotonic() - self.info_requested_at > 0.5):
            self.info_requested_at = None
            self.write('i')          # the firmware replies with its settings

        fresh = (time.monotonic() - self.last_cmd_time) < self.cmd_timeout
        if fresh:
            left, right = self.target_l, self.target_r
            self.was_commanding = True
        else:
            left = right = 0.0
            if self.was_commanding:
                self.get_logger().info('cmd_vel went quiet -> sending zero speed')
                self.was_commanding = False
        self.write(f'v {left:.3f} {right:.3f}')

    def on_lift_cmd(self, msg):
        line = lift_command_line(msg.data)
        if line is None:
            self.get_logger().warn(f"Ignoring lift command '{msg.data}'. Use: home | park | "
                                   "stop | info | goto <mm> [speed]")
            return
        self.write(line)

    def on_arm(self, request, response):
        if request.data:
            self.target_l = self.target_r = 0.0     # never arm into a moving command
            self.last_cmd_time = 0.0
            self.write('a 1')
            response.message = ('Sent "a 1" (arm). The relay needs ~0.3 s; watch the '
                                'bridge log or /pico/status for ARMED.')
        else:
            self.write('v 0 0')
            self.write('a 0')
            response.message = 'Sent "a 0" (disarm).'
        response.success = self.ser is not None
        if not response.success:
            response.message = 'Pico not connected!'
        self.get_logger().info(response.message)
        return response

    def on_clear(self, request, response):
        self.write('c')
        response.success = self.ser is not None
        response.message = 'Sent "c" (clear latched faults).'
        return response

    # ======================================================================
    # Data from the Pico
    # ======================================================================
    def read_tick(self):
        if self.ser is None:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buffer += self.ser.read(waiting)
        except (serial.SerialException, OSError) as err:
            self.lost_connection(err)
            return

        # Split the buffer into complete lines; keep any half-line for later
        while b'\n' in self.rx_buffer:
            raw, self.rx_buffer = self.rx_buffer.split(b'\n', 1)
            self.last_rx_time = time.monotonic()
            line = raw.decode('ascii', errors='ignore')
            if self.ultrasonic.handle_line(line):     # Phase 7: ultrasonic
                continue
            msg = parse_line(line)
            if isinstance(msg, EncoderMsg):
                self.handle_encoder(msg)
            elif isinstance(msg, StatusMsg):
                self.handle_status(msg)
            elif isinstance(msg, LiftMsg):
                self.handle_lift(msg, line.strip())
            elif isinstance(msg, tuple):
                self.handle_text(msg[1])

        if len(self.rx_buffer) > 4096:           # garbage with no newlines
            self.rx_buffer = b''

    def handle_encoder(self, msg: EncoderMsg):
        # Time step, from the Pico's own clock if it sent one
        dt = 0.02
        if msg.micros is not None:
            if self.odom.last_micros is not None:
                dt = micros_delta(msg.micros, self.odom.last_micros) / 1e6
            self.odom.last_micros = msg.micros
        if dt <= 0 or dt > 0.5:
            dt = 0.02

        first_reading = self.odom.last_l is None
        step = self.odom.update(msg.ticks_l, msg.ticks_r, self.max_jump)
        if step is None:
            if not first_reading:
                self.get_logger().warn('Tick counters jumped: Pico restarted? '
                                       'Odometry continues from here.')
            return
        d_left, d_right = step
        self.wheel_pos_l += d_left
        self.wheel_pos_r += d_right

        # Wheel speeds. Phase 7: from the tick differences over the Pico's own
        # time step (twist_from_ticks, default). The Pico's filtered speed
        # readings showed up to 0.07 m/s while standing still (17 Oct 2026).
        if self.twist_from_ticks or msg.vel_l is None or msg.vel_r is None:
            vel_l, vel_r = d_left / dt, d_right / dt
        else:
            vel_l, vel_r = msg.vel_l, msg.vel_r
        v_x, w_z = wheels_to_body(vel_l, vel_r, self.wheel_radius,
                                  self.wheel_separation)

        # Self-check of ticks_per_wheel_rev
        self.checker.add(d_left, d_right, msg.vel_l, msg.vel_r, dt)
        ratio = self.checker.result()
        if ratio is not None and abs(ratio - 1.0) > 0.05:
            self.get_logger().warn(
                f'TICK SCALE CHECK: ticks say the wheels turned {ratio:.3f}x '
                f'as far as the Pico speed readings say. ticks_per_wheel_rev '
                f'is probably wrong - try {self.ticks_per_rev * ratio:.0f}.')
        elif ratio is not None:
            self.get_logger().info(f'Tick scale check OK (ratio {ratio:.3f})',
                                   throttle_duration_sec=30.0)

        self.publish_odometry(v_x, w_z, vel_l, vel_r)

    def publish_odometry(self, v_x, w_z, vel_l, vel_r):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self.odom.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom.x
        odom.pose.pose.position.y = self.odom.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = w_z
        # Covariance = "how much to trust each number". Big = don't trust.
        # We move in 2-D, so z, roll and pitch get huge values.
        odom.pose.covariance[0] = 0.01      # x
        odom.pose.covariance[7] = 0.01      # y
        odom.pose.covariance[14] = 1e6      # z
        odom.pose.covariance[21] = 1e6      # roll
        odom.pose.covariance[28] = 1e6      # pitch
        odom.pose.covariance[35] = 0.05     # yaw
        odom.twist.covariance[0] = 0.005    # vx
        odom.twist.covariance[7] = 0.005    # vy
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.03    # vyaw
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.odom.x
            t.transform.translation.y = self.odom.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        js = JointState()
        js.header.stamp = now
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.wheel_pos_l, self.wheel_pos_r]
        js.velocity = [vel_l, vel_r]
        self.joint_pub.publish(js)

    def handle_text(self, text: str):
        """'#' lines from the Pico. Also checks the firmware's settings
        (reply to 'i') against this node's parameters."""
        if text.startswith("ignoring 'v'"):
            # Normal while disarmed: the bridge keeps sending 'v' at 50 Hz.
            self.not_armed_msgs += 1
            if self.not_armed_msgs == 1:
                self.get_logger().info("[Pico] ignoring 'v' commands until armed "
                                       "(normal - further repeats hidden)")
            return
        if self.echo_pico_text:
            self.get_logger().info(f'[Pico] {text}')

        # Reply to 'i':  "gear=3.750  ticks per wheel rev=15360"
        if text.startswith('gear=') and 'ticks per wheel rev=' in text:
            try:
                fw_ticks = float(text.split('ticks per wheel rev=')[1].split()[0])
            except (ValueError, IndexError):
                return
            self.info_checked = True
            if abs(fw_ticks - self.ticks_per_rev) > 0.005 * fw_ticks:
                self.get_logger().error(
                    f'MISMATCH: the firmware says {fw_ticks:.0f} ticks per wheel rev, '
                    f'but ticks_per_wheel_rev = {self.ticks_per_rev:.0f} in base.yaml. '
                    'Odometry will be wrong - fix base.yaml.')
            else:
                self.get_logger().info(f'ticks_per_wheel_rev matches the firmware '
                                       f'({fw_ticks:.0f})')
        # Reply to 'i':  "limits: speed=9.00 rad/s accel=..."
        if text.startswith('limits: speed='):
            try:
                fw_max = float(text.split('speed=')[1].split()[0])
            except (ValueError, IndexError):
                return
            if abs(fw_max - self.max_wheel_speed) > 0.01:
                self.get_logger().warn(
                    f'max_wheel_speed is {self.max_wheel_speed} in base.yaml but '
                    f'{fw_max} in the firmware. Make them equal.')

    def handle_lift(self, msg: LiftMsg, raw: str):
        self.lift_pub.publish(String(data=raw))
        key = (msg.state, msg.homed, msg.fault)
        if key != self.last_lift_key:
            self.get_logger().info(f'Lift: {lift_text(msg)}')
            self.last_lift_key = key
        if msg.homed:                      # height is only meaningful once homed
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [self.column_joint]
            js.position = [msg.pos_mm / 1000.0]
            self.column_pub.publish(js)

    def handle_status(self, msg: StatusMsg):
        text = status_text(msg)
        self.status_pub.publish(String(data=text))

        state = status_summary(msg)
        self.state_pub.publish(String(data=state))
        # Log only when something other than CMD_TIMEOUT changes (like pico_console)
        key = (msg.flags & ~0x0004, msg.arm_state)
        if key != self.last_status_key:
            self.get_logger().info(f'Pico status: {text}  ->  /robot_state = {state}')
            self.last_status_key = key
        if msg.arm_state == 2:
            self.not_armed_msgs = 0      # show the message again after the next disarm

        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = msg.vbat
        b.present = msg.vbat > 5.0          # firmware: below 5 V = no battery (USB only)
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        b.percentage = float('nan')
        self.batt_pub.publish(b)

    # ======================================================================
    # Health
    # ======================================================================
    def health_tick(self):
        if self.ser is not None and time.monotonic() - self.last_rx_time > 1.0:
            self.get_logger().warn('No data from the Pico for over 1 s. '
                                   'Is the firmware running? Is something else '
                                   'using the port (Arduino IDE, miniterm)?',
                                   throttle_duration_sec=5.0)

    def shutdown(self):
        """Called on Ctrl+C: stop the wheels, optionally disarm."""
        if self.ser is not None:
            self.write('v 0 0')
            self.write('l s')              # stop the lift too
            if self.disarm_on_exit:
                self.write('a 0')
            time.sleep(0.05)
            try:
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PicoBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
