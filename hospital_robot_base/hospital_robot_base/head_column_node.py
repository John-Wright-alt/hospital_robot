#!/usr/bin/env python3
"""
head_column_node.py  --  Phase 8 head-column behaviour (wiring rev 2).

    ESP32-S3 on the column ──USB /dev/headcam──►  'f' face lines ─┐
                                                                    ▼
    /robot_state (ARMED?) ─────────────────────────────►  [head_column]  ── /lift/command ──►  pico_bridge ──► Pico (lift.h)
    /lift/state  ('l' lines from the Pico) ────────────►                                         GP10-13, DRV8825

Topics out:  /head_column/state          std_msgs/String   e.g. TRACKING, PARKED, WAIT_ARM
             /head_column/parked         std_msgs/Bool     True = lift resting on the switch
             /head_column/face_detected  std_msgs/Bool
Topic in:    /head_column/command        std_msgs/String   search | park | stop | hold | home
Services:    /head_column/park    std_srvs/Trigger  answers only once PARKED (or failed)
             /head_column/search  std_srvs/Trigger

The lift is powered through the E-stop + relay, so it can only move while the
robot is ARMED. This node never arms anything: arming stays a human decision.

Tracking is "move and look": the lift makes a correction, stops, waits for a
camera frame taken AFTER it stopped, then decides the next correction. That
removes the camera/USB delay from the control loop, so it cannot overshoot.
"""

import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

import serial

from hospital_robot_base.pico_protocol import (
    LiftMsg, parse_line, parse_face_line, LIFT_IDLE, LIFT_FAULT, LIFT_FAULT_NAMES,
)


class HeadColumn(Node):

    def __init__(self):
        super().__init__('head_column')
        p = self.declare_parameter
        self.esp_port = p('esp_port', '/dev/headcam').value
        self.auto_home = p('auto_home_on_arm', True).value
        self.search_after_home = p('search_after_home', True).value
        self.soft_min = p('soft_min_mm', 5.0).value          # = LIFT_SOFT_MIN_MM in lift.h
        self.soft_max = p('soft_max_mm', 390.0).value        # = LIFT_SOFT_MAX_MM in lift.h
        self.search_speed = p('search_speed', 15.0).value
        self.track_speed = p('track_speed', 20.0).value
        self.gain = p('track_gain', 0.8).value
        self.deadband = p('deadband_mm', 15.0).value
        self.confirm_frames = p('confirm_frames', 2).value
        self.settle_frames = p('settle_frames', 2).value
        self.face_lost_timeout = p('face_lost_timeout', 15.0).value
        self.search_timeout = p('search_timeout', 60.0).value
        self.park_timeout = p('park_timeout', 90.0).value
        self.cmd_grace = 0.35      # s: ignore lift status this long after a command

        self.lock = threading.RLock()
        self.state = 'STARTING'
        self.state_since = time.monotonic()
        self.after_home = 'PARKED'
        self.armed = False
        self.lift = None                  # latest LiftMsg
        self.lift_time = 0.0
        self.cmd_time = 0.0
        self.sweep_up = True
        self.settle_frame = None
        self.last_lift_fault = 0

        # face data
        self.face = None                  # latest FaceMsg (found or not)
        self.face_streak = 0
        self.last_face_time = 0.0
        self.last_frame = -1
        self.last_face_line_time = 0.0

        # ESP serial
        self.ser = None
        self.rx = b''
        self.last_open_try = 0.0
        self.last_faces_on = 0.0

        self.state_pub = self.create_publisher(String, 'head_column/state', 10)
        self.parked_pub = self.create_publisher(Bool, 'head_column/parked', 10)
        self.face_pub = self.create_publisher(Bool, 'head_column/face_detected', 10)
        self.lift_cmd_pub = self.create_publisher(String, 'lift/command', 10)

        tick_group = MutuallyExclusiveCallbackGroup()
        srv_group = ReentrantCallbackGroup()
        self.create_subscription(String, 'lift/state', self.on_lift, 10, callback_group=tick_group)
        self.create_subscription(String, 'robot_state', self.on_robot_state, 10,
                                 callback_group=tick_group)
        self.create_subscription(String, 'head_column/command', self.on_command, 10,
                                 callback_group=tick_group)
        self.create_timer(0.02, self.read_esp, callback_group=tick_group)
        self.create_timer(0.1, self.tick, callback_group=tick_group)
        self.create_service(Trigger, 'head_column/park', self.srv_park, callback_group=srv_group)
        self.create_service(Trigger, 'head_column/search', self.srv_search, callback_group=srv_group)

        self.get_logger().info(f'head_column started. ESP32 camera on {self.esp_port}. '
                               'The lift moves only while the robot is ARMED.')

    # ================================================================ ESP32 camera
    def open_esp(self):
        self.last_open_try = time.monotonic()
        try:
            s = serial.Serial()
            s.port = self.esp_port
            s.baudrate = 115200
            s.timeout = 0
            s.dtr = False            # don't let the USB control lines reset the ESP32
            s.rts = False
            s.open()
            self.ser = s
            self.rx = b''
            self.last_face_line_time = time.monotonic()
            self.last_faces_on = 0.0
            self.get_logger().info(f'Connected to the head camera on {self.esp_port}')
        except (serial.SerialException, OSError) as err:
            self.ser = None
            self.get_logger().warn(f'Cannot open {self.esp_port}: {err} (retrying)',
                                   throttle_duration_sec=10.0)

    def esp_write(self, text):
        try:
            self.ser.write((text + '\n').encode('ascii'))
        except (serial.SerialException, OSError):
            self.esp_lost()

    def esp_lost(self):
        self.get_logger().error('Lost the head camera (USB). Retrying...')
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def read_esp(self):
        now = time.monotonic()
        if self.ser is None:
            if now - self.last_open_try > 2.0:
                self.open_esp()
            return
        # ask for face lines; repeat if they stop (e.g. the ESP32 rebooted)
        if now - self.last_face_line_time > 2.0 and now - self.last_faces_on > 2.0:
            self.last_faces_on = now
            self.esp_write('faces on')
            if self.ser is None:
                return
        try:
            n = self.ser.in_waiting
            if n:
                self.rx += self.ser.read(n)
        except (serial.SerialException, OSError):
            self.esp_lost()
            return
        while b'\n' in self.rx:
            raw, self.rx = self.rx.split(b'\n', 1)
            line = raw.decode('ascii', errors='ignore').strip()
            f = parse_face_line(line)
            if f is not None:
                self.on_face(f, now)
            elif line and not line.startswith('#'):
                self.get_logger().info(f'[ESP32] {line}', throttle_duration_sec=1.0)
        if len(self.rx) > 4096:
            self.rx = b''

    def on_face(self, f, now):
        with self.lock:
            self.last_face_line_time = now
            if f.frame < self.last_frame:          # ESP32 restarted: frame numbers reset
                self.settle_frame = None
            self.last_frame = f.frame
            self.face = f
            if f.found:
                self.face_streak += 1
                self.last_face_time = now
            else:
                self.face_streak = 0

    # ================================================================ inputs from ROS
    def on_lift(self, msg):
        m = parse_line(msg.data)
        if isinstance(m, LiftMsg):
            with self.lock:
                self.lift = m
                self.lift_time = time.monotonic()

    def on_robot_state(self, msg):
        with self.lock:
            self.armed = (msg.data == 'ARMED')

    def on_command(self, msg):
        ok, text = self.command(msg.data.strip().lower())
        (self.get_logger().info if ok else self.get_logger().warn)(text)

    # ================================================================ helpers
    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[head] {self.state} -> {s}')
            self.state = s
            self.state_since = time.monotonic()

    def lift_cmd(self, text):
        self.lift_cmd_pub.publish(String(data=text))
        self.cmd_time = time.monotonic()
        self.settle_frame = None

    def lift_ok(self):
        return self.lift is not None and time.monotonic() - self.lift_time < 1.0

    def lift_settled_idle(self):
        return (time.monotonic() - self.cmd_time > self.cmd_grace
                and self.lift_ok() and self.lift.state == LIFT_IDLE)

    def is_parked(self):
        return (self.state == 'PARKED' and self.lift_ok() and self.lift.homed
                and self.lift.switch)

    def start_home(self, after):
        self.after_home = after
        self.lift_cmd('home')
        self.set_state('HOMING')

    def start_search(self):
        self.face_streak = 0
        self.sweep_up = True
        self.lift_cmd(f'goto {self.soft_max:.1f} {self.search_speed:.1f}')
        self.set_state('SEARCHING')

    def command(self, cmd):
        """Returns (accepted, message). Used by the topic and the services."""
        with self.lock:
            if cmd not in ('search', 'park', 'stop', 'hold', 'home'):
                return False, f"Unknown head command '{cmd}'. Use search | park | stop | hold | home"
            if cmd == 'stop':
                self.lift_cmd('stop')
                self.set_state('STOPPED')
                return True, 'Lift stopped'
            if not self.armed:
                return False, 'Robot is not ARMED - the lift has no power'
            if not self.lift_ok():
                return False, 'No lift status from the Pico (is pico_bridge running?)'
            if cmd == 'home':
                self.start_home('PARKED')
                return True, 'Homing the lift'
            if cmd == 'park':
                if self.is_parked():
                    return True, 'Already parked'
                self.lift_cmd('park')
                self.set_state('PARKING')
                return True, 'Parking the lift'
            if self.state == 'FAULT':
                return False, "Lift is in FAULT - send 'home' (or 'park') first"
            if cmd == 'search':
                if not self.lift.homed:
                    self.start_home('SEARCHING')
                else:
                    self.start_search()
                return True, 'Searching for a face'
            # hold
            self.lift_cmd('stop')
            self.face_streak = 0
            self.set_state('WAITING')
            return True, 'Holding position, will track a face if one appears'

    # ================================================================ the state machine
    def tick(self):
        with self.lock:
            now = time.monotonic()
            self.step(now)
            face_now = (self.face is not None and self.face.found
                        and now - self.last_face_time < 1.0)
            self.state_pub.publish(String(data=self.state))
            self.parked_pub.publish(Bool(data=self.is_parked()))
            self.face_pub.publish(Bool(data=face_now))

    def step(self, now):
        if not self.armed:
            if self.state != 'WAIT_ARM':
                self.set_state('WAIT_ARM')
            return
        if not self.lift_ok():
            self.get_logger().warn('No lift status from the Pico - is pico_bridge running '
                                   'with firmware 1.1.0?', throttle_duration_sec=10.0)
            return
        lift = self.lift

        if lift.state == LIFT_FAULT and self.state != 'FAULT':
            self.set_state('FAULT')
            self.get_logger().error(f'Lift FAULT: {LIFT_FAULT_NAMES.get(lift.fault, lift.fault)}')
            return

        s = self.state
        if s in ('WAIT_ARM', 'STARTING'):
            if lift.homed:
                self.set_state('PARKED' if lift.switch else 'WAITING')
            elif self.auto_home:
                self.start_home('SEARCHING' if self.search_after_home else 'PARKED')
            else:
                self.set_state('STOPPED')
                self.get_logger().info("Lift not homed. Send 'home' or 'search' on /head_column/command")

        elif s in ('HOMING', 'PARKING'):
            if self.lift_settled_idle() and lift.homed and lift.switch:
                if s == 'HOMING' and self.after_home == 'SEARCHING':
                    self.start_search()
                else:
                    self.set_state('PARKED')
            elif now - self.state_since > self.park_timeout:
                self.get_logger().error(f'{s} took longer than {self.park_timeout:.0f} s')
                self.lift_cmd('stop')
                self.set_state('STOPPED')

        elif s == 'SEARCHING':
            if self.face_streak >= self.confirm_frames:
                self.lift_cmd('stop')
                self.set_state('TRACKING')
                self.get_logger().info('Face found - tracking')
            elif now - self.state_since > self.search_timeout:
                self.lift_cmd('stop')
                self.face_streak = 0
                self.set_state('WAITING')
                self.get_logger().info("No face found - holding (send 'search' to try again)")
            elif self.lift_settled_idle():
                self.sweep_up = not self.sweep_up
                end = self.soft_max if self.sweep_up else self.soft_min
                self.lift_cmd(f'goto {end:.1f} {self.search_speed:.1f}')

        elif s == 'TRACKING':
            if now - self.last_face_time > self.face_lost_timeout:
                self.get_logger().info('Face lost - searching again')
                self.start_search()
            else:
                self.move_and_look(lift)

        elif s == 'WAITING':
            if self.face_streak >= self.confirm_frames:
                self.set_state('TRACKING')

    def move_and_look(self, lift):
        if not self.lift_settled_idle():
            self.settle_frame = None                 # still moving
            return
        if self.settle_frame is None:
            self.settle_frame = self.last_frame      # just stopped: remember this frame
            return
        f = self.face
        if f is None or f.frame < self.settle_frame + self.settle_frames:
            return                                   # wait for a picture taken after stopping
        if not f.found or abs(f.error_mm) < self.deadband:
            return                                   # no face in this frame, or centred
        target = lift.pos_mm + self.gain * f.error_mm
        target = max(self.soft_min, min(self.soft_max, target))
        if abs(target - lift.pos_mm) < 1.0:
            return                                   # already at the limit
        self.lift_cmd(f'goto {target:.1f} {self.track_speed:.1f}')

    # ================================================================ services
    def srv_park(self, request, response):
        del request
        ok, text = self.command('park')
        if not ok:
            response.success, response.message = False, text
            self.get_logger().error(f'Park refused: {text}')
            return response
        time.sleep(0.5)
        deadline = time.monotonic() + self.park_timeout
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if self.is_parked():
                    response.success, response.message = True, 'Head column parked on the switch'
                    return response
                if self.state in ('FAULT', 'WAIT_ARM', 'STOPPED'):
                    response.success = False
                    response.message = f'Park failed: head column is {self.state}'
                    return response
            time.sleep(0.1)
        response.success, response.message = False, 'Park timed out'
        return response

    def srv_search(self, request, response):
        del request
        response.success, response.message = self.command('search')
        return response

    def shutdown(self):
        if self.ser is not None:
            try:
                self.esp_write('faces off')
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = HeadColumn()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
