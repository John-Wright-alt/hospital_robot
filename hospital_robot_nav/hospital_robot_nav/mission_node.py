#!/usr/bin/env python3
"""
mission_node.py  --  ROS 2 Jazzy node "mission": drives the robot to a NAMED
place with Nav2. All decisions are in mission_logic.py; this file only
connects them to ROS.

    /mission/command  (std_msgs/String JSON) ─┐
    /robot_state      (std_msgs/String)      ─┼─► [mission] ─► NavigateToPose action (Nav2)
    /head_column/state(std_msgs/String)      ─┘            ─► /speed_limit (nav2_msgs/SpeedLimit)
                                                           ─► /head_column/command ('park' only)
                                                           ─► /mission/status (std_msgs/String JSON)

Commands (the touchscreen flow sends these; you can too, for testing):
    ros2 run hospital_robot_nav go_to pharmacy              # guide mode (slow)
    ros2 run hospital_robot_nav go_to home --mode alone
    ros2 run hospital_robot_nav go_to --cancel

The place names come from the waypoint file (re-read on every 'go', so a
newly saved waypoint works without restarting anything).

SAFETY: this node NEVER arms the robot. It refuses to start unless
/robot_state says ARMED and the head column says PARKED, and cancels the goal
the moment either stops being true. The Pico's watchdog, speed clamp and the
E-stop work exactly as before, whatever this node does.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from hospital_robot_nav import waypoints as wp
from hospital_robot_nav.mission_logic import MissionLogic, MissionParams


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        dp = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        p = MissionParams()
        for name, default in vars(MissionParams()).items():
            setattr(p, name, type(default)(dp(name, default)))
        self.logic = MissionLogic(p)
        self.wp_file = dp('waypoints_file', '~/robot_ws/maps/apartment/waypoints.yaml')

        self.nav = ActionClient(self, NavigateToPose, dp('nav_action', 'navigate_to_pose'))
        self.goal_handle = None
        self.goal_seq = 0                       # ignores answers about older goals

        self.status_pub = self.create_publisher(String, 'mission/status', 10)
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.head_pub = self.create_publisher(String, dp('head_command_topic',
                                                         'head_column/command'), 10)
        self.create_subscription(String, 'mission/command', self.on_command, 10)
        self.create_subscription(String, dp('robot_state_topic', 'robot_state'),
                                 self.on_robot_state, 10)
        self.create_subscription(String, dp('head_state_topic', 'head_column/state'),
                                 self.on_head_state, 10)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.republish)
        self.speed = 0.0

        try:
            places = wp.load(self.wp_file)
            miss = wp.missing(places)
            self.get_logger().info(f'mission: waypoints {", ".join(sorted(places))} from '
                                   f'{self.wp_file}' + (f'. MISSING: {", ".join(miss)}'
                                                        if miss else ''))
        except wp.WaypointError as err:
            self.get_logger().warn(f'mission: {err}. Record waypoints with save_waypoint.')
        self.get_logger().info(
            f'mission ready: guide {p.guide_speed} m/s, alone {p.alone_speed} m/s. '
            f'Requires ARMED{" and PARKED" if p.require_parked else ""}. Never arms by itself.')

    # ------------------------------------------------------------------ actions
    def do(self, actions):
        for kind, value in actions:
            if kind == 'log':
                self.get_logger().info(str(value))
            elif kind == 'status':
                self.status_pub.publish(String(data=json.dumps(value)))
                self.get_logger().info(f'status: {value["state"]} {value["target"]} ({value["mode"]})'
                    + (f' - {value["reason"]}' if value['reason'] else ''))
            elif kind == 'speed':
                self.speed = float(value)
                self.publish_speed()
            elif kind == 'head':
                self.head_pub.publish(String(data=str(value)))
                self.get_logger().info(f'head_column <- {value}')
            elif kind == 'goal':
                self.send_goal(value)
            elif kind == 'cancel':
                self.cancel_goal()

    def publish_speed(self):
        m = SpeedLimit()
        m.header.stamp = self.get_clock().now().to_msg()
        m.percentage = False
        m.speed_limit = float(self.speed)
        self.speed_pub.publish(m)

    def send_goal(self, g):
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_handle = None
        if not self.nav.wait_for_server(timeout_sec=1.0):
            self.do(self.logic.on_goal_response(time.monotonic(), False))
            self.get_logger().error('Nav2 action server navigate_to_pose not available. '
                                    'Is navigation.launch.py running and active?')
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # stamp 0 = 'use the latest transform' (no extrapolation errors)
        pose.pose.position.x = float(g['x'])
        pose.pose.position.y = float(g['y'])
        pose.pose.orientation.z = math.sin(g['yaw'] / 2.0)
        pose.pose.orientation.w = math.cos(g['yaw'] / 2.0)
        goal.pose = pose
        self.get_logger().info(f'NavigateToPose x={g["x"]:.2f} y={g["y"]:.2f} '
                               f'yaw={math.degrees(g["yaw"]):.0f} deg, limit {g["speed"]:.2f} m/s')
        fut = self.nav.send_goal_async(goal, feedback_callback=lambda fb, s=seq: self.on_feedback(s, fb))
        fut.add_done_callback(lambda f, s=seq: self.on_goal_response(s, f))

    def on_goal_response(self, seq, future):
        try:
            handle = future.result()
        except Exception as err:                              # noqa: BLE001
            self.get_logger().error(f'goal request failed: {err}')
            handle = None
        if seq != self.goal_seq:
            # an older goal that was cancelled/replaced before Nav2 answered:
            # if Nav2 accepted it anyway, cancel it so it can never drive
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
                self.get_logger().info('cancelled a late-accepted old goal')
            return
        accepted = handle is not None and handle.accepted
        if accepted:
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f, s=seq: self.on_result(s, f))
        self.do(self.logic.on_goal_response(time.monotonic(), accepted))

    def on_feedback(self, seq, fb):
        if seq == self.goal_seq:
            self.logic.on_feedback(time.monotonic(), float(fb.feedback.distance_remaining))

    def on_result(self, seq, future):
        if seq != self.goal_seq:
            return                         # result of an older, replaced goal
        try:
            code = future.result().status
        except Exception:                                     # noqa: BLE001
            code = GoalStatus.STATUS_ABORTED
        self.goal_handle = None
        self.do(self.logic.on_result(time.monotonic(), code))

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.get_logger().info('cancel sent to Nav2')
        else:
            # goal not accepted yet: make any late acceptance harmless
            # (on_goal_response cancels it if Nav2 accepts it after all)
            self.goal_seq += 1
            if self.logic.state == 'CANCELING':
                self.do(self.logic.on_result(time.monotonic(), GoalStatus.STATUS_CANCELED))

    # ------------------------------------------------------------------ inputs
    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except (ValueError, TypeError):
            words = str(msg.data).split()        # also accept plain "go pharmacy" / "cancel"
            cmd = {'cmd': words[0], 'target': words[1] if len(words) > 1 else ''} if words else {}
        places = {}
        if isinstance(cmd, dict) and str(cmd.get('cmd', '')).lower() == 'go':
            try:
                places = wp.load(self.wp_file)
            except wp.WaypointError as err:
                self.get_logger().error(str(err))
        self.do(self.logic.on_command(time.monotonic(), cmd, places))

    def on_robot_state(self, msg):
        self.logic.on_robot_state(time.monotonic(), msg.data)

    def on_head_state(self, msg):
        self.logic.on_head_state(time.monotonic(), msg.data)

    def tick(self):
        self.do(self.logic.tick(time.monotonic()))

    def republish(self):
        st = dict(self.logic.last_status)
        st['state'] = self.logic.state if self.logic.state != 'IDLE' else st['state']
        if self.logic.state == 'ACTIVE' and self.logic.remaining is not None:
            st['remaining_m'] = round(self.logic.remaining, 2)
        self.status_pub.publish(String(data=json.dumps(st)))
        if self.logic.state in ('STARTING', 'ACTIVE'):
            self.publish_speed()               # late-joining controller still gets the limit

    def stop(self):
        if self.goal_handle is not None:
            self.get_logger().info('shutting down: cancelling the active goal')
            self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
