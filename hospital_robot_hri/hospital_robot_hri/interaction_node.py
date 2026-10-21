#!/usr/bin/env python3
"""
interaction_node.py  --  ROS 2 Jazzy node "hri_interaction": Anna's behaviour.
All decisions are in interaction_logic.py; this file only connects them to ROS.

    /hri/touch              (std_msgs/String JSON)  ─┐  taps and menu choices from the screen
    /hri/speech/status      (std_msgs/String JSON)  ─┤  is she still talking?
    /head_column/state      (std_msgs/String)       ─┼─► [hri_interaction] ─► /hri/expression
    /lift/state             (std_msgs/String)       ─┤                     ─► /hri/speech/event
    /hri/interaction/command (std_msgs/String)      ─┘                     ─► /head_column/command
                                                       (wake | sleep | done)─► /hri/ui (JSON)
                                                                            ─► /hri/destination
                                                                            ─► /hri/interaction/state (JSON)

/head_column/* and /lift/state come from the Phase 8 head_column node and
pico_bridge (hospital_robot_base). The topic names are parameters, so the
simulated head (simulate_head) can be used instead for testing.

SAFETY: this node never arms the robot and never talks to the Pico directly.
It sends head_column the same commands you can type yourself
(search, hold, park). head_column and the Pico keep all their own limits.
The lift has no power unless a person armed the robot.
"""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from hospital_robot_hri.interaction_logic import InteractionLogic, InteractionParams
from hospital_robot_hri.nav_flow import NavInteractionLogic, NavParams  # Phase 7


class InteractionNode(Node):

    def __init__(self):
        super().__init__('hri_interaction')
        dp = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731

        p = InteractionParams()
        for name, default in vars(InteractionParams()).items():
            setattr(p, name, type(default)(dp(name, default)))
        nav = NavParams()
        for name, default in vars(NavParams()).items():
            setattr(nav, name, type(default)(dp(name, default)))
        # Phase 7: the Phase 9 flow + the trip (identical to InteractionLogic
        # while nav_available is false)
        self.logic = NavInteractionLogic(p, nav)
        self.logic.mission_seq = int(time.time()) % 100000 * 10   # unique trip ids
        ui_rate = float(dp('ui_rate', 1.0))

        self.pub = {
            'expression': self.create_publisher(String, 'hri/expression', 10),
            'say': self.create_publisher(String, 'hri/speech/event', 10),
            'head': self.create_publisher(String, dp('head_command_topic', 'head_column/command'), 10),
            'ui': self.create_publisher(String, 'hri/ui', 10),
            'destination': self.create_publisher(String, 'hri/destination', 10),
            'mission': self.create_publisher(String, dp('mission_command_topic',
                                                        'mission/command'), 10),
        }
        self.state_pub = self.create_publisher(String, 'hri/interaction/state', 10)
        self.create_subscription(String, 'hri/touch', self.on_touch, 10)
        self.create_subscription(String, 'hri/speech/status', self.on_speech, 10)
        self.create_subscription(String, dp('head_state_topic', 'head_column/state'),
                                 self.on_head, 10)
        self.create_subscription(String, dp('lift_state_topic', 'lift/state'), self.on_lift, 10)
        self.create_subscription(String, 'hri/interaction/command', self.on_command, 10)
        self.create_subscription(String, dp('mission_status_topic', 'mission/status'),
                                 self.on_mission, 10)               # Phase 7
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0 / max(0.2, ui_rate), self.republish)

        self.last = {}                       # last published value per action kind
        self.get_logger().info(
            'hri_interaction: Anna sleeps until the screen is tapped. Column adjustment '
            f'{"ON" if p.adjust_enabled else "OFF"}, navigation '
            f'{"ON" if p.nav_available else "not available yet (Phase 7)"}.')
        self.do(self.logic.start(time.monotonic()))

    # ------------------------------------------------------------------
    def do(self, actions):
        for kind, value in actions:
            if kind == 'log':
                self.get_logger().info(str(value))
                continue
            data = json.dumps(value) if isinstance(value, dict) else str(value)
            if kind in ('head', 'say', 'destination', 'mission'):
                self.get_logger().info(f'{kind}: {data}')
            self.pub[kind].publish(String(data=data))
            self.last[kind] = data
        if actions:
            self.publish_state()

    def republish(self):
        """The screen state and expression are re-sent every second, so a
        browser or hri_face that (re)starts later shows the right thing."""
        for kind in ('ui', 'expression'):
            if kind in self.last:
                self.pub[kind].publish(String(data=self.last[kind]))
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(self.logic.status())))

    # ------------------------------------------------------------------
    def on_touch(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            d = {'kind': str(msg.data).strip()}
        if not isinstance(d, dict):
            return
        t = time.monotonic()
        if d.get('kind') == 'tap':
            self.do(self.logic.on_tap(t))
        elif d.get('kind') == 'choice':
            self.do(self.logic.on_choice(t, d.get('value')))

    def on_speech(self, msg: String):
        try:
            self.logic.on_speech_status(time.monotonic(), json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def on_head(self, msg: String):
        self.do(self.logic.on_head_state(time.monotonic(), msg.data))

    def on_lift(self, msg: String):
        self.logic.on_lift_state(time.monotonic(), msg.data)

    def on_command(self, msg: String):
        self.do(self.logic.on_command(time.monotonic(), msg.data))

    def on_mission(self, msg: String):
        """Phase 7: /mission/status from hospital_robot_nav's mission node."""
        try:
            st = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.do(self.logic.on_mission_status(time.monotonic(), st))
    def tick(self):
        self.do(self.logic.tick(time.monotonic()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InteractionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
