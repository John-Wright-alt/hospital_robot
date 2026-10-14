#!/usr/bin/env python3
"""
face_server_node.py  --  ROS 2 Jazzy node "hri_face": serves the animated eyes
as a web page and feeds them live data.

    /hri/gaze/status  (std_msgs/String, JSON) ──┐
    /hri/expression   (std_msgs/String)       ──┼─► [hri_face] ──► http://<robot>:8080/   (the eyes)
                                                │              └─► /hri/face/state (std_msgs/String, JSON)

Open the eyes in any browser on the same network:
    http://robot.local:8080/          patient view
    http://robot.local:8080/?dev=1    with the test panel (this browser only)
Later the touchscreen runs the same page in kiosk mode: http://localhost:8080/

Change the expression from a terminal (the Phase 6 interaction manager will do this):
    ros2 topic pub --once /hri/expression std_msgs/msg/String "{data: happy}"
Valid: idle curious listening speaking thinking happy waiting confused assistance sleeping

This node commands NOTHING on the robot. It only displays.
"""

import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from hospital_robot_hri.face_http import FaceHttpServer
from hospital_robot_hri.face_logic import EXPRESSIONS, FaceLogic, FaceParams


class FaceServerNode(Node):

    def __init__(self):
        super().__init__('hri_face')
        dp = self.declare_parameter
        host = dp('host', '0.0.0.0').value
        port = int(dp('port', 8080).value)
        web_dir = dp('web_dir', '').value or os.path.join(
            get_package_share_directory('hospital_robot_hri'), 'web')
        default_expr = dp('default_expression', 'idle').value
        rate = float(dp('publish_rate', 20.0).value)

        p = FaceParams()
        for name, default in vars(FaceParams()).items():
            setattr(p, name, type(default)(dp(name, default).value))
        self.logic = FaceLogic(p, default_expr)

        self.http = FaceHttpServer(web_dir, host, port)
        self.http.start()

        self.state_pub = self.create_publisher(String, 'hri/face/state', 10)
        self.create_subscription(String, 'hri/gaze/status', self.on_gaze, 10)
        self.create_subscription(String, 'hri/expression', self.on_expression, 10)
        self.create_timer(1.0 / rate, self.tick)

        self.last_key = None
        self.last_pub = 0.0
        self.bad_json = 0
        self.get_logger().info(
            f'hri_face serving the eyes on http://{host}:{port}/  '
            f'(from the PC: http://robot.local:{port}/?dev=1). '
            f'Expression: {self.logic.base}.')

    # ------------------------------------------------------------------
    def on_gaze(self, msg: String):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            self.bad_json += 1
            self.get_logger().warn('Ignoring a /hri/gaze/status message that is not JSON',
                                   throttle_duration_sec=10.0)
            return
        self.logic.on_gaze_status(status, time.monotonic())

    def on_expression(self, msg: String):
        if self.logic.set_expression(msg.data):
            self.get_logger().info(f'expression -> {self.logic.base}')
        else:
            self.get_logger().warn(f'Unknown expression "{msg.data}". '
                                   f'Valid: {" ".join(EXPRESSIONS)}')

    def tick(self):
        now = time.monotonic()
        state = self.logic.update(now)
        self.http.publish(state)
        # /hri/face/state only on change (plus 1 Hz), so it is easy to read / record
        key = (state['expression'], state['gaze']['source'], state['gaze_data'],
               round(state['gaze']['x'], 2))
        if key != self.last_key or now - self.last_pub >= 1.0:
            self.state_pub.publish(String(data=json.dumps(state)))
            self.last_key, self.last_pub = key, now

    def shutdown(self):
        self.http.stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FaceServerNode()
        rclpy.spin(node)
    except OSError as err:
        # Typical: port 8080 already in use (another hri_face still running)
        print(f'[hri_face] could not start the web server: {err}. '
              'Is another hri_face already running? Try: pkill -f face_server_node')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
