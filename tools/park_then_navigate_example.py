#!/usr/bin/env python3
"""
park_then_navigate_example.py - EXAMPLE of what your touchscreen code should do
when the patient picks a destination:

    1. call /head_column/park   (waits until the head is DOWN on the micro switch)
    2. send the destination to Nav2 and wait until the robot arrives
    3. publish "search" so the head lifts to find the patient's face again

Try it:
    python3 park_then_navigate_example.py pharmacy

Copy the go_to_destination() function into your touchscreen program and call
it when the "Go" button is pressed. Replace the coordinates in DESTINATIONS
with real ones from your map (in RViz use "Publish Point" or "2D Goal Pose"
and read the numbers from the terminal / `ros2 topic echo /goal_pose`).

Needs Nav2 running (nav2_simple_commander is part of Nav2).
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from std_srvs.srv import Trigger

# name: (x metres, y metres, heading in degrees) in the "map" frame  <- EDIT THESE
DESTINATIONS = {
    "reception": (0.0, 0.0, 0.0),
    "pharmacy":  (5.2, 1.0, 90.0),
    "radiology": (8.4, -3.5, 180.0),
}


def make_pose(nav, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def lower_head(nav):
    client = nav.create_client(Trigger, "head_column/park")
    if not client.wait_for_service(timeout_sec=5.0):
        nav.get_logger().error("head_column/park service not available - is the bridge running?")
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(nav, future, timeout_sec=120.0)
    result = future.result()
    if result is None or not result.success:
        nav.get_logger().error(f"Head did not park: {result.message if result else 'no reply'}")
        return False
    nav.get_logger().info(result.message)
    return True


def go_to_destination(nav, name):
    if name not in DESTINATIONS:
        nav.get_logger().error(f"Unknown destination '{name}'")
        return False

    # 1. head down first - never drive with the head raised
    if not lower_head(nav):
        return False

    # 2. drive there
    nav.goToPose(make_pose(nav, *DESTINATIONS[name]))
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.2)
    arrived = nav.getResult() == TaskResult.SUCCEEDED
    nav.get_logger().info("Arrived!" if arrived else f"Navigation ended: {nav.getResult()}")

    # 3. lift the head again to find the patient's face
    pub = nav.create_publisher(String, "head_column/command", 10)
    rclpy.spin_once(nav, timeout_sec=0.3)          # give the publisher a moment to connect
    pub.publish(String(data="search"))
    return arrived


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()      # assumes AMCL; if you localise differently, see the
                                   # "localizer" argument in the nav2_simple_commander docs
    destination = sys.argv[1] if len(sys.argv) > 1 else "pharmacy"
    go_to_destination(nav, destination)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
