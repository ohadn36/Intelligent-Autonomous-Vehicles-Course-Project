#!/usr/bin/env python3
"""
Post-scan control hand-off (interactive).

Flow:
  1. Wait until the autonomous scan reports complete (or its time cap).
  2. Ask: keep mapping MANUALLY (drive it yourself) or finish?
  3. Save the map to maps/summit_world.{pgm,yaml}.
  4. (standalone mode) print how to start the navigation phase.

Manual driving suspends move_base (killed at handoff) so cmd_vel is exclusive.
Press 'c' to toggle corridor-assist for narrow passages (e.g. kitchen entrance).
"""

import os
import subprocess
import sys
import termios
import threading
import tty

import actionlib
import rospy
import rospkg
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

try:
    sys.path.insert(
        0,
        os.path.join(
            rospkg.RosPack().get_path("summit_xl_autonomous_nav"), "scripts"),
    )
except Exception:
    pass

from exploration_utils import (
    front_clearance,
    resolve_cmd_vel_topic,
    side_clearances,
)

PKG = "summit_xl_autonomous_nav"


class MappingControl(object):
    def __init__(self):
        self.complete = False
        self.print_next_steps = rospy.get_param("~print_next_steps", True)
        self.manual_linear = rospy.get_param("~manual_linear", 0.4)
        self.manual_angular = rospy.get_param("~manual_angular", 0.9)
        self.corridor_linear = rospy.get_param("~corridor_linear", 0.22)
        self.corridor_front_min = rospy.get_param("~corridor_front_min", 0.22)
        self.scan_topic = rospy.get_param("~scan_topic", "/kobuki/laser/scan")
        self.latest_scan = None
        rospy.Subscriber(
            "/structured_mapper/complete", Bool,
            self._complete_cb, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _complete_cb(self, msg):
        if msg.data:
            self.complete = True

    def _map_base(self):
        try:
            return os.path.join(
                rospkg.RosPack().get_path(PKG), "maps", "summit_world")
        except Exception:
            return "summit_world"

    def wait_for_complete(self):
        rospy.loginfo("Waiting for the autonomous scan to finish...")
        rate = rospy.Rate(2)
        while not rospy.is_shutdown() and not self.complete:
            rate.sleep()

    def _ensure_move_base_stopped(self):
        """Stop move_base so it cannot fight manual cmd_vel."""
        try:
            client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            if client.wait_for_server(rospy.Duration(0.5)):
                client.cancel_all_goals()
        except Exception:
            pass
        try:
            subprocess.call(["rosnode", "kill", "/move_base"])
            rospy.sleep(0.4)
            print("move_base stopped — you have full manual control.")
        except (OSError, subprocess.SubprocessError):
            pass

    def save_map(self):
        base = self._map_base()
        print("\nSaving map to %s.{pgm,yaml} ..." % base)
        try:
            subprocess.check_call(
                ["rosrun", "map_server", "map_saver", "-f", base])
            print("Map saved.")
            return True
        except (subprocess.CalledProcessError, OSError) as exc:
            print("ERROR: map_saver failed: %s" % exc)
            print("Make sure gmapping is still running (it publishes /map).")
            return False

    def _apply_corridor_twist(self, twist, linear_speed):
        """Center in a narrow gap using left/right laser clearance."""
        scan = self.latest_scan
        if scan is None:
            twist.linear.x = linear_speed
            twist.angular.z = 0.0
            return
        left, right, front = side_clearances(scan)
        if front < self.corridor_front_min:
            twist.linear.x = 0.0
            twist.angular.z = max(-0.4, min(0.4, 0.9 * (left - right)))
            return
        twist.linear.x = linear_speed
        twist.angular.z = max(-0.35, min(0.35, 0.85 * (left - right)))

    def manual_mapping(self):
        """Keyboard teleop while gmapping maps. 'c' = corridor-assist mode."""
        self._ensure_move_base_stopped()
        topic = resolve_cmd_vel_topic()
        pub = rospy.Publisher(topic, Twist, queue_size=1)
        twist = Twist()
        state = {"running": True}
        corridor_mode = {"on": False}

        def publish_loop():
            rate = rospy.Rate(15)
            while state["running"] and not rospy.is_shutdown():
                pub.publish(twist)
                rate.sleep()

        print("\n=== MANUAL MAPPING (drive on %s) ===" % topic)
        print("  i = forward   , = back   j = left   l = right")
        print("  k / space = stop")
        print("  c = toggle CORRIDOR ASSIST (for narrow passages / kitchen)")
        print("  q / Ctrl+C = finish manual mapping")
        print("gmapping keeps building the map as you drive.\n")

        thread = threading.Thread(target=publish_loop)
        thread.daemon = True
        thread.start()

        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        lin = self.manual_linear
        ang = self.manual_angular
        try:
            tty.setraw(fd)
            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)
                if ch in ("q", "\x03"):
                    break
                if ch == "c":
                    corridor_mode["on"] = not corridor_mode["on"]
                    status = "ON (centering in gaps)" if corridor_mode["on"] else "OFF"
                    print("\rCorridor assist: %s          " % status)
                    continue
                if ch == "i":
                    if corridor_mode["on"]:
                        self._apply_corridor_twist(twist, self.corridor_linear)
                    else:
                        twist.linear.x, twist.angular.z = lin, 0.0
                elif ch == ",":
                    scan = self.latest_scan
                    if scan is not None and front_clearance(scan) < 0.35:
                        twist.linear.x, twist.angular.z = 0.0, 0.0
                    else:
                        twist.linear.x, twist.angular.z = -lin * 0.7, 0.0
                elif ch == "j":
                    twist.linear.x = 0.0
                    twist.angular.z = ang * (0.6 if corridor_mode["on"] else 1.0)
                elif ch == "l":
                    twist.linear.x = 0.0
                    twist.angular.z = -ang * (0.6 if corridor_mode["on"] else 1.0)
                elif ch in ("k", " "):
                    twist.linear.x, twist.angular.z = 0.0, 0.0
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
            state["running"] = False
            twist.linear.x, twist.angular.z = 0.0, 0.0
            pub.publish(twist)
            rospy.sleep(0.2)
        print("\nManual mapping finished.")

    @staticmethod
    def _ask(question):
        try:
            return input(question).strip().lower()
        except (EOFError, KeyboardInterrupt):
            return "n"

    def run(self):
        self.wait_for_complete()
        print("\n" + "=" * 60)
        print(" AUTONOMOUS SCAN COMPLETE - control is yours.")
        print("=" * 60)
        manual_q = "Keep mapping MANUALLY (drive it yourself)? [y/N]: "
        if self._ask(manual_q) in ("y", "yes"):
            self.manual_mapping()
            if self._ask("Save the map now? [Y/n]: ") in ("", "y", "yes"):
                self.save_map()
        else:
            self.save_map()

        if self.print_next_steps:
            print("\nNext step - autonomous navigation to a point you pick:")
            print("  1) Stop the mapping launch (Ctrl+C in its terminal).")
            print("  2) roslaunch %s kobuki_go_to_goal.launch" % PKG)
            print("  3) In RViz: 2D Pose Estimate, then 2D Nav Goal.")
        else:
            print("\nHanding off to the navigation phase...")


def main():
    rospy.init_node("mapping_control")
    MappingControl().run()


if __name__ == "__main__":
    main()
