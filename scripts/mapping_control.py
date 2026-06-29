#!/usr/bin/env python3
"""
Post-scan control hand-off (interactive).

Flow:
  1. Wait until the autonomous scan reports complete (or its time cap).
  2. Ask: keep mapping MANUALLY (drive it yourself) or finish?
       - YES -> built-in keyboard teleop; gmapping keeps building the map while
                you drive. Press q (or Ctrl+C) to stop driving.
       - NO  -> skip straight to saving.
  3. Save the map to maps/summit_world.{pgm,yaml}.
  4. (standalone mode) print how to start the navigation phase.

Used both standalone and from run_demo.sh, which orchestrates the launches so
everything happens in a single terminal.
"""

import os
import subprocess
import sys
import termios
import threading
import tty

import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

try:
    sys.path.insert(
        0,
        os.path.join(
            rospkg.RosPack().get_path("summit_xl_autonomous_nav"), "scripts"),
    )
except Exception:
    pass

from exploration_utils import resolve_cmd_vel_topic

PKG = "summit_xl_autonomous_nav"


class MappingControl(object):
    def __init__(self):
        self.complete = False
        self.print_next_steps = rospy.get_param("~print_next_steps", True)
        self.manual_linear = rospy.get_param("~manual_linear", 0.4)
        self.manual_angular = rospy.get_param("~manual_angular", 0.9)
        rospy.Subscriber(
            "/structured_mapper/complete", Bool,
            self._complete_cb, queue_size=1)

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

    def manual_mapping(self):
        """Built-in keyboard teleop so the user can drive while gmapping maps.

        Runs in raw-tty mode, so Ctrl+C is read as a normal character (quit)
        rather than a signal - the orchestrating shell is never interrupted.
        """
        topic = resolve_cmd_vel_topic()
        pub = rospy.Publisher(topic, Twist, queue_size=1)
        twist = Twist()
        state = {"running": True}

        def publish_loop():
            rate = rospy.Rate(10)
            while state["running"] and not rospy.is_shutdown():
                pub.publish(twist)
                rate.sleep()

        print("\n=== MANUAL MAPPING (drive on %s) ===" % topic)
        print("  i = forward   , = back   j = turn left   l = turn right")
        print("  k or space = stop        q or Ctrl+C = finish driving")
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
                if ch in ("q", "\x03"):  # q or Ctrl+C
                    break
                if ch == "i":
                    twist.linear.x, twist.angular.z = lin, 0.0
                elif ch == ",":
                    twist.linear.x, twist.angular.z = -lin, 0.0
                elif ch == "j":
                    twist.linear.x, twist.angular.z = 0.0, ang
                elif ch == "l":
                    twist.linear.x, twist.angular.z = 0.0, -ang
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
