#!/usr/bin/env python3
"""
Post-scan control hand-off (run this in its own terminal).

Flow:
  1. Wait until the autonomous scan reports complete (or its 200s cap).
  2. Ask: keep mapping MANUALLY (drive it yourself) or finish?
       - YES -> launch keyboard teleop; gmapping keeps building the map while
                you drive to any spots you want. Ctrl+C teleop when done.
       - NO  -> skip straight to saving.
  3. Save the map to maps/summit_world.{pgm,yaml}.
  4. Print exactly how to start the autonomous navigation phase (pick a point
     on the map; the robot plans the shortest collision-free path and drives).

Mapping (gmapping) and navigation (map_server + AMCL) are different ROS graphs,
so navigation is started as a separate launch after the mapping launch is
stopped - this node prints that command for you.
"""

import os
import subprocess
import sys

import rospy
import rospkg
from std_msgs.msg import Bool

try:
    import rospkg as _rospkg
    sys.path.insert(
        0,
        os.path.join(_rospkg.RosPack().get_path("summit_xl_autonomous_nav"), "scripts"),
    )
except Exception:
    pass

from exploration_utils import resolve_cmd_vel_topic

PKG = "summit_xl_autonomous_nav"


class MappingControl(object):
    def __init__(self):
        self.complete = False
        rospy.Subscriber(
            "/structured_mapper/complete", Bool,
            self._complete_cb, queue_size=1)

    def _complete_cb(self, msg):
        if msg.data:
            self.complete = True

    def _map_base(self):
        try:
            return os.path.join(rospkg.RosPack().get_path(PKG), "maps", "summit_world")
        except Exception:
            return "summit_world"

    def wait_for_complete(self):
        rospy.loginfo(
            "Waiting for the autonomous scan to finish (or its 200s cap)...")
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
        topic = resolve_cmd_vel_topic()
        print("\n=== MANUAL MAPPING ===")
        print("Drive the robot with the keyboard to map any spots you want.")
        print("gmapping keeps building the map as you drive.")
        print("Press Ctrl+C in THIS terminal when you are done driving.\n")
        cmd = [
            "rosrun", "teleop_twist_keyboard", "teleop_twist_keyboard.py",
            "cmd_vel:=%s" % topic,
        ]
        try:
            subprocess.call(cmd)
        except KeyboardInterrupt:
            pass
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

        print("\nNext step - autonomous navigation to a point you pick:")
        print("  1) Stop the mapping launch (Ctrl+C in its terminal).")
        print("  2) roslaunch %s kobuki_go_to_goal.launch" % PKG)
        print("  3) In RViz: click 2D Pose Estimate (set where the robot is),")
        print("     then 2D Nav Goal (or Publish Point) to choose a goal.")
        print("     The robot plans the shortest collision-free path there.\n")


def main():
    rospy.init_node("mapping_control")
    MappingControl().run()


if __name__ == "__main__":
    main()
