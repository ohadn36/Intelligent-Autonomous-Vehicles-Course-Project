#!/usr/bin/env python3
"""
Smart exploration monitor — saves only when coverage is good enough.

Uses frontier count, unknown ratio, and total travel distance.
"""

import os
import subprocess
import sys
import time

import rospkg
import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32, Int32

try:
    sys.path.insert(0, os.path.join(rospkg.RosPack().get_path("summit_xl_autonomous_nav"), "scripts"))
except Exception:
    pass

from exploration_utils import coverage_stats


class ExplorationMonitor(object):
    def __init__(self):
        self.stable_seconds = rospy.get_param("~stable_seconds", 20.0)
        self.startup_grace = rospy.get_param("~startup_grace", 40.0)
        self.max_runtime = rospy.get_param("~max_runtime", 1200.0)
        self.max_unknown_ratio = rospy.get_param("~max_unknown_ratio", 0.06)
        self.min_map_width = rospy.get_param("~min_map_width", 30)
        self.min_map_height = rospy.get_param("~min_map_height", 30)

        self.start_wall = None
        self.saved = False
        self.map_msg = None
        self.frontier_count = 0
        self.unknown_ratio = 1.0
        self.explorer_complete = False
        self.bootstrap_complete = False
        self.robot_active = False
        self.last_progress_wall = None
        self.last_unknown_ratio = 1.0

        rospy.Subscriber("/exploration_bootstrap/complete", Bool, self.bootstrap_cb, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber("/structured_mapper/frontier_count", Int32, self.frontier_cb, queue_size=1)
        rospy.Subscriber("/structured_mapper/unknown_ratio", Float32, self.unknown_cb, queue_size=1)
        rospy.Subscriber("/structured_mapper/complete", Bool, self.complete_cb, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_cb, queue_size=1)

        rospy.Timer(rospy.Duration(2.0), self.tick)
        rospy.loginfo("Smart exploration monitor active.")

    def _mark_progress(self):
        self.last_progress_wall = time.time()

    def bootstrap_cb(self, msg):
        if not msg.data:
            return
        self.bootstrap_complete = True
        if self.start_wall is None:
            self.start_wall = time.time()
            self._mark_progress()

    def map_cb(self, msg):
        self.map_msg = msg

    def frontier_cb(self, msg):
        self.frontier_count = msg.data
        if msg.data > 0:
            self._mark_progress()

    def unknown_cb(self, msg):
        if msg.data < self.last_unknown_ratio - 0.005:
            self._mark_progress()
        self.last_unknown_ratio = msg.data
        self.unknown_ratio = msg.data

    def complete_cb(self, msg):
        if msg.data:
            self.explorer_complete = True

    def status_cb(self, msg):
        self.robot_active = any(
            st.status
            in (
                GoalStatus.ACTIVE,
                GoalStatus.PENDING,
                GoalStatus.PREEMPTING,
                GoalStatus.RECALLING,
            )
            for st in msg.status_list
        )

    def map_is_large_enough(self):
        if self.map_msg is None:
            return False
        return (
            self.map_msg.info.width >= self.min_map_width
            and self.map_msg.info.height >= self.min_map_height
        )

    def coverage_ok(self):
        ratio, _, known, bbox_total = coverage_stats(self.map_msg)
        self.unknown_ratio = ratio
        if bbox_total == 0 or known < 100:
            return False
        return ratio <= self.max_unknown_ratio

    def save_map(self, reason):
        if self.saved:
            return
        self.saved = True
        pkg_path = rospkg.RosPack().get_path("summit_xl_autonomous_nav")
        map_base = os.path.join(pkg_path, "maps", "summit_world")
        rospy.loginfo("Saving map (%s). Unknown in bbox: %.1f%%", reason, self.unknown_ratio * 100.0)
        try:
            subprocess.check_call(["rosrun", "map_server", "map_saver", "-f", map_base])
        except subprocess.CalledProcessError as exc:
            rospy.logerr("map_saver failed: %s", exc)
            self.saved = False
            return
        rospy.logwarn(
            "Map saved:\n  %s.yaml\n  %s.pgm\n"
            "Next: bash $(rospack find summit_xl_autonomous_nav)/scripts/start_demo.sh",
            map_base,
            map_base,
        )

    def tick(self, _event):
        if self.saved:
            return

        if not self.bootstrap_complete:
            if rospy.get_param("/exploration_bootstrap/complete", False):
                self.bootstrap_complete = True
                self.start_wall = time.time()
                self._mark_progress()
            else:
                return

        if self.start_wall is None:
            self.start_wall = time.time()
            self._mark_progress()

        elapsed = time.time() - self.start_wall
        if elapsed < self.startup_grace:
            return
        if not self.map_is_large_enough():
            return

        ratio, _, _, _ = coverage_stats(self.map_msg)
        self.unknown_ratio = ratio

        idle_for = 0.0
        if self.last_progress_wall is not None:
            idle_for = time.time() - self.last_progress_wall

        if self.explorer_complete and not self.robot_active:
            if self.coverage_ok():
                self.save_map("explorer complete + coverage OK")
            elif elapsed >= self.max_runtime:
                self.save_map("explorer complete + max runtime")
            else:
                rospy.logwarn_throttle(
                    15.0,
                    "Explorer finished but unknown still %.1f%% — waiting",
                    ratio * 100.0,
                )
            return

        if self.frontier_count == 0 and not self.robot_active and idle_for >= self.stable_seconds:
            if self.coverage_ok():
                self.save_map("no frontiers + coverage OK")
            elif elapsed >= self.max_runtime:
                self.save_map("max runtime with partial coverage")
            else:
                rospy.logwarn_throttle(
                    15.0,
                    "Idle with %.1f%% unknown remaining — not saving yet",
                    ratio * 100.0,
                )


def main():
    rospy.init_node("exploration_monitor")
    ExplorationMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
