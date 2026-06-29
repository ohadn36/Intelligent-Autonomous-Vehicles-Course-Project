#!/usr/bin/env python3
"""
Convert RViz map clicks (/clicked_point) into move_base navigation goals.

Includes stuck recovery for narrow passages: when DWA cannot execute a global
plan (common in ~0.85 m doorways with high inflation), detects lack of progress
and runs laser-guarded back-up / align / corridor creep before retrying.
"""

import math
import os
import sys

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler

try:
    import rospkg
    sys.path.insert(
        0, os.path.join(rospkg.RosPack().get_path("summit_xl_autonomous_nav"), "scripts")
    )
except Exception:
    pass

from exploration_utils import (
    best_open_bearing,
    cell_region_free,
    creep_through_passage,
    doorway_opening,
    drive_cmd,
    front_clearance,
    get_robot_pose,
    map_indices,
    map_to_world,
    nearest_free_cell,
    rear_clearance,
    rear_in_fov,
    resolve_cmd_vel_topic,
    side_clearances,
    stop_robot,
)
TF_EXC = (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ClickGoalNode(object):
    def __init__(self):
        self.default_yaw = rospy.get_param("~default_yaw", 0.0)
        self.stuck_time = rospy.get_param("~stuck_time", 5.0)
        self.stuck_distance = rospy.get_param("~stuck_distance", 0.10)
        self.max_recoveries = int(rospy.get_param("~max_recoveries", 4))
        self.goal_clearance = rospy.get_param("~goal_clearance", 0.28)
        self.recovery_creep_speed = rospy.get_param("~recovery_creep_speed", 0.10)
        self.recovery_creep_duration = rospy.get_param("~recovery_creep_duration", 5.0)
        self.recovery_back_speed = rospy.get_param("~recovery_back_speed", 0.10)
        self.recovery_back_dist = rospy.get_param("~recovery_back_dist", 0.25)
        self.recovery_opening_max = rospy.get_param("~recovery_opening_max", 1.35)
        scan_topic = rospy.get_param("~scan_topic", "/kobuki/laser/scan")

        self.map_msg = None
        self.latest_scan = None
        self.goal_xy = None
        self.recoveries = 0
        self.progress_origin = None
        self.progress_time = None
        self.active = False
        self.recovering = False

        self.tf_listener = tf.TransformListener()
        self.cmd_topic = resolve_cmd_vel_topic(rospy.get_param("~cmd_vel_topic", ""))
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.Subscriber("/map", OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber(scan_topic, LaserScan, self._scan_cb, queue_size=1)

        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo(
            "move_base ready. Click a point in RViz (Publish Point). "
            "Stuck recovery: up to %d attempts.", self.max_recoveries)
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self._watchdog_cb)

    def _map_cb(self, msg):
        self.map_msg = msg

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _get_scan(self):
        return self.latest_scan

    def robot_xy_yaw(self):
        return get_robot_pose(self.tf_listener, "map", "base_footprint")

    def _clearance_cells(self):
        if self.map_msg is None:
            return 2
        return max(2, int(round(self.goal_clearance / self.map_msg.info.resolution)))

    def snap_goal(self, x, y):
        """Snap a clicked point to a footprint-clear cell on the static map."""
        if self.map_msg is None:
            return x, y
        mx, my = map_indices(self.map_msg, x, y)
        clearance = self._clearance_cells()
        if cell_region_free(self.map_msg, mx, my, clearance):
            return x, y
        free = nearest_free_cell(
            self.map_msg, mx, my, max_radius=20, clearance_cells=clearance)
        if free is None:
            rospy.logwarn(
                "Goal (%.2f, %.2f) is inside an obstacle — no clear cell nearby.",
                x, y)
            return None, None
        sx, sy = map_to_world(self.map_msg, free[0], free[1])
        if math.hypot(sx - x, sy - y) > 0.01:
            rospy.loginfo(
                "Snapped goal (%.2f, %.2f) -> (%.2f, %.2f) for clearance.",
                x, y, sx, sy)
        return sx, sy

    def clicked_point_cb(self, msg):
        sx, sy = self.snap_goal(msg.point.x, msg.point.y)
        if sx is None:
            return
        self.begin_goal(sx, sy)

    def begin_goal(self, x, y):
        """Start a fresh navigation goal (resets recovery counter)."""
        self.client.cancel_all_goals()
        stop_robot(self.cmd_pub)
        self.goal_xy = (x, y)
        self.recoveries = 0
        self.active = True
        self.recovering = False
        self._mark_progress()
        self._dispatch_move_base()

    def _make_goal_msg(self):
        x, y = self.goal_xy
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, self.default_yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def _dispatch_move_base(self):
        x, y = self.goal_xy
        rospy.loginfo("Goal sent to (%.2f, %.2f)", x, y)
        self.client.send_goal(self._make_goal_msg(), done_cb=self.done_cb)

    def _mark_progress(self):
        try:
            x, y, _ = self.robot_xy_yaw()
        except TF_EXC:
            return
        self.progress_origin = (x, y)
        self.progress_time = rospy.Time.now()

    def _watchdog_cb(self, _event):
        if not self.active or self.recovering or self.goal_xy is None:
            return
        state = self.client.get_state()
        if state not in (GoalStatus.ACTIVE, GoalStatus.PENDING, GoalStatus.PREEMPTING):
            return
        if self.progress_origin is None or self.progress_time is None:
            self._mark_progress()
            return
        try:
            x, y, _ = self.robot_xy_yaw()
        except TF_EXC:
            return
        ox, oy = self.progress_origin
        moved = math.hypot(x - ox, y - oy)
        elapsed = (rospy.Time.now() - self.progress_time).to_sec()
        if moved >= self.stuck_distance:
            self._mark_progress()
            return
        if elapsed < self.stuck_time:
            return
        self._attempt_recovery("watchdog (no progress for %.0fs, moved %.2fm)" % (
            elapsed, moved))

    def _attempt_recovery(self, reason):
        if self.recoveries >= self.max_recoveries:
            rospy.logwarn(
                "%s — max recoveries (%d) reached, giving up.", reason, self.max_recoveries)
            self.client.cancel_all_goals()
            self.active = False
            return False
        self.recoveries += 1
        rospy.logwarn(
            "%s — recovery %d/%d.", reason, self.recoveries, self.max_recoveries)
        self.recovering = True
        self.client.cancel_all_goals()
        rospy.sleep(0.3)
        self._run_recovery()
        self.recovering = False
        self._mark_progress()
        self._dispatch_move_base()
        return True

    def _run_recovery(self):
        """Back out of corners, align with the doorway, creep if narrow."""
        stop_robot(self.cmd_pub)
        scan = self.latest_scan
        gx, gy = self.goal_xy

        if scan is not None and front_clearance(scan) < 0.35:
            rospy.loginfo("Recovery: front blocked — backing up.")
            self._safe_reverse(self.recovery_back_dist)

        try:
            rx, ry, yaw = self.robot_xy_yaw()
        except TF_EXC:
            return
        goal_bearing = normalize_angle(math.atan2(gy - ry, gx - rx) - yaw)
        scan = self.latest_scan
        if scan is not None:
            opening_bearing, width = doorway_opening(
                scan, preferred_bearing=goal_bearing)
            if 0.0 < width <= self.recovery_opening_max:
                if abs(opening_bearing) > math.radians(8.0):
                    drive_cmd(
                        self.cmd_pub,
                        angular_z=math.copysign(0.5, opening_bearing),
                        duration=min(abs(opening_bearing) / 0.5, 3.0))
                rospy.loginfo(
                    "Recovery: narrow opening ~%.2fm — creeping through.", width)
                creep_through_passage(
                    self.cmd_pub,
                    self._get_scan,
                    goal_bearing=0.0,
                    speed=self.recovery_creep_speed,
                    max_duration=self.recovery_creep_duration,
                    min_front=0.22,
                )
                stop_robot(self.cmd_pub)
                return
            if abs(goal_bearing) > math.radians(10.0):
                drive_cmd(
                    self.cmd_pub,
                    angular_z=math.copysign(0.6, goal_bearing),
                    duration=min(abs(goal_bearing) / 0.6, 3.5))
        elif abs(goal_bearing) > math.radians(10.0):
            drive_cmd(
                self.cmd_pub,
                angular_z=math.copysign(0.6, goal_bearing),
                duration=min(abs(goal_bearing) / 0.6, 3.5))

        scan = self.latest_scan
        if scan is not None and front_clearance(scan) > 0.40:
            left, right, front = side_clearances(scan)
            if front > 0.35:
                twist = Twist()
                twist.linear.x = self.recovery_creep_speed
                twist.angular.z = max(-0.3, min(0.3, 0.6 * (left - right)))
                end = rospy.Time.now() + rospy.Duration(2.0)
                rate = rospy.Rate(10)
                while not rospy.is_shutdown() and rospy.Time.now() < end:
                    scan = self.latest_scan
                    if scan is None or front_clearance(scan) < 0.28:
                        break
                    self.cmd_pub.publish(twist)
                    rate.sleep()
        stop_robot(self.cmd_pub)
        rospy.sleep(0.2)

    def _safe_reverse(self, distance):
        scan = self.latest_scan
        if scan is None or not rear_in_fov(scan):
            return False
        if rear_clearance(scan) < 0.30:
            open_b = best_open_bearing(scan)
            drive_cmd(
                self.cmd_pub,
                angular_z=math.copysign(0.5, open_b),
                duration=min(abs(open_b) / 0.5, 3.0))
            return False
        travelled = 0.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and travelled < distance:
            scan = self.latest_scan
            if scan is None or rear_clearance(scan) < 0.28:
                break
            twist = Twist()
            twist.linear.x = -abs(self.recovery_back_speed)
            self.cmd_pub.publish(twist)
            travelled += abs(self.recovery_back_speed) * 0.1
            rate.sleep()
        stop_robot(self.cmd_pub)
        return travelled > 0.02

    def done_cb(self, status, result):
        if self.recovering:
            return
        if status == GoalStatus.SUCCEEDED:
            self.active = False
            rospy.loginfo("Navigation succeeded — goal reached.")
            return
        if status == GoalStatus.PREEMPTED:
            self.active = False
            rospy.logwarn("Navigation preempted (new goal or cancel).")
            return
        gx, gy = self.goal_xy or (0.0, 0.0)
        if self._attempt_recovery(
                "move_base aborted (status %s) toward (%.2f, %.2f)" % (status, gx, gy)):
            return
        self.active = False
        rospy.logwarn("Navigation failed with status: %s", status)


def main():
    rospy.init_node("click_goal")
    ClickGoalNode()
    rospy.spin()


if __name__ == "__main__":
    main()
