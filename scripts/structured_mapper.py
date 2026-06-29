#!/usr/bin/env python3
"""
Center-based autonomous mapper for a Kobuki on The Construct (ROS Noetic).

The robot NEVER hugs walls during autonomous mapping. Instead it repeatedly:

    bootstrap -> seed 360 spin ->
    [ find center of an open unmapped expanse ->
      drive there (via move_base) -> 360 spin ] * until done or 200 s ->
    hand off to the operator.

Each scan point is the interior center of a known-free pocket that still
overlooks unknown space - never a perimeter waypoint and never a wall-follow
path. Narrow passages are crossed only when move_base stalls (laser-guarded
corridor creep), not as a mapping strategy.

Design guarantees:
  * Every point-to-point move goes through move_base (no driving through walls).
  * Goals sit at least goal_min_clearance from obstacles (never inch into walls).
  * Direct cmd_vel is used ONLY for in-place spins and emergency creep/escape.
"""

import math
import os
import subprocess
import sys

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String
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
    cell_value,
    cluster_frontier_cells,
    coverage_stats,
    creep_through_passage,
    doorway_opening,
    drive_cmd,
    find_expanse_scan_centers,
    find_frontier_cells,
    front_clearance,
    frontier_goal_from_cluster,
    get_robot_pose,
    map_indices,
    map_to_world,
    multi_angle_scan,
    nearest_free_cell,
    rear_clearance,
    rear_in_fov,
    resolve_cmd_vel_topic,
    stop_robot,
)

TF_EXC = (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class StructuredMapper(object):
    def __init__(self):
        # Frames / topics
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.scan_topic = rospy.get_param("~scan_topic", "/kobuki/laser/scan")
        self.cmd_topic = resolve_cmd_vel_topic(rospy.get_param("~cmd_vel_topic", ""))

        # Occupancy grid thresholds
        self.occupied_value = rospy.get_param("~occupied_value", 65)

        # Center-scan exploration (interior of open expanses, never wall-hugging)
        self.max_expanse_radius = int(rospy.get_param("~max_expanse_radius", 60))
        self.visited_center_radius = rospy.get_param("~visited_center_radius", 1.2)

        # Spins (capped <= 1.0 rad/s so gmapping can match scans while turning)
        self.spin_speed = min(1.0, rospy.get_param("~spin_speed", 0.9))
        self.spin_steps = int(rospy.get_param("~spin_steps", 6))
        self.spin_pause = rospy.get_param("~spin_pause", 0.4)

        # Goal placement / passage geometry (corridor creep recovery only)
        self.goal_min_clearance = rospy.get_param("~goal_min_clearance", 0.30)
        self.narrow_passage_width = rospy.get_param("~narrow_passage_width", 0.85)

        # Watchdogs / anti-stuck
        self.per_goal_timeout = rospy.get_param("~per_goal_timeout", 14.0)
        self.min_travel = rospy.get_param("~min_travel", 0.12)
        self.max_consecutive_stuck = int(rospy.get_param("~max_consecutive_stuck", 6))
        self.blacklist_radius = rospy.get_param("~blacklist_radius", 0.40)
        self.global_progress_timeout = rospy.get_param("~global_progress_timeout", 150.0)
        # Corridor creep: when a goal stalls but there is a narrow gap toward it,
        # creep through instead of giving up (keeps small passages from blocking
        # full coverage of the bounded area).
        self.corridor_creep_range = rospy.get_param("~corridor_creep_range", 3.0)
        self.corridor_front_min = rospy.get_param("~corridor_front_min", 0.30)

        # Escape / creep
        self.escape_clearance = rospy.get_param("~escape_clearance", 0.30)
        self.escape_back_speed = rospy.get_param("~escape_back_speed", 0.12)
        self.escape_back_time = rospy.get_param("~escape_back_time", 1.5)
        self.creep_speed = rospy.get_param("~creep_speed", 0.10)
        self.creep_duration = rospy.get_param("~creep_duration", 4.0)
        self.creep_min_front = rospy.get_param("~creep_min_front", 0.24)

        # Reverse safety (escape only - never used for wall-hugging mapping)
        self.rear_safe_clearance = rospy.get_param("~rear_safe_clearance", 0.35)

        # move_base server connection
        self.server_wait_timeout = rospy.get_param("~server_wait_timeout", 60.0)
        self.global_max_stuck = int(rospy.get_param("~global_max_stuck", 25))

        # Must-visit points (map frame). The robot actively drives to each at
        # least once, independent of frontiers, so a known area is never missed.
        # Seeded from the visit_waypoints param AND/OR clicked live in RViz
        # (Publish Point -> /clicked_point). Clicked points are always correct
        # because they refer to the CURRENT map; hard-coded visit_waypoints are
        # only valid if the map frame is consistent across runs (gmapping
        # anchors the map at the spawn pose, which varies with a random spawn).
        self.visit_timeout = rospy.get_param("~visit_timeout", 45.0)
        self.pending_visits = []
        for wp in rospy.get_param("~visit_waypoints", []):
            if isinstance(wp, (list, tuple)) and len(wp) >= 2:
                self.pending_visits.append((float(wp[0]), float(wp[1])))

        # Bootstrap
        self.bootstrap_clearance = rospy.get_param("~bootstrap_clearance", 0.35)
        self.bootstrap_forward_clearance = rospy.get_param("~bootstrap_forward_clearance", 0.85)
        self.bootstrap_forward_speed = rospy.get_param("~bootstrap_forward_speed", 0.25)

        # Completion / coverage
        self.max_unknown_ratio = rospy.get_param("~max_unknown_ratio", 0.10)
        # Hard cap on the autonomous scan. After this (or when exploration ends
        # earlier) control is handed back to the operator.
        self.max_runtime = rospy.get_param("~max_runtime", 200.0)
        # Save the map at hand-off so a map always exists for the next phase.
        self.save_map_at_handoff = rospy.get_param("~save_map_at_handoff", True)
        self.map_save_path = rospy.get_param("~map_save_path", "")
        self.min_frontier_cells = int(rospy.get_param("~min_frontier_cells", 6))
        self.distance_penalty = rospy.get_param("~distance_penalty", 5.0)
        # Corridor creep is ONLY allowed inside these map-frame boxes (e.g.
        # kitchen entrance). Everywhere else the robot skips stuck goals and
        # picks the next expanse center — never hugs walls.
        self.corridor_zones = []
        for zone in rospy.get_param("~corridor_zones", []):
            if isinstance(zone, dict) and "min_x" in zone:
                self.corridor_zones.append(zone)

        # Optional virtual boundary. OFF by default so the robot maps the WHOLE
        # reachable space - through doorways, other rooms, and OUTSIDE the
        # building. WARNING: a planar LiDAR cannot see drop-offs/ledges; only
        # enable bounds if the world has unguarded edges the robot could fall off.
        self.bounds_enabled = rospy.get_param("~explore_bounds_enabled", False)
        self.bounds_min_x = rospy.get_param("~explore_min_x", -1e9)
        self.bounds_max_x = rospy.get_param("~explore_max_x", 1e9)
        self.bounds_min_y = rospy.get_param("~explore_min_y", -1e9)
        self.bounds_max_y = rospy.get_param("~explore_max_y", 1e9)

        self.map_msg = None
        self.latest_scan = None
        self.blacklist = []
        self.visited_centers = []
        self.consecutive_stuck = 0
        self.total_stuck = 0
        self.start_time = None
        self.last_progress = None
        self.last_ratio_at_progress = 1.0

        self.tf_listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.state_pub = rospy.Publisher(
            "/structured_mapper/state", String, queue_size=1, latch=True)
        self.unknown_pub = rospy.Publisher(
            "/structured_mapper/unknown_ratio", Float32, queue_size=1)
        self.frontier_pub = rospy.Publisher(
            "/structured_mapper/frontier_count", Int32, queue_size=1)
        self.complete_pub = rospy.Publisher(
            "/structured_mapper/complete", Bool, queue_size=1, latch=True)
        # Kept so the existing exploration_monitor's start gate still fires.
        self.started_pub = rospy.Publisher(
            "/exploration_bootstrap/complete", Bool, queue_size=1, latch=True)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=1)
        # Live "go map there" points clicked in RViz (Publish Point tool).
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb, queue_size=8)

        rospy.loginfo("Structured mapper: waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(self.server_wait_timeout)):
            raise rospy.ROSException(
                "move_base action server not available after %.0fs. Is move_base "
                "running? (check `rosnode list | grep move_base`)"
                % self.server_wait_timeout)
        rospy.loginfo(
            "Structured mapper ready (scan=%s, cmd_vel=%s).",
            self.scan_topic, self.cmd_topic)

    # ------------------------------------------------------------------ I/O
    def map_cb(self, msg):
        self.map_msg = msg

    def scan_cb(self, msg):
        self.latest_scan = msg

    def clicked_point_cb(self, msg):
        self.pending_visits.append((msg.point.x, msg.point.y))
        rospy.loginfo(
            "Queued must-visit point (%.2f, %.2f) from RViz Publish Point.",
            msg.point.x, msg.point.y)

    def _get_scan(self):
        return self.latest_scan

    def set_state(self, name):
        self.state_pub.publish(String(data=name))
        rospy.loginfo("[structured_mapper] state -> %s", name)

    def wait_for_map(self, timeout=60.0):
        start = rospy.Time.now()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and self.map_msg is None:
            if (rospy.Time.now() - start).to_sec() > timeout:
                raise rospy.ROSException("Timed out waiting for /map")
            rate.sleep()

    def wait_for_scan(self, timeout=30.0):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.latest_scan is None:
            if (rospy.Time.now() - start).to_sec() > timeout:
                raise rospy.ROSException(
                    "No laser scan on '%s' after %.0fs. Check the topic name "
                    "(`rostopic list | grep scan`) and pass scan_topic:=... ."
                    % (self.scan_topic, timeout))
            rate.sleep()

    def robot_pose(self):
        return get_robot_pose(self.tf_listener, self.map_frame, self.base_frame)

    def robot_xy(self):
        try:
            x, y, _ = self.robot_pose()
            return x, y
        except TF_EXC:
            return None

    # -------------------------------------------------------------- geometry
    def within_bounds(self, x, y):
        if not self.bounds_enabled:
            return True
        return (
            self.bounds_min_x <= x <= self.bounds_max_x
            and self.bounds_min_y <= y <= self.bounds_max_y
        )

    def is_blacklisted(self, x, y):
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def is_center_visited(self, x, y):
        for vx, vy in self.visited_centers:
            if math.hypot(x - vx, y - vy) < self.visited_center_radius:
                return True
        return False

    def _corridor_zone_at(self, x, y):
        """Return the corridor zone dict containing (x,y), or None."""
        for zone in self.corridor_zones:
            if (
                zone["min_x"] <= x <= zone["max_x"]
                and zone["min_y"] <= y <= zone["max_y"]
            ):
                return zone
        return None

    def _corridor_recovery_allowed(self, gx, gy):
        """Corridor creep is permitted only inside configured zones."""
        if not self.corridor_zones:
            return False
        robot = self.robot_xy()
        if robot is None:
            return False
        rx, ry = robot
        return self._corridor_zone_at(gx, gy) is not None or self._corridor_zone_at(rx, ry) is not None

    def snap_to_clear(self, x, y):
        """Return a footprint-clear point near (x,y) with goal_min_clearance, or (None,None)."""
        map_msg = self.map_msg
        if map_msg is None:
            return x, y
        mx, my = map_indices(map_msg, x, y)
        clearance = self._clearance_cells()
        if cell_region_free(map_msg, mx, my, clearance):
            return x, y
        free = nearest_free_cell(map_msg, mx, my, max_radius=12, clearance_cells=clearance)
        if free is None:
            return None, None
        return map_to_world(map_msg, free[0], free[1])

    def _clearance_cells(self):
        return max(2, int(round(self.goal_min_clearance / self.map_msg.info.resolution)))

    # ------------------------------------------------------------- move_base
    def make_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def drive_to(self, x, y, facing=None, timeout=None):
        """Send a move_base goal at a safe standoff and watch for no-progress.

        Returns True only if the robot actually moved AND move_base reported
        success. On no motion it triggers escape/unstick + blacklist so the
        tour never stalls on an unreachable point.
        """
        if timeout is None:
            timeout = self.per_goal_timeout
        if self.map_msg is None:
            return False
        if self.is_blacklisted(x, y):
            return False
        sx, sy = self.snap_to_clear(x, y)
        if sx is None:
            rospy.logwarn("No clear cell near goal (%.2f, %.2f) - skipping.", x, y)
            self.blacklist.append((x, y))
            return False
        if not self.within_bounds(sx, sy):
            return False

        try:
            rx, ry, _ = self.robot_pose()
        except TF_EXC:
            rospy.logwarn("TF unavailable - skipping goal.")
            return False
        yaw = facing if facing is not None else math.atan2(sy - ry, sx - rx)

        # In a narrow-corridor zone only: try laser-centered creep BEFORE
        # move_base (which often refuses tight doorways). Nowhere else.
        if self._corridor_recovery_allowed(sx, sy):
            self._attempt_corridor_creep(sx, sy)

        self.client.send_goal(self.make_goal(sx, sy, yaw))
        finished = self.client.wait_for_result(rospy.Duration(timeout))
        state = self.client.get_state()
        if not finished:
            self.client.cancel_all_goals()

        try:
            ex, ey, _ = self.robot_pose()
        except TF_EXC:
            ex, ey = rx, ry
        travel = math.hypot(ex - rx, ey - ry)

        if travel < self.min_travel:
            self.handle_stuck(sx, sy)
            return False

        self.consecutive_stuck = 0
        self.mark_progress()
        return state == GoalStatus.SUCCEEDED

    def handle_stuck(self, x, y):
        rospy.logwarn("No progress toward (%.2f, %.2f) - recovering.", x, y)
        self.client.cancel_all_goals()
        rospy.sleep(0.2)
        # Corridor creep ONLY inside configured zones (e.g. kitchen passage).
        if self._corridor_recovery_allowed(x, y):
            if self._attempt_corridor_creep(x, y):
                self.consecutive_stuck = 0
                self.mark_progress()
                return
        if not self.escape_if_boxed_in():
            self.unstick_rotate(x, y)
        self.blacklist.append((x, y))
        self.consecutive_stuck += 1
        self.total_stuck += 1

    def _attempt_corridor_creep(self, x, y):
        """Laser-centered creep through a narrow gap toward (x,y).

        Only runs inside corridor_zones — never wall-hugs in open areas.
        """
        if not self._corridor_recovery_allowed(x, y):
            return False
        zone = self._corridor_zone_at(x, y)
        if zone is None:
            robot = self.robot_xy()
            if robot is not None:
                zone = self._corridor_zone_at(robot[0], robot[1])
        max_width = (
            zone.get("max_passage_width", self.narrow_passage_width)
            if zone else self.narrow_passage_width
        )
        scan = self.latest_scan
        if scan is None:
            return False
        try:
            rx, ry, yaw = self.robot_pose()
        except TF_EXC:
            return False
        if math.hypot(x - rx, y - ry) > self.corridor_creep_range:
            return False
        goal_bearing = normalize_angle(math.atan2(y - ry, x - rx) - yaw)
        opening_bearing, width = doorway_opening(scan, preferred_bearing=goal_bearing)
        if width <= 0.0 or width > max_width:
            return False
        if abs(opening_bearing) > math.radians(10.0):
            self.face_bearing_relative(opening_bearing)
        rospy.loginfo(
            "Corridor zone: narrow passage ~%.2fm — creeping through.", width)
        self.set_state("corridor-creep")
        return creep_through_passage(
            self.cmd_pub,
            self._get_scan,
            goal_bearing=0.0,
            speed=self.creep_speed,
            max_duration=self.creep_duration,
            min_front=self.corridor_front_min,
        )

    def safe_reverse(self, distance):
        """Reverse up to `distance` m, but ONLY while the rear is provably clear.

        The 2D LiDAR usually cannot see directly behind, so if there are no rear
        beams we refuse to reverse blindly (turn-in-place is used instead). This
        is what keeps the robot from backing into a wall/obstacle.
        """
        scan = self.latest_scan
        if scan is None:
            return False
        if not rear_in_fov(scan):
            rospy.logwarn("Rear not in laser FOV - refusing to reverse blindly.")
            return False
        if rear_clearance(scan) < self.rear_safe_clearance:
            rospy.logwarn(
                "Rear blocked (%.2fm < %.2fm) - not reversing.",
                rear_clearance(scan), self.rear_safe_clearance)
            return False

        travelled = 0.0
        rate = rospy.Rate(10)
        dt = 0.1
        while not rospy.is_shutdown() and travelled < distance:
            scan = self.latest_scan
            if scan is None or rear_clearance(scan) < self.rear_safe_clearance:
                break
            twist = Twist()
            twist.linear.x = -abs(self.escape_back_speed)
            self.cmd_pub.publish(twist)
            travelled += abs(self.escape_back_speed) * dt
            rate.sleep()
        stop_robot(self.cmd_pub)
        rospy.sleep(0.2)
        return travelled > 0.02

    def escape_if_boxed_in(self):
        """Reverse out when wedged against a wall/corner so a plan can start again."""
        scan = self.latest_scan
        if scan is None or front_clearance(scan) >= self.escape_clearance:
            return False
        self.client.cancel_all_goals()
        rospy.sleep(0.2)
        rospy.logwarn("Boxed in (front %.2fm) - reversing out.", front_clearance(scan))
        if not self.safe_reverse(self.escape_back_speed * self.escape_back_time):
            # Cannot back up safely (rear blocked/unseen): rotate toward the
            # most open bearing in place instead of forcing a blind reverse.
            self.face_bearing_relative(best_open_bearing(self.latest_scan))
            return True
        open_bearing = best_open_bearing(self.latest_scan)
        if abs(open_bearing) > math.radians(12.0):
            self.face_bearing_relative(open_bearing)
        rospy.sleep(0.2)
        return True

    def unstick_rotate(self, x, y):
        try:
            rx, ry, yaw = self.robot_pose()
        except TF_EXC:
            return
        delta = normalize_angle(math.atan2(y - ry, x - rx) - yaw)
        if abs(delta) < math.radians(12.0):
            return
        direction = 1.0 if delta > 0.0 else -1.0
        drive_cmd(self.cmd_pub, angular_z=direction * 0.45, duration=min(abs(delta) / 0.45, 4.0))
        rospy.sleep(0.3)

    def face_bearing(self, target_map_bearing):
        """Rotate in place until the robot points at a map-frame bearing."""
        self.client.cancel_all_goals()
        try:
            _, _, yaw = self.robot_pose()
        except TF_EXC:
            return
        delta = normalize_angle(target_map_bearing - yaw)
        if abs(delta) < math.radians(8.0):
            return
        direction = 1.0 if delta > 0.0 else -1.0
        drive_cmd(self.cmd_pub, angular_z=direction * self.spin_speed,
                  duration=min(abs(delta) / self.spin_speed, 4.0))

    def spin360(self, reason):
        self.set_state(reason)
        self.client.cancel_all_goals()
        rospy.sleep(0.2)
        multi_angle_scan(
            self.cmd_pub,
            rotate_speed=self.spin_speed,
            pause_sec=self.spin_pause,
            steps=self.spin_steps,
        )

    # -------------------------------------------------------------- progress
    def mark_progress(self):
        self.last_progress = rospy.Time.now()
        self.last_ratio_at_progress = self._current_unknown_ratio()

    def timed_out(self):
        if self.start_time is None:
            return False
        return (rospy.Time.now() - self.start_time).to_sec() > self.max_runtime

    def _current_unknown_ratio(self):
        ratio, _, _, _ = coverage_stats(self.map_msg)
        return ratio

    # --------------------------------------------------------------- phases
    def bootstrap(self):
        """Escape a wall spawn, face open space, and nudge into the room."""
        self.set_state("bootstrap")
        scan = self.latest_scan
        if scan is None:
            return
        if front_clearance(scan) < self.bootstrap_clearance:
            rospy.logwarn("Spawned against a wall (%.2fm) - backing up.", front_clearance(scan))
            if not self.safe_reverse(self.escape_back_speed * self.escape_back_time):
                # Rear blocked/unseen: just turn toward open space in place.
                rospy.logwarn("Cannot reverse safely at spawn - turning in place.")
            self.face_bearing_relative(best_open_bearing(self.latest_scan))
        scan = self.latest_scan
        if scan is not None and (
            front_clearance(scan) < self.bootstrap_forward_clearance
            or abs(best_open_bearing(scan)) > math.radians(15.0)
        ):
            self.face_bearing_relative(best_open_bearing(scan))
        # Nudge forward into open space (laser-guarded).
        end = rospy.Time.now() + rospy.Duration(6.0)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            scan = self.latest_scan
            if scan is None or front_clearance(scan) < self.bootstrap_forward_clearance:
                break
            drive_cmd(self.cmd_pub, linear_x=self.bootstrap_forward_speed, duration=0.25)
            rate.sleep()

    def face_bearing_relative(self, robot_frame_bearing):
        """Rotate by a bearing expressed in the robot frame (laser bearings)."""
        if abs(robot_frame_bearing) < math.radians(8.0):
            return
        direction = 1.0 if robot_frame_bearing > 0.0 else -1.0
        drive_cmd(
            self.cmd_pub,
            angular_z=direction * self.spin_speed,
            duration=min(abs(robot_frame_bearing) / self.spin_speed, 4.0),
        )

    def signal_started(self):
        """Tell the auto-save monitor that mapping has begun."""
        self.started_pub.publish(Bool(data=True))
        rospy.set_param("/exploration_bootstrap/complete", True)
        self.mark_progress()

    def service_visits(self):
        """Actively drive to every queued must-visit point (>= once each).

        Independent of center-scan: guarantees the robot physically reaches each
        requested area and scans it. Uses the same standoff + recovery as any goal.
        """
        while self.pending_visits and not rospy.is_shutdown():
            if self.timed_out():
                return
            vx, vy = self.pending_visits.pop(0)
            rospy.loginfo("Actively visiting requested point (%.2f, %.2f).", vx, vy)
            self.set_state("visit")
            self.blacklist = [
                b for b in self.blacklist
                if math.hypot(b[0] - vx, b[1] - vy) >= self.blacklist_radius
            ]
            self.drive_to(vx, vy, timeout=self.visit_timeout)
            self.spin360("visit-scan")

    def center_scan_loop(self):
        """Drive to interior centers of open expanses and 360-scan each one.

        Never places goals on walls or perimeter rings. Each iteration picks the
        best unvisited center of a free pocket that still overlooks unknown space,
        navigates there via move_base, spins 360, then repeats until the map is
        covered or max_runtime (200 s) expires.
        """
        self.set_state("center-scan")
        self.consecutive_stuck = 0
        clearance = self._clearance_cells()
        last_ratio = 1.0
        last_improve = rospy.Time.now()
        scans_done = 0
        blacklist_resets = 0
        max_blacklist_resets = int(rospy.get_param("~max_blacklist_resets", 2))

        while not rospy.is_shutdown():
            if self.timed_out():
                rospy.logwarn("Center-scan hit max runtime (%.0fs) - stopping.", self.max_runtime)
                break

            if self.pending_visits:
                self.service_visits()
                last_improve = rospy.Time.now()

            centers = find_expanse_scan_centers(
                self.map_msg,
                min_cluster_size=self.min_frontier_cells,
                clearance_cells=clearance,
                occupied_thresh=self.occupied_value,
                max_expanse_radius=self.max_expanse_radius,
            )

            robot = self.robot_xy() or (0.0, 0.0)
            rx, ry = robot
            candidates = []
            for cx, cy, size in centers:
                if self.is_center_visited(cx, cy):
                    continue
                if self.is_blacklisted(cx, cy):
                    continue
                if not self.within_bounds(cx, cy):
                    continue
                candidates.append((cx, cy, size))

            self.frontier_pub.publish(Int32(data=len(candidates)))

            if not candidates:
                ratio = self._current_unknown_ratio()
                if ratio <= self.max_unknown_ratio:
                    rospy.loginfo(
                        "No unvisited expanse centers and unknown ~%.1f%% - done.",
                        ratio * 100.0)
                    break
                # Fallback: try a frontier-edge goal (still via move_base, not
                # wall-hug) when interior centers are exhausted but unknown remains.
                if not self._attempt_frontier_fallback(clearance, rx, ry):
                    if blacklist_resets < max_blacklist_resets and self.blacklist:
                        blacklist_resets += 1
                        rospy.logwarn(
                            "No centers reachable - clearing blacklist (reset %d/%d).",
                            blacklist_resets, max_blacklist_resets)
                        self.blacklist = []
                        self.consecutive_stuck = 0
                        continue
                    rospy.logwarn("No reachable expanse centers left - stopping.")
                    break
                last_improve = rospy.Time.now()
                continue

            candidates.sort(
                key=lambda c: c[2] - self.distance_penalty * math.hypot(c[0] - rx, c[1] - ry),
                reverse=True)
            cx, cy, size = candidates[0]
            rospy.loginfo(
                "Expanse center (%.2f, %.2f) size=%d - driving to scan.",
                cx, cy, size)
            self.set_state("go-to-center")
            # Small frontiers get a short timeout — skip fast, try the next center.
            timeout = self.per_goal_timeout
            if size < self.min_frontier_cells * 3:
                timeout = max(6.0, self.per_goal_timeout * 0.6)
            if self.drive_to(cx, cy, timeout=timeout):
                self.visited_centers.append((cx, cy))
                scans_done += 1
                self.spin360("center-scan")
                last_improve = rospy.Time.now()
                rospy.loginfo("Completed scan #%d at expanse center.", scans_done)

            ratio = self._current_unknown_ratio()
            self.unknown_pub.publish(Float32(data=ratio))
            if ratio < last_ratio - 0.005:
                last_ratio = ratio
                last_improve = rospy.Time.now()
            elif (rospy.Time.now() - last_improve).to_sec() > self.global_progress_timeout:
                if blacklist_resets < max_blacklist_resets and self.blacklist:
                    blacklist_resets += 1
                    rospy.logwarn(
                        "Stalled for %.0fs - clearing blacklist (reset %d/%d).",
                        self.global_progress_timeout, blacklist_resets, max_blacklist_resets)
                    self.blacklist = []
                    self.consecutive_stuck = 0
                    last_improve = rospy.Time.now()
                    continue
                rospy.logwarn(
                    "No coverage gain for %.0fs - ending center-scan.",
                    self.global_progress_timeout)
                break

            if self.consecutive_stuck >= self.max_consecutive_stuck:
                if blacklist_resets < max_blacklist_resets and self.blacklist:
                    blacklist_resets += 1
                    rospy.logwarn(
                        "Repeatedly stuck - clearing blacklist (reset %d/%d).",
                        blacklist_resets, max_blacklist_resets)
                    self.blacklist = []
                    self.consecutive_stuck = 0
                    continue
                rospy.logwarn("Repeatedly stuck - ending center-scan.")
                break
            if self.total_stuck >= self.global_max_stuck:
                rospy.logwarn(
                    "Hit global stuck limit (%d) - ending center-scan.",
                    self.global_max_stuck)
                break

    def _attempt_frontier_fallback(self, clearance, rx, ry):
        """Last resort: one frontier goal when no interior center is reachable."""
        cells = find_frontier_cells(self.map_msg)
        clusters = cluster_frontier_cells(cells)
        goals = []
        for cluster in clusters:
            if len(cluster) < self.min_frontier_cells:
                continue
            goal = frontier_goal_from_cluster(
                self.map_msg, cluster, clearance_cells=clearance)
            if goal is None:
                continue
            gx, gy, gyaw = goal
            if self.is_blacklisted(gx, gy) or not self.within_bounds(gx, gy):
                continue
            goals.append((gx, gy, gyaw, len(cluster)))
        if not goals:
            return False
        goals.sort(
            key=lambda g: g[3] - self.distance_penalty * math.hypot(g[0] - rx, g[1] - ry),
            reverse=True)
        gx, gy, gyaw, _ = goals[0]
        rospy.loginfo("Fallback frontier goal (%.2f, %.2f).", gx, gy)
        if self.drive_to(gx, gy, facing=gyaw):
            self.spin360("frontier-fallback-scan")
            return True
        return False

    def _map_base_path(self):
        if self.map_save_path:
            return self.map_save_path
        try:
            import rospkg
            pkg = rospkg.RosPack().get_path("summit_xl_autonomous_nav")
            return os.path.join(pkg, "maps", "summit_world")
        except Exception:
            return "summit_world"

    def _save_map(self):
        base = self._map_base_path()
        rospy.loginfo("Saving map to %s.{pgm,yaml} ...", base)
        try:
            subprocess.check_call(["rosrun", "map_server", "map_saver", "-f", base])
            rospy.loginfo("Map saved.")
            return True
        except (subprocess.CalledProcessError, OSError) as exc:
            rospy.logerr("map_saver failed: %s", exc)
            return False

    def _print_handoff_banner(self, ratio):
        base = self._map_base_path()
        rospy.logwarn(
            "\n%s\n AUTONOMOUS SCAN DONE - control is now YOURS (unknown ~%.1f%%).\n"
            " The robot is stopped; gmapping is still running so you can keep mapping.\n"
            " Map saved to: %s.{pgm,yaml}\n%s\n"
            " Choose the next step (run in a NEW terminal):\n"
            "   rosrun summit_xl_autonomous_nav mapping_control.py\n"
            " It will ask: keep mapping MANUALLY (drive it yourself) or finish.\n"
            "   - YES -> teleop drive to any spots; gmapping maps them; then it saves.\n"
            "   - NO  -> it saves the map and tells you how to start navigation.\n"
            " Then, for autonomous go-to-point navigation:\n"
            "   1) Ctrl+C this mapping launch.\n"
            "   2) roslaunch summit_xl_autonomous_nav kobuki_go_to_goal.launch\n"
            "   3) In RViz: 2D Pose Estimate, then 2D Nav Goal (or Publish Point).\n%s",
            "=" * 70, ratio * 100.0, base, "-" * 70, "=" * 70)

    def finish(self):
        """Hand control back to the operator (do NOT keep commanding the robot).

        Stops the robot, saves the map so the next phase always has one, marks
        complete, then stays alive (gmapping keeps running) so the operator can
        either keep mapping manually via teleop or move on to navigation.
        """
        self.set_state("done")
        self.client.cancel_all_goals()
        stop_robot(self.cmd_pub)
        # Release cmd_vel so manual teleop has exclusive control (move_base
        # would otherwise fight keyboard commands during manual mapping).
        try:
            subprocess.call(["rosnode", "kill", "/move_base"])
            rospy.loginfo("move_base stopped for manual-mapping handoff.")
        except (OSError, subprocess.SubprocessError):
            pass
        ratio = self._current_unknown_ratio()
        self.unknown_pub.publish(Float32(data=ratio))
        self.frontier_pub.publish(Int32(data=0))
        if self.save_map_at_handoff:
            self._save_map()
        self.complete_pub.publish(Bool(data=True))
        self.set_state("handoff")
        self._print_handoff_banner(ratio)
        # Stay alive but idle: the robot is stopped and we no longer publish
        # cmd_vel, so manual teleop has full control while gmapping keeps mapping.
        rospy.spin()

    def run(self):
        self.wait_for_map()
        self.wait_for_scan()
        self.start_time = rospy.Time.now()
        self.mark_progress()

        self.bootstrap()
        self.spin360("seed-spin")
        self.signal_started()
        self.service_visits()
        self.center_scan_loop()
        self.finish()


def main():
    rospy.init_node("structured_mapper")
    try:
        StructuredMapper().run()
    except rospy.ROSException as exc:
        rospy.logerr("Structured mapper failed: %s", exc)


if __name__ == "__main__":
    main()
