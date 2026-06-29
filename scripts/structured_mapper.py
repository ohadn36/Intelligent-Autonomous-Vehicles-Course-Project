#!/usr/bin/env python3
"""
Structured autonomous mapper for a Kobuki on The Construct (ROS Noetic).

Replaces greedy frontier exploration with a deliberate "guided tour" so the
robot maps the whole area smoothly instead of hugging walls, crawling, or
getting stuck:

    bootstrap -> seed 360 spin -> drive to room center -> 360 spin ->
    perimeter sweep at a fixed standoff from the walls -> at each opening:
    stop, 360 spin, enter (creep if narrow), drive to side-room center,
    360 spin, exit -> full-coverage frontier exploration of the WHOLE
    reachable space (other rooms AND outside the building) -> done.

    The robot may spawn anywhere (the start pose is read from the laser/scan,
    nothing is hard-coded), and there is no virtual boundary, so it maps the
    entire reachable area inside and outside.

Design guarantees:
  * Every point-to-point move goes through move_base, so the costmap +
    planner never let the robot drive through walls or obstacles.
  * Goals are always placed at a standoff from obstacles (>= goal_min_clearance),
    so the robot maps a wall from a comfortable distance and never inches into it.
  * Direct cmd_vel is used ONLY for in-place spins and laser-guarded creep/escape,
    each gated by front/side clearance so it cannot drive into a wall.
  * Two-wall pockets / dead-ends are detected and skipped (or reversed out of)
    instead of stalling.
"""

import math
import os
import sys

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
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
    find_frontier_cells,
    front_clearance,
    frontier_goal_from_cluster,
    get_robot_pose,
    is_occupied,
    is_unknown,
    map_indices,
    map_to_world,
    multi_angle_scan,
    nearest_free_cell,
    occupied_borders_unknown,
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


def circular_mean(angles):
    sin_sum = sum(math.sin(a) for a in angles)
    cos_sum = sum(math.cos(a) for a in angles)
    return math.atan2(sin_sum, cos_sum)


class StructuredMapper(object):
    def __init__(self):
        # Frames / topics
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.scan_topic = rospy.get_param("~scan_topic", "/kobuki/laser/scan")
        self.cmd_topic = resolve_cmd_vel_topic(rospy.get_param("~cmd_vel_topic", ""))

        # Perimeter sweep / ray-cast analysis
        self.wall_standoff = rospy.get_param("~wall_standoff", 0.80)
        self.waypoint_spacing = rospy.get_param("~waypoint_spacing", 0.6)
        self.ray_count = int(rospy.get_param("~ray_count", 72))
        self.max_ray = rospy.get_param("~max_ray", 12.0)
        self.occupied_value = rospy.get_param("~occupied_value", 65)

        # Spins (capped <= 1.0 rad/s so gmapping can match scans while turning)
        self.spin_speed = min(1.0, rospy.get_param("~spin_speed", 0.9))
        self.spin_steps = int(rospy.get_param("~spin_steps", 6))
        self.spin_pause = rospy.get_param("~spin_pause", 0.4)

        # Goal placement / opening + pocket geometry
        self.goal_min_clearance = rospy.get_param("~goal_min_clearance", 0.30)
        self.opening_min_width = rospy.get_param("~opening_min_width", 0.50)
        self.narrow_passage_width = rospy.get_param("~narrow_passage_width", 0.85)
        self.pocket_clearance = rospy.get_param("~pocket_clearance", 0.45)
        self.deadend_front = rospy.get_param("~deadend_front", 0.35)
        self.approach_standoff = rospy.get_param("~approach_standoff", 0.8)
        self.enter_depth = rospy.get_param("~enter_depth", 0.9)

        # Watchdogs / anti-stuck
        self.per_goal_timeout = rospy.get_param("~per_goal_timeout", 14.0)
        self.min_travel = rospy.get_param("~min_travel", 0.12)
        self.max_consecutive_stuck = int(rospy.get_param("~max_consecutive_stuck", 6))
        self.blacklist_radius = rospy.get_param("~blacklist_radius", 0.40)
        self.global_progress_timeout = rospy.get_param("~global_progress_timeout", 150.0)
        self.opening_visited_radius = rospy.get_param("~opening_visited_radius", 1.0)
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

        # Reverse safety + "understand the wall" behavior
        self.rear_safe_clearance = rospy.get_param("~rear_safe_clearance", 0.35)
        self.back_step_dist = rospy.get_param("~back_step_dist", 0.25)
        self.wall_near_clearance = rospy.get_param("~wall_near_clearance", 0.45)
        self.understand_wall_on_stuck = rospy.get_param("~understand_wall_on_stuck", True)

        # Wall-follow recovery: when pinned at a wall with the goal far on the
        # OTHER side, slide along the wall toward open space to find the real
        # opening, instead of grinding straight into the wall.
        self.wall_follow_on_stuck = rospy.get_param("~wall_follow_on_stuck", True)
        self.wall_follow_speed = rospy.get_param("~wall_follow_speed", 0.14)
        self.wall_follow_time = rospy.get_param("~wall_follow_time", 4.0)
        self.wall_follow_goal_dist = rospy.get_param("~wall_follow_goal_dist", 1.5)

        # move_base server connection
        self.server_wait_timeout = rospy.get_param("~server_wait_timeout", 60.0)
        self.global_max_stuck = int(rospy.get_param("~global_max_stuck", 25))

        # Bootstrap
        self.bootstrap_clearance = rospy.get_param("~bootstrap_clearance", 0.35)
        self.bootstrap_forward_clearance = rospy.get_param("~bootstrap_forward_clearance", 0.85)
        self.bootstrap_forward_speed = rospy.get_param("~bootstrap_forward_speed", 0.25)

        # Completion / coverage
        self.max_unknown_ratio = rospy.get_param("~max_unknown_ratio", 0.10)
        self.max_runtime = rospy.get_param("~max_runtime", 1800.0)
        self.min_frontier_cells = int(rospy.get_param("~min_frontier_cells", 6))
        self.distance_penalty = rospy.get_param("~distance_penalty", 5.0)
        # 360 scan every N reached frontiers so each new room / outdoor area is
        # mapped from a good vantage, not just glanced at while passing.
        self.frontier_scan_every = int(rospy.get_param("~frontier_scan_every", 4))

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
        self.visited_openings = []
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

    def is_opening_visited(self, ax, ay):
        for vx, vy in self.visited_openings:
            if math.hypot(ax - vx, ay - vy) < self.opening_visited_radius:
                return True
        return False

    def cast_ray(self, ox, oy, angle):
        """March a ray on the occupancy grid; classify what it hits.

        Returns (kind, distance, x, y, mx, my) where kind is "hit" (wall),
        "unknown" (reached unmapped space -> opening direction) or "max"
        (free all the way to range). mx,my are the grid indices of the cell
        that ended the ray.
        """
        map_msg = self.map_msg
        step = max(0.02, map_msg.info.resolution * 0.5)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        r = map_msg.info.resolution
        while r <= self.max_ray:
            wx = ox + r * cos_a
            wy = oy + r * sin_a
            mx, my = map_indices(map_msg, wx, wy)
            value = cell_value(map_msg, mx, my)
            if is_occupied(value, self.occupied_value):
                return "hit", r, wx, wy, mx, my
            if is_unknown(value):
                return "unknown", r, wx, wy, mx, my
            r += step
        ex = ox + self.max_ray * cos_a
        ey = oy + self.max_ray * sin_a
        mx, my = map_indices(map_msg, ex, ey)
        return "max", self.max_ray, ex, ey, mx, my

    def _clearance_cells(self):
        return max(2, int(round(self.goal_min_clearance / self.map_msg.info.resolution)))

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

    def compute_room_center(self):
        """Estimate the centroid of the room around the robot from wall hits."""
        pose = self.robot_xy()
        if pose is None:
            return None
        ox, oy = pose
        xs, ys = [], []
        for i in range(self.ray_count):
            angle = -math.pi + (2.0 * math.pi * i) / self.ray_count
            kind, _, hx, hy, _, _ = self.cast_ray(ox, oy, angle)
            if kind == "hit":
                xs.append(hx)
                ys.append(hy)
        if len(xs) < max(4, self.ray_count // 6):
            return ox, oy
        cx = sum(xs) / float(len(xs))
        cy = sum(ys) / float(len(ys))
        sx, sy = self.snap_to_clear(cx, cy)
        if sx is None or not self.within_bounds(sx, sy):
            return ox, oy
        return sx, sy

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
        # First: if a narrow passage leads toward the goal, creep through it
        # instead of giving up. This is how tight corridors/doorways get mapped
        # (move_base/DWA refuses them, but a slow centered creep fits). If it
        # works we make progress and do NOT blacklist the goal.
        if self._attempt_corridor_creep(x, y):
            self.consecutive_stuck = 0
            self.mark_progress()
            return
        # Pinned at a wall with the goal far on the other side? Slide along the
        # wall toward open space to discover the opening, then drop THIS goal so
        # a reachable frontier is chosen next (avoids grinding into the wall).
        followed = self._wall_follow_to_find_opening(x, y)
        if not followed:
            scan = self.latest_scan
            if (
                self.understand_wall_on_stuck
                and scan is not None
                and front_clearance(scan) < self.wall_near_clearance
            ):
                self.understand_wall()
            elif not self.escape_if_boxed_in():
                self.unstick_rotate(x, y)
        self.blacklist.append((x, y))
        self.consecutive_stuck += 1
        self.total_stuck += 1
        if followed:
            self.mark_progress()

    def _wall_follow_to_find_opening(self, x, y):
        """Slide toward the most open direction to find an opening in a wall.

        Triggered when the robot is nose-to-a-wall but the goal is far beyond
        it: the connecting doorway is somewhere ALONG the wall, so driving into
        the wall never works. Turn to the most open bearing (tangential to the
        wall / toward the opening) and creep there, laser-guarded and bounded.
        Returns True if the robot moved.
        """
        if not self.wall_follow_on_stuck:
            return False
        scan = self.latest_scan
        if scan is None:
            return False
        rxy = self.robot_xy()
        if rxy is None:
            return False
        if math.hypot(x - rxy[0], y - rxy[1]) < self.wall_follow_goal_dist:
            return False  # goal is near; this is not a cross-the-wall problem
        if front_clearance(scan) >= self.wall_near_clearance:
            return False  # not actually pinned against a wall
        self.set_state("wall-follow")
        rospy.loginfo("Pinned at a wall, goal is beyond it - following wall to find the opening.")
        self.face_bearing_relative(best_open_bearing(self.latest_scan))
        moved = False
        end = rospy.Time.now() + rospy.Duration(self.wall_follow_time)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            scan = self.latest_scan
            if scan is None or front_clearance(scan) < self.creep_min_front:
                break
            twist = Twist()
            twist.linear.x = self.wall_follow_speed
            self.cmd_pub.publish(twist)
            moved = True
            rate.sleep()
        stop_robot(self.cmd_pub)
        rospy.sleep(0.2)
        return moved

    def _attempt_corridor_creep(self, x, y):
        """If a narrow passage leads toward (x,y), align and creep through it.

        Returns True if the robot made forward progress. Lets tight corridors
        and doorways get traversed so they do not block full coverage of the
        bounded area.
        """
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
        if width <= 0.0 or width > self.narrow_passage_width:
            return False
        if abs(opening_bearing) > math.radians(10.0):
            self.face_bearing_relative(opening_bearing)
        rospy.loginfo("Narrow passage (~%.2fm) toward goal - creeping through.", width)
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

    def understand_wall(self):
        """Back one safe step, then 360-scan to read how the wall is built.

        Replaces 'inch diagonally into the wall'. After this the perimeter is
        re-analyzed from the new vantage, so the robot moves PARALLEL to the
        wall toward the part that is still unmapped.
        """
        rospy.loginfo("Reached a wall - backing one step and scanning to read it.")
        self.set_state("understand-wall")
        self.safe_reverse(self.back_step_dist)
        self.spin360("wall-scan")

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

    def cluster_openings(self, opening_rays, ox, oy):
        """Group adjacent no-hit rays into side-room/corridor openings."""
        if not opening_rays:
            return []
        angular_step = (2.0 * math.pi) / self.ray_count
        groups = []
        current = [opening_rays[0]]
        for prev, curr in zip(opening_rays, opening_rays[1:]):
            if abs(normalize_angle(curr[0] - prev[0])) <= 1.8 * angular_step:
                current.append(curr)
            else:
                groups.append(current)
                current = [curr]
        groups.append(current)
        # Merge wrap-around (last group adjacent to first across +/-pi).
        if len(groups) > 1:
            first, last = groups[0], groups[-1]
            if abs(normalize_angle(first[0][0] - last[-1][0])) <= 1.8 * angular_step:
                groups[0] = last + first
                groups.pop()

        openings = []
        for group in groups:
            bearings = [a for a, _ in group]
            dists = sorted(d for _, d in group)
            mean_bearing = circular_mean(bearings)
            mean_dist = dists[len(dists) // 2]
            half_width = 0.5 * len(group) * angular_step
            if half_width < math.pi / 2.0:
                est_width = 2.0 * mean_dist * math.sin(half_width)
            else:
                est_width = mean_dist
            if est_width < self.opening_min_width:
                continue
            approach_d = max(self.goal_min_clearance, mean_dist - self.approach_standoff)
            ax = ox + approach_d * math.cos(mean_bearing)
            ay = oy + approach_d * math.sin(mean_bearing)
            enter_d = mean_dist + self.enter_depth
            tx = ox + enter_d * math.cos(mean_bearing)
            ty = oy + enter_d * math.sin(mean_bearing)
            openings.append(
                {
                    "bearing": mean_bearing,
                    "dist": mean_dist,
                    "approach": (ax, ay),
                    "enter": (tx, ty),
                    "width": est_width,
                }
            )
        return openings

    def analyze_from(self, ox, oy):
        """Ray-cast from (ox,oy): build a standoff perimeter ring + opening list.

        Perimeter waypoints are created ONLY for wall segments that still
        border unknown space. A wall that is already fully mapped is skipped -
        there is no reason to approach it (the user's rule: "if it's mapped,
        no need to get close"). Openings are rays that reach UNKNOWN space
        (a real gap to a room/corridor), not rays that merely run free to max
        range inside the same room.
        """
        ring = []
        opening_rays = []
        prev_x = None
        prev_y = None
        for i in range(self.ray_count):
            angle = -math.pi + (2.0 * math.pi * i) / self.ray_count
            kind, dist, _, _, cmx, cmy = self.cast_ray(ox, oy, angle)
            if kind == "unknown":
                opening_rays.append((angle, dist))
                continue
            if kind != "hit":
                continue
            # Skip walls that are already fully observed.
            if not occupied_borders_unknown(self.map_msg, cmx, cmy, radius_cells=3):
                continue
            standoff_d = dist - self.wall_standoff
            if standoff_d < self.goal_min_clearance:
                continue
            wx = ox + standoff_d * math.cos(angle)
            wy = oy + standoff_d * math.sin(angle)
            sx, sy = self.snap_to_clear(wx, wy)
            if sx is None or not self.within_bounds(sx, sy):
                continue
            if prev_x is not None and math.hypot(sx - prev_x, sy - prev_y) < self.waypoint_spacing:
                continue
            # Travel tangentially along the wall for a smooth sweep.
            yaw = normalize_angle(angle + math.pi / 2.0)
            ring.append((angle, sx, sy, yaw))
            prev_x = sx
            prev_y = sy
        openings = self.cluster_openings(opening_rays, ox, oy)
        return ring, openings

    def perimeter_tour(self):
        """Sweep the room perimeter at standoff, entering each opening in order."""
        center = self.robot_xy()
        if center is None:
            return
        ox, oy = center
        ring, openings = self.analyze_from(ox, oy)
        rospy.loginfo("Perimeter plan: %d waypoints, %d openings.", len(ring), len(openings))
        if not ring and not openings:
            rospy.loginfo(
                "Nothing to sweep from here (walls already mapped, no openings) "
                "- going straight to cleanup.")
            return

        items = [(bearing, "wp", x, y, yaw) for (bearing, x, y, yaw) in ring]
        items += [(op["bearing"], "opening", op) for op in openings]
        items.sort(key=lambda t: t[0])

        for item in items:
            if rospy.is_shutdown() or self.timed_out():
                return
            if self.total_stuck >= self.global_max_stuck:
                rospy.logwarn(
                    "Hit global stuck limit (%d) during sweep - moving to cleanup.",
                    self.global_max_stuck)
                return
            if item[1] == "wp":
                self.set_state("sweep")
                self.drive_to(item[2], item[3], facing=item[4])
            else:
                self.visit_room(item[2])

    def visit_room(self, opening):
        ax, ay = opening["approach"]
        if self.is_opening_visited(ax, ay):
            return
        self.set_state("opening-approach")
        reached = self.drive_to(ax, ay, facing=opening["bearing"])
        self.visited_openings.append((ax, ay))
        if not reached:
            rospy.logwarn("Could not reach opening entrance - skipping it.")
            return

        self.spin360("opening-spin")

        scan = self.latest_scan
        if scan is None:
            return
        left, right, front = side_clearances(scan)
        opening_bearing, width = doorway_opening(scan)
        is_pocket = left <= self.pocket_clearance and right <= self.pocket_clearance
        if front <= self.deadend_front and is_pocket:
            rospy.logwarn(
                "Opening is a dead-end pocket (L%.2f R%.2f F%.2f) - not entering.",
                left, right, front)
            return

        if not self.enter_passage(opening, opening_bearing, width):
            rospy.logwarn("Could not enter the room - skipping.")
            return

        room_center = self.compute_room_center()
        if room_center is not None:
            self.set_state("room-center")
            self.drive_to(room_center[0], room_center[1], timeout=self.per_goal_timeout * 1.5)
        self.spin360("room-spin")
        self.exit_room(opening)

    def enter_passage(self, opening, opening_bearing, width):
        """Cross the doorway: creep through if narrow, else drive in via move_base."""
        if abs(opening_bearing) > math.radians(10.0):
            self.face_bearing_relative(opening_bearing)
        if width <= self.narrow_passage_width:
            self.set_state("creep-in")
            return creep_through_passage(
                self.cmd_pub,
                self._get_scan,
                goal_bearing=0.0,
                speed=self.creep_speed,
                max_duration=self.creep_duration,
                min_front=self.creep_min_front,
            )
        self.set_state("enter")
        tx, ty = opening["enter"]
        return self.drive_to(tx, ty, timeout=self.per_goal_timeout)

    def exit_room(self, opening):
        ax, ay = opening["approach"]
        self.set_state("exit")
        if self.drive_to(ax, ay, timeout=self.per_goal_timeout * 1.5):
            return
        # Doorway too tight for move_base on the way out: turn around and creep.
        self.face_bearing(normalize_angle(opening["bearing"] + math.pi))
        creep_through_passage(
            self.cmd_pub,
            self._get_scan,
            goal_bearing=0.0,
            speed=self.creep_speed,
            max_duration=self.creep_duration,
            min_front=self.creep_min_front,
        )

    def explore_frontiers(self):
        """Full-coverage frontier exploration of the WHOLE reachable space.

        After the structured local sweep, this drives to every reachable
        frontier - through doorways, into other rooms, and OUT of the building
        to map the exterior too - placing each goal at a safe standoff and doing
        a periodic 360 scan so new areas are mapped well. There is no virtual
        boundary, so it does not stop when the first room is done; it ends only
        when no reachable frontier remains (or on timeout / repeated stuck).
        """
        self.set_state("explore")
        self.consecutive_stuck = 0
        clearance = self._clearance_cells()
        last_ratio = 1.0
        last_improve = rospy.Time.now()
        reached = 0
        blacklist_resets = 0
        max_blacklist_resets = int(rospy.get_param("~max_blacklist_resets", 2))

        while not rospy.is_shutdown():
            if self.timed_out():
                rospy.logwarn("Exploration hit max runtime - stopping.")
                break

            cells = find_frontier_cells(self.map_msg)
            clusters = cluster_frontier_cells(cells)
            robot = self.robot_xy() or (0.0, 0.0)
            rx, ry = robot
            goals = []
            for cluster in clusters:
                if len(cluster) < self.min_frontier_cells:
                    continue
                goal = frontier_goal_from_cluster(self.map_msg, cluster, clearance_cells=clearance)
                if goal is None:
                    continue
                gx, gy, gyaw = goal
                if self.is_blacklisted(gx, gy) or not self.within_bounds(gx, gy):
                    continue
                goals.append((gx, gy, gyaw, len(cluster)))

            self.frontier_pub.publish(Int32(data=len(goals)))
            if not goals:
                rospy.loginfo("No reachable frontiers left - whole reachable area mapped.")
                break

            goals.sort(
                key=lambda g: g[3] - self.distance_penalty * math.hypot(g[0] - rx, g[1] - ry),
                reverse=True)
            gx, gy, gyaw, _ = goals[0]
            if self.drive_to(gx, gy, facing=gyaw):
                reached += 1
                # Reaching a new frontier IS progress. Reset the no-gain timer
                # so a long transit across already-mapped space (e.g. driving
                # to a far room like the kitchen) does not abort exploration.
                last_improve = rospy.Time.now()
                if reached % self.frontier_scan_every == 0:
                    self.spin360("frontier-scan")

            ratio = self._current_unknown_ratio()
            self.unknown_pub.publish(Float32(data=ratio))
            if ratio < last_ratio - 0.005:
                last_ratio = ratio
                last_improve = rospy.Time.now()
            elif (rospy.Time.now() - last_improve).to_sec() > self.global_progress_timeout:
                # No coverage gain and not reaching frontiers for a long time.
                # Before giving up, drop the blacklist a couple of times: a
                # frontier (e.g. the kitchen) may have been blacklisted during
                # an earlier stuck episode but be reachable now.
                if blacklist_resets < max_blacklist_resets and self.blacklist:
                    blacklist_resets += 1
                    rospy.logwarn(
                        "Stalled for %.0fs - clearing blacklist (reset %d/%d) and retrying.",
                        self.global_progress_timeout, blacklist_resets, max_blacklist_resets)
                    self.blacklist = []
                    self.consecutive_stuck = 0
                    last_improve = rospy.Time.now()
                    continue
                rospy.logwarn(
                    "No reachable progress for %.0fs - ending exploration.",
                    self.global_progress_timeout)
                break

            if self.consecutive_stuck >= self.max_consecutive_stuck:
                # Don't end outright - clear the blacklist once and keep trying
                # so a far reachable room is not abandoned over local stalls.
                if blacklist_resets < max_blacklist_resets and self.blacklist:
                    blacklist_resets += 1
                    rospy.logwarn(
                        "Repeatedly stuck - clearing blacklist (reset %d/%d) and retrying.",
                        blacklist_resets, max_blacklist_resets)
                    self.blacklist = []
                    self.consecutive_stuck = 0
                    continue
                rospy.logwarn("Repeatedly stuck - ending exploration.")
                break
            if self.total_stuck >= self.global_max_stuck:
                rospy.logwarn(
                    "Hit global stuck limit (%d) - ending exploration.",
                    self.global_max_stuck)
                break

    def finish(self):
        self.set_state("done")
        self.client.cancel_all_goals()
        stop_robot(self.cmd_pub)
        ratio = self._current_unknown_ratio()
        self.unknown_pub.publish(Float32(data=ratio))
        self.frontier_pub.publish(Int32(data=0))
        self.complete_pub.publish(Bool(data=True))
        rospy.loginfo(
            "Structured mapping complete. Unknown ~%.1f%%. Monitor will save the map.",
            ratio * 100.0)
        rospy.spin()

    def run(self):
        self.wait_for_map()
        self.wait_for_scan()
        self.start_time = rospy.Time.now()
        self.mark_progress()

        self.bootstrap()
        self.spin360("seed-spin")
        self.signal_started()

        center = self.compute_room_center()
        if center is not None:
            self.set_state("go-to-center")
            self.drive_to(center[0], center[1], timeout=self.per_goal_timeout * 1.5)
        self.spin360("center-spin")

        self.perimeter_tour()
        self.explore_frontiers()
        self.finish()


def main():
    rospy.init_node("structured_mapper")
    try:
        StructuredMapper().run()
    except rospy.ROSException as exc:
        rospy.logerr("Structured mapper failed: %s", exc)


if __name__ == "__main__":
    main()
