#!/usr/bin/env python3
"""Shared helpers for autonomous exploration nodes."""

import math
from collections import deque

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

CMD_VEL_CANDIDATES = (
    "/cmd_vel",
    "/mobile_base/commands/velocity",
    "/kobuki/commands/velocity",
)

FREE_THRESH = 50
UNKNOWN = -1
OCCUPIED = 100


def resolve_cmd_vel_topic(configured=""):
    if configured:
        return configured
    names = {name for name, _ in rospy.get_published_topics()}
    for candidate in CMD_VEL_CANDIDATES:
        if candidate in names:
            return candidate
    return "/cmd_vel"


def valid_range(value, scan):
    if value is None or math.isinf(value) or math.isnan(value):
        return False
    return scan.range_min <= value <= scan.range_max + 0.01


def yaw_from_quaternion(quaternion):
    return tf.transformations.euler_from_quaternion(quaternion)[2]


def get_robot_pose(listener, map_frame="map", base_frame="base_footprint", timeout=5.0):
    listener.waitForTransform(map_frame, base_frame, rospy.Time(0), rospy.Duration(timeout))
    trans, rot = listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
    return trans[0], trans[1], yaw_from_quaternion(rot)


def stop_robot(cmd_pub):
    cmd_pub.publish(Twist())


def drive_cmd(cmd_pub, linear_x=0.0, angular_z=0.0, duration=0.0):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    rate = rospy.Rate(20)
    end = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        cmd_pub.publish(twist)
        rate.sleep()
    stop_robot(cmd_pub)
    rospy.sleep(0.15)


def multi_angle_scan(cmd_pub, rotate_speed=0.35, pause_sec=2.5, steps=4):
    """Pause and rotate at several headings so gmapping sees doors from new angles."""
    rospy.loginfo("Multi-angle scan verification (%d stops, %.1fs each)", steps, pause_sec)
    for step in range(steps):
        rospy.sleep(pause_sec)
        angle = (2.0 * math.pi) / float(steps)
        drive_cmd(cmd_pub, angular_z=rotate_speed, duration=angle / rotate_speed)
    rospy.sleep(pause_sec)


def best_open_bearing(scan):
    points = []
    angle = scan.angle_min
    for dist in scan.ranges:
        if valid_range(dist, scan):
            points.append((angle, dist))
        angle += scan.angle_increment
    if not points:
        return 0.0

    sector_width = math.radians(60.0)
    step = math.radians(5.0)
    best_angle = 0.0
    best_score = -1.0
    test_angle = scan.angle_min
    while test_angle <= scan.angle_max:
        sector = []
        for bearing, dist in points:
            delta = bearing - test_angle
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            if abs(delta) <= sector_width * 0.5:
                sector.append(dist)
        if sector:
            sector.sort()
            score = 0.7 * sector[len(sector) // 2] + 0.3 * (sum(sector) / float(len(sector)))
            if score > best_score:
                best_score = score
                best_angle = test_angle
        test_angle += step

    while best_angle > math.pi:
        best_angle -= 2.0 * math.pi
    while best_angle < -math.pi:
        best_angle += 2.0 * math.pi
    return best_angle


def front_clearance(scan, half_angle_deg=45.0):
    points = []
    angle = scan.angle_min
    half = math.radians(half_angle_deg)
    for dist in scan.ranges:
        if valid_range(dist, scan) and abs(angle) <= half:
            points.append(dist)
        angle += scan.angle_increment
    return min(points) if points else scan.range_max


def rear_clearance(scan, half_angle_deg=50.0):
    """Minimum laser range in the rear sector (bearings near +/-180 deg).

    Used to guarantee a reverse maneuver will not back the robot into a wall
    or obstacle. Returns range_max when the rear is out of the laser's FOV
    (many indoor LiDARs do not see directly behind), which the caller must
    treat as "unknown behind" rather than "clear".
    """
    points = []
    angle = scan.angle_min
    half = math.radians(half_angle_deg)
    for dist in scan.ranges:
        if valid_range(dist, scan):
            delta = math.pi - abs(angle)  # 0 at exactly behind
            if abs(delta) <= half:
                points.append(dist)
        angle += scan.angle_increment
    return min(points) if points else scan.range_max


def rear_in_fov(scan, half_angle_deg=50.0):
    """True if the laser actually has beams in the rear sector."""
    angle = scan.angle_min
    half = math.radians(half_angle_deg)
    while angle <= scan.angle_max:
        if abs(math.pi - abs(angle)) <= half:
            return True
        angle += scan.angle_increment
    return False


def side_clearances(scan, front_half_deg=40.0):
    """Return (left_min, right_min, front_min) distances in the robot frame."""
    left = []
    right = []
    front = []
    angle = scan.angle_min
    front_half = math.radians(front_half_deg)
    for dist in scan.ranges:
        if valid_range(dist, scan):
            if abs(angle) <= front_half:
                front.append(dist)
            elif angle > 0.0:
                left.append(dist)
            else:
                right.append(dist)
        angle += scan.angle_increment

    fallback = scan.range_max
    return (
        min(left) if left else fallback,
        min(right) if right else fallback,
        min(front) if front else fallback,
    )


def opening_width_at_bearing(scan, bearing, half_width_rad=0.45):
    """Estimate physical opening width at a bearing using laser sector depth."""
    ranges = []
    angle = scan.angle_min
    for dist in scan.ranges:
        if valid_range(dist, scan):
            delta = angle - bearing
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            if abs(delta) <= half_width_rad:
                ranges.append(dist)
        angle += scan.angle_increment
    if not ranges:
        return 0.0
    depth = min(ranges)
    return 2.0 * depth * math.sin(half_width_rad)


def doorway_opening(scan, preferred_bearing=None):
    """Return (bearing_rad, width_m) for the best passage opening."""
    if preferred_bearing is None:
        bearing = best_open_bearing(scan)
    else:
        bearing = preferred_bearing
    width = opening_width_at_bearing(scan, bearing)
    open_bearing = best_open_bearing(scan)
    open_width = opening_width_at_bearing(scan, open_bearing)
    if open_width > width:
        return open_bearing, open_width
    return bearing, width


def creep_through_passage(
    cmd_pub,
    scan_getter,
    goal_bearing=0.0,
    speed=0.08,
    max_duration=4.5,
    min_front=0.26,
    center_gain=0.9,
):
    """
    Drive slowly while centering in a gap (equalize left/right clearance).

    Used when move_base/DWA cannot fit through a narrow doorway.
    """
    rate = rospy.Rate(10)
    end = rospy.Time.now() + rospy.Duration(max_duration)
    moved = False
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        scan = scan_getter()
        if scan is None:
            rate.sleep()
            continue

        left, right, front = side_clearances(scan)
        if front < min_front:
            break

        # Keep the robot in the middle of the opening; bias toward goal bearing.
        centering = center_gain * (left - right)
        goal_bias = 0.35 * math.sin(goal_bearing)
        angular = max(-0.35, min(0.35, centering + goal_bias))

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angular
        cmd_pub.publish(twist)
        moved = True
        rate.sleep()

    stop_robot(cmd_pub)
    rospy.sleep(0.2)
    return moved


def map_indices(map_msg, wx, wy):
    mx = int((wx - map_msg.info.origin.position.x) / map_msg.info.resolution)
    my = int((wy - map_msg.info.origin.position.y) / map_msg.info.resolution)
    return mx, my


def map_to_world(map_msg, mx, my):
    wx = map_msg.info.origin.position.x + (mx + 0.5) * map_msg.info.resolution
    wy = map_msg.info.origin.position.y + (my + 0.5) * map_msg.info.resolution
    return wx, wy


def cell_value(map_msg, mx, my):
    if mx < 0 or my < 0 or mx >= map_msg.info.width or my >= map_msg.info.height:
        return UNKNOWN
    index = my * map_msg.info.width + mx
    return map_msg.data[index]


def is_free(value):
    return 0 <= value < FREE_THRESH


def is_unknown(value):
    return value < 0


def cell_region_free(map_msg, mx, my, radius_cells=2):
    """True only if every cell in a (2r+1)x(2r+1) box around (mx,my) is free.

    Used to guarantee a goal sits in space the robot footprint can occupy,
    not on an obstacle edge or inside inflation.
    """
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if not is_free(cell_value(map_msg, mx + dx, my + dy)):
                return False
    return True


def nearest_free_cell(map_msg, mx, my, max_radius=10, clearance_cells=2):
    """Spiral outward from (mx,my) to the closest cell with free footprint clearance."""
    if cell_region_free(map_msg, mx, my, clearance_cells):
        return mx, my
    for r in range(1, max_radius + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                if cell_region_free(map_msg, mx + dx, my + dy, clearance_cells):
                    return mx + dx, my + dy
    return None


def coverage_stats(map_msg):
    """Return unknown ratio inside the bounding box of known cells."""
    if map_msg is None or map_msg.info.width == 0:
        return 1.0, 0, 0, 0

    data = map_msg.data
    width = map_msg.info.width
    height = map_msg.info.height

    min_x = width
    min_y = height
    max_x = -1
    max_y = -1
    unknown = 0
    known = 0

    for my in range(height):
        row = my * width
        for mx in range(width):
            value = data[row + mx]
            if is_unknown(value):
                unknown += 1
            else:
                known += 1
                min_x = min(min_x, mx)
                min_y = min(min_y, my)
                max_x = max(max_x, mx)
                max_y = max(max_y, my)

    if max_x < 0:
        return 1.0, unknown, known, 0

    bbox_unknown = 0
    bbox_total = 0
    for my in range(min_y, max_y + 1):
        row = my * width
        for mx in range(min_x, max_x + 1):
            bbox_total += 1
            if is_unknown(data[row + mx]):
                bbox_unknown += 1

    ratio = float(bbox_unknown) / float(bbox_total) if bbox_total else 1.0
    return ratio, unknown, known, bbox_total


def is_occupied(value, occupied_thresh=65):
    """True if an occupancy-grid cell is a (mapped) obstacle."""
    return value >= occupied_thresh


def _has_unknown_neighbor(map_msg, mx, my):
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
        if is_unknown(cell_value(map_msg, mx + dx, my + dy)):
            return True
    return False


def occupied_borders_unknown(map_msg, mx, my, radius_cells=2):
    """True if an occupied cell still borders unknown space within radius.

    A wall cell whose neighborhood is fully known (occupied/free) has been
    completely observed, so there is no reason to approach it again. One that
    still touches unknown space hides unmapped area behind/around it and is
    worth getting closer to.
    """
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if is_unknown(cell_value(map_msg, mx + dx, my + dy)):
                return True
    return False


def find_frontier_cells(map_msg):
    """Return free cells that border unknown space (frontier cells)."""
    width = map_msg.info.width
    height = map_msg.info.height
    data = map_msg.data
    cells = []
    for my in range(height):
        row = my * width
        for mx in range(width):
            if not is_free(data[row + mx]):
                continue
            if _has_unknown_neighbor(map_msg, mx, my):
                cells.append((mx, my))
    return cells


def cluster_frontier_cells(cells):
    """Group adjacent frontier cells (4-connectivity) into clusters."""
    cell_set = set(cells)
    clusters = []
    while cell_set:
        start = cell_set.pop()
        queue = deque([start])
        cluster = [start]
        while queue:
            mx, my = queue.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (mx + dx, my + dy)
                if neighbor in cell_set:
                    cell_set.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)
        clusters.append(cluster)
    return clusters


def frontier_goal_from_cluster(map_msg, cluster, clearance_cells=2):
    """Place a goal on a free, footprint-clear cell that faces the unknown.

    Returns (gx, gy, yaw) in world coordinates, or None when no reachable free
    cell with the requested clearance exists near the cluster.
    """
    sum_mx = 0.0
    sum_my = 0.0
    push_x = 0.0
    push_y = 0.0
    push_count = 0
    for mx, my in cluster:
        sum_mx += mx
        sum_my += my
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
            if is_unknown(cell_value(map_msg, mx + dx, my + dy)):
                push_x += dx
                push_y += dy
                push_count += 1
    if push_count == 0:
        return None
    cmx = int(round(sum_mx / float(len(cluster))))
    cmy = int(round(sum_my / float(len(cluster))))
    free = nearest_free_cell(map_msg, cmx, cmy, max_radius=10, clearance_cells=clearance_cells)
    if free is None:
        return None
    gx, gy = map_to_world(map_msg, free[0], free[1])
    yaw = math.atan2(push_y, push_x)
    return gx, gy, yaw
