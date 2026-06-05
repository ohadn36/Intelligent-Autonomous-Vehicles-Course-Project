#!/usr/bin/env python3
"""
Waypoint patrol node for Summit XL autonomous navigation project.

Sends a sequence of navigation goals to move_base using actionlib.
Edit WAYPOINTS below after you save your map and know valid coordinates.
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


# Edit these coordinates after mapping your environment in RViz.
# Tip: use "2D Pose Estimate" and "2D Nav Goal" in RViz to find good values.
WAYPOINTS = [
    {"x": 1.0, "y": 0.0, "yaw": 0.0},
    {"x": 2.0, "y": 1.0, "yaw": 1.57},
    {"x": 1.0, "y": 2.0, "yaw": 3.14},
    {"x": 0.0, "y": 0.0, "yaw": 0.0},
]


def make_goal(x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(x, y, 0.0)
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*q)
    return goal


def main():
    rospy.init_node("waypoint_patrol")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("move_base connected. Starting patrol with %d waypoints.", len(WAYPOINTS))

    for i, wp in enumerate(WAYPOINTS, start=1):
        goal = make_goal(wp["x"], wp["y"], wp["yaw"])
        rospy.loginfo("Sending waypoint %d/%d: x=%.2f y=%.2f yaw=%.2f",
                      i, len(WAYPOINTS), wp["x"], wp["y"], wp["yaw"])
        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(120.0))

        if not finished:
            rospy.logwarn("Waypoint %d timed out.", i)
            client.cancel_goal()
            continue

        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint %d reached.", i)
        else:
            rospy.logwarn("Waypoint %d failed with state %s", i, state)

        rospy.sleep(1.0)

    rospy.loginfo("Patrol complete.")
    rospy.spin()


if __name__ == "__main__":
    main()
