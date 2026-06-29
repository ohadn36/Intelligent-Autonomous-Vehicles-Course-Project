#!/usr/bin/env python3
"""
Convert RViz map clicks (/clicked_point) into move_base navigation goals.

Usage in RViz:
  1. Select "Publish Point" tool
  2. Click anywhere on the map
  3. Robot plans a path and drives there autonomously
"""

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class ClickGoalNode(object):
    def __init__(self):
        self.default_yaw = rospy.get_param("~default_yaw", 0.0)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("move_base ready. Click a point in RViz (Publish Point tool).")
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb, queue_size=1)

    def clicked_point_cb(self, msg):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = msg.point.x
        goal.target_pose.pose.position.y = msg.point.y
        goal.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.default_yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Goal sent to (%.2f, %.2f)", msg.point.x, msg.point.y)
        self.client.send_goal(goal, done_cb=self.done_cb)

    def done_cb(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation succeeded — goal reached.")
        elif status == GoalStatus.PREEMPTED:
            rospy.logwarn("Navigation preempted (new goal or cancel).")
        else:
            rospy.logwarn("Navigation failed with status: %s", status)


def main():
    rospy.init_node("click_goal")
    ClickGoalNode()
    rospy.spin()


if __name__ == "__main__":
    main()
