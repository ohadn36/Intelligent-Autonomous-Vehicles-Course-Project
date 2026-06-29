#!/usr/bin/env python3
"""Print human-readable move_base status updates to the terminal."""

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray

STATUS_MAP = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "MOVING",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "GOAL REACHED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


class NavigationStatusNode(object):
    def __init__(self):
        self.last_status = None
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_cb, queue_size=1)
        rospy.loginfo("Navigation status monitor started.")

    def status_cb(self, msg):
        if not msg.status_list:
            return
        # Last entry is the current/most-recent goal; [0] can be a stale goal.
        status = msg.status_list[-1].status
        if status == self.last_status:
            return
        self.last_status = status
        name = STATUS_MAP.get(status, "UNKNOWN(%d)" % status)
        rospy.loginfo("[Navigation] %s", name)


def main():
    rospy.init_node("navigation_status")
    NavigationStatusNode()
    rospy.spin()


if __name__ == "__main__":
    main()
