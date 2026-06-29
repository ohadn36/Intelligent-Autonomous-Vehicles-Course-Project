#!/usr/bin/env python3
"""
Publish AMCL initial pose from current odom->base_footprint transform.
Use when the robot spawn position matches the map origin area (course simulation).
"""

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    rospy.init_node("set_initial_pose_from_odom")
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    listener = tf.TransformListener()

    rospy.loginfo("Waiting for odom -> base_footprint transform...")
    listener.waitForTransform("odom", "base_footprint", rospy.Time(0), rospy.Duration(30.0))

    # Publish at most twice. AMCL resets its particle cloud on EVERY
    # /initialpose, so the old 10x loop kept re-scattering particles —
    # delaying convergence and overriding a manual 2D Pose Estimate. Two
    # publishes cover AMCL subscribing a moment late; the latched publisher
    # serves anyone who joins afterward.
    rate = rospy.Rate(1)
    published = 0
    for attempt in range(5):
        try:
            (trans, rot) = listener.lookupTransform("odom", "base_footprint", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logwarn("TF lookup failed (attempt %d): %s", attempt + 1, ex)
            rate.sleep()
            continue

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = trans[0]
        msg.pose.pose.position.y = trans[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = rot[0]
        msg.pose.pose.orientation.y = rot[1]
        msg.pose.pose.orientation.z = rot[2]
        msg.pose.pose.orientation.w = rot[3]
        # Moderate covariance — not too tight, not too loose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        pub.publish(msg)
        published += 1
        rospy.loginfo("Published initial pose from odom: x=%.2f y=%.2f", trans[0], trans[1])
        if published >= 2:
            break
        rospy.sleep(1.0)

    if published == 0:
        rospy.logwarn("Could not set initial pose from odom — use RViz '2D Pose Estimate'.")
    else:
        rospy.loginfo("Initial pose set. Wait for AMCL to converge, then send a goal.")


if __name__ == "__main__":
    main()
