#!/usr/bin/env python3
"""Print one-time launch instructions for mapping or navigation."""

import rospy

HINTS = {
    "navigation": (
        "Navigation ready: set 2D Pose Estimate in RViz, then Publish Point to navigate."
    ),
    "mapping": (
        "Mapping ready: wait for autonomous scan, then follow terminal prompts."
    ),
}


def main():
    rospy.init_node("launch_hint")
    mode = rospy.get_param("~mode", "navigation")
    rospy.logwarn(HINTS.get(mode, HINTS["navigation"]))


if __name__ == "__main__":
    main()
