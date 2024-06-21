#!/usr/bin/env python3

import rospy
from navigation_bot.srv import StopNavigation

def stop_navigation():
    rospy.init_node('stop_navigation_client')
    rospy.wait_for_service('stop_navigation')
    try:
        stop_nav = rospy.ServiceProxy('stop_navigation', StopNavigation)
        response = stop_nav()
        rospy.loginfo(response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    stop_navigation()
