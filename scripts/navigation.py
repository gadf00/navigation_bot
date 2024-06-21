#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from navigation_bot.msg import OperationStatusMsg
from navigation_bot.srv import StopNavigation, StopNavigationResponse

class Navigation:

    def __init__(self):
        rospy.init_node("navigation", anonymous=True)
        rospy.Subscriber("/selected_waypoint", String, self.navigate)
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.status_pub = rospy.Publisher("/operation_status", OperationStatusMsg, queue_size=10)
        self.stop_service = rospy.Service('stop_navigation', StopNavigation, self.handle_stop_navigation)
        self.nav_client.wait_for_server()
        rospy.spin()

    def navigate(self, msg):
        waypoint = list(map(float, msg.data.split()))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.position.z = waypoint[2]
        goal.target_pose.pose.orientation.w = 1.0

        self.nav_client.send_goal(goal, done_cb=self.success_cb, feedback_cb=self.feedback_cb)
        self.nav_client.wait_for_result()

    def success_cb(self, status, result):
        rospy.loginfo(f"Navigation result: {status}")
        message = "Navigation completed" if status == 3 else "Navigation failed or cancelled"
        self.operation_completed(message)

    def feedback_cb(self, feedback):
        rospy.loginfo(f"Current position: {feedback.base_position.pose.position}")

    def operation_completed(self, message):
        status_msg = OperationStatusMsg()
        status_msg.completed = True
        status_msg.message = message
        self.status_pub.publish(status_msg)

    def handle_stop_navigation(self, req):
        self.nav_client.cancel_all_goals()
        response = StopNavigationResponse()
        response.success = True
        response.message = "Navigation stopped"
        self.operation_completed("Navigation stopped by user")
        return response

if __name__ == "__main__":
    try:
        Navigation()
    except Exception as e:
        rospy.logerr(e)
