#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from navigation_bot.srv import Waypoint, WaypointResponse, StopNavigation, StopNavigationResponse
from navigation_bot.msg import NavigationStatusMsg
import math
import time

class Navigation:

    def __init__(self):
        rospy.init_node("navigation", anonymous=True)
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.status_pub = rospy.Publisher("/navigation_status", NavigationStatusMsg, queue_size=10)
        self.nav_client.wait_for_server()
        self.navigation_service = rospy.Service('navigation_service', Waypoint, self.handle_navigation_request)
        self.stop_service = rospy.Service('stop_navigation', StopNavigation, self.handle_stop_navigation)
        self.last_log_time = 0
        self.current_goal = None
        self.navigation_stopped_by_user = False
        self.is_navigating = False
        self.publish_navigation_status("stopped", "Navigation node started")
        rospy.Timer(rospy.Duration(1), self.publish_moving_status)
        rospy.spin()

    def handle_navigation_request(self, req):
        self.navigation_stopped_by_user = False
        self.is_navigating = True
        self.current_goal = (req.x, req.y, req.z)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = req.x
        goal.target_pose.pose.position.y = req.y
        goal.target_pose.pose.position.z = req.z
        goal.target_pose.pose.orientation.w = 1.0

        self.nav_client.send_goal(goal, done_cb=self.success_cb, feedback_cb=self.feedback_cb)
        self.publish_navigation_status("moving", "Navigation started")
        finished = self.nav_client.wait_for_result()
        response = WaypointResponse()

        if self.navigation_stopped_by_user:
            response.esito = "Navigation stopped by user"
        elif finished:
            response.esito = "Navigation completed"
        else:
            response.esito = "Navigation failed or cancelled"

        self.is_navigating = False
        return response

    def success_cb(self, status, result):
        if status == 3 and not self.navigation_stopped_by_user:
            message = "Navigation completed"
            self.publish_navigation_status("stopped", message)
        elif self.navigation_stopped_by_user:
            message = "Navigation stopped by user"
            self.publish_navigation_status("stopped", message)
        else:
            message = "Navigation failed"
            self.publish_navigation_status("failure", message)

    def feedback_cb(self, feedback):
        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:
            current_position = feedback.base_position.pose.position
            distance_remaining = self.calculate_distance(current_position, self.current_goal)
            rospy.loginfo(f"Current position: x={current_position.x}, y={current_position.y}, z={current_position.z}")
            rospy.loginfo(f"Distance remaining: {distance_remaining:.2f} meters")
            self.last_log_time = current_time

    def calculate_distance(self, current_position, goal_position):
        return math.sqrt(
            (current_position.x - goal_position[0]) ** 2 +
            (current_position.y - goal_position[1]) ** 2 +
            (current_position.z - goal_position[2]) ** 2
        )

    def handle_stop_navigation(self, req):
        self.navigation_stopped_by_user = True
        self.nav_client.cancel_all_goals()
        response = StopNavigationResponse()
        response.success = True
        response.message = "Navigation stopped"
        self.publish_navigation_status("stopped", "Navigation stopped by user")
        self.is_navigating = False
        return response

    def publish_navigation_status(self, status, message):
        nav_status_msg = NavigationStatusMsg()
        nav_status_msg.status = status
        nav_status_msg.message = message
        self.status_pub.publish(nav_status_msg)

    def publish_moving_status(self, event):
        if self.is_navigating:
            self.publish_navigation_status("moving", "Navigation in progress")

if __name__ == "__main__":
    try:
        Navigation()
    except Exception as e:
        rospy.logerr(e)
