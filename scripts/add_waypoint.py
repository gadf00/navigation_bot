#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from navigation_bot.msg import OperationStatusMsg
import yaml
import os

class RemoveWaypoint:

    def __init__(self):
        rospy.init_node("remove_waypoint", anonymous=True)
        self.waypoints_file = "/home/sium/catkin_ws/src/navigation_bot/scripts/waypoints.yaml"
        rospy.Subscriber("/operation", String, self.start_removing_waypoint)
        self.status_pub = rospy.Publisher("/operation_status", OperationStatusMsg, queue_size=10)
        rospy.spin()

    def start_removing_waypoint(self, msg):
        if msg.data == "remove_waypoint":
            self.remove_waypoint()

    def remove_waypoint(self):
        waypoints = rospy.get_param("waypoints", {})
        if not waypoints:
            rospy.loginfo("No waypoints available.")
            self.operation_completed("No waypoints available.")
            return

        print("Choose a waypoint to remove or type 'back' to return:")
        for i, point in enumerate(waypoints.keys()):
            print(f"{i}. {point}")

        while not rospy.is_shutdown():
            choice = input("Enter the number of the waypoint to remove: ")
            if choice.lower() == 'back':
                self.operation_completed("Operation cancelled by user.")
                return

            if choice.isdigit() and int(choice) in range(len(waypoints)):
                point = list(waypoints.keys())[int(choice)]
                del waypoints[point]
                rospy.set_param("waypoints", waypoints)
                self.save_waypoints(waypoints)
                rospy.loginfo(f"Waypoint {point} removed.")
                self.operation_completed(f"Waypoint {point} removed.")
                break
            else:
                print("Invalid choice. Please try again.")

    def save_waypoints(self, waypoints):
        with open(self.waypoints_file, 'w') as f:
            yaml.safe_dump(waypoints, f)

    def operation_completed(self, message):
        status_msg = OperationStatusMsg()
        status_msg.completed = True
        status_msg.message = message
        self.status_pub.publish(status_msg)

if __name__ == "__main__":
    try:
        RemoveWaypoint()
    except Exception as e:
        rospy.logerr(e)
