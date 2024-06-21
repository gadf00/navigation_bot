#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from navigation_bot.msg import OperationStatusMsg

class ChooseWaypoint:

    def __init__(self):
        rospy.init_node("choose_waypoint", anonymous=True)
        rospy.Subscriber("/operation", String, self.start_choosing)
        self.waypoint_pub = rospy.Publisher("/selected_waypoint", String, queue_size=10)
        self.status_pub = rospy.Publisher("/operation_status", OperationStatusMsg, queue_size=10)
        rospy.spin()

    def start_choosing(self, msg):
        if msg.data == "choose_waypoint":
            self.choose_destination()

    def choose_destination(self):
        waypoints = rospy.get_param("waypoints", {})
        if not waypoints:
            rospy.loginfo("No waypoints available.")
            self.operation_completed("No waypoints available.")
            return

        print("Choose a destination or type 'back' to return:")
        for i, point in enumerate(waypoints.keys()):
            print(f"{i}. {point}")

        while not rospy.is_shutdown():
            choice = input("Enter the number of the destination: ")
            if choice.lower() == 'back':
                self.operation_completed("Operation cancelled by user.")
                return

            if choice.isdigit() and int(choice) in range(len(waypoints)):
                selected_waypoint = list(waypoints.values())[int(choice)]
                self.publish_waypoint(selected_waypoint)
                break
            else:
                print("Invalid choice. Please try again.")

    def publish_waypoint(self, waypoint):
        waypoint_str = f"{waypoint[0]} {waypoint[1]} {waypoint[2]}"
        rospy.sleep(1)  # Assicura che il publisher sia pronto
        self.waypoint_pub.publish(waypoint_str)
        rospy.loginfo(f"Waypoint {waypoint_str} published for navigation")
        self.operation_completed(f"Waypoint {waypoint_str} published for navigation")

    def operation_completed(self, message):
        status_msg = OperationStatusMsg()
        status_msg.completed = True
        status_msg.message = message
        self.status_pub.publish(status_msg)

if __name__ == "__main__":
    try:
        ChooseWaypoint()
    except Exception as e:
        rospy.logerr(e)
