#!/usr/bin/env python3

import rospy
import yaml
from navigation_bot.msg import NavigationStatusMsg, StartStatusMsg
from navigation_bot.srv import Waypoint
import sys

class ChooseWaypoint:

    def __init__(self):
        rospy.init_node("choose_waypoint", anonymous=True)
        self.pose_calculated = False
        rospy.Subscriber("/start_status", StartStatusMsg, self.start_status_callback)
        rospy.Subscriber("/navigation_status", NavigationStatusMsg, self.navigation_status_callback)
        self.navigation_service = rospy.ServiceProxy('navigation_service', Waypoint)
        rospy.spin()

    def start_status_callback(self, msg):
        if msg.started and msg.message == "Position calculated":
            self.pose_calculated = True
            self.choose_destination()

    def navigation_status_callback(self, msg):
        if msg.status in ["stopped", "failure"]:
            self.choose_destination()

    def load_waypoints(self):
        waypoints_file = "/home/sium/catkin_ws/src/navigation_bot/scripts/waypoints.yaml"
        with open(waypoints_file, 'r') as f:
            waypoints = yaml.safe_load(f)
        return waypoints

    def choose_destination(self):
        if not self.pose_calculated:
            rospy.loginfo("Waiting for initial pose to be calculated...")
            return

        waypoints = self.load_waypoints()
        if not waypoints:
            rospy.loginfo("No waypoints available.")
            return

        print("Choose a destination:")
        for i, point in enumerate(waypoints.keys()):
            print(f"{i}. {point}")

        choice = input("Enter the number of the destination: ")

        if choice.isdigit() and int(choice) in range(len(waypoints)):
            selected_waypoint = list(waypoints.values())[int(choice)]
            self.send_waypoint(selected_waypoint)
        else:
            print("Invalid choice. Please try again.")
            self.choose_destination()

    def send_waypoint(self, waypoint):
        try:
            response = self.navigation_service(waypoint[0], waypoint[1], waypoint[2])
            rospy.loginfo(f"Navigation response: {response.esito}")
            self.choose_destination()  # Richiama la funzione di scelta del waypoint
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        ChooseWaypoint()
    except rospy.ROSInterruptException:
        sys.exit(0)
    except Exception as e:
        rospy.logerr(e)
        sys.exit(1)
