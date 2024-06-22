#!/usr/bin/env python3

import rospy
import yaml

class RemoveWaypoint:

    def __init__(self):
        rospy.init_node("remove_waypoint", anonymous=True)
        self.waypoints_file = "/home/sium/catkin_ws/src/navigation_bot/scripts/waypoints.yaml"
        self.remove_waypoint()

    def remove_waypoint(self):
        waypoints = self.load_waypoints()
        if not waypoints:
            rospy.loginfo("No waypoints available.")
            return

        print("Choose a waypoint to remove or type 'back' to return:")
        for i, point in enumerate(waypoints.keys()):
            print(f"{i}. {point}")

        choice = input("Enter the number of the waypoint to remove: ")
        if choice.lower() == 'back':
            rospy.signal_shutdown("User requested shutdown")
            return

        if choice.isdigit() and int(choice) in range(len(waypoints)):
            selected_waypoint = list(waypoints.keys())[int(choice)]
            del waypoints[selected_waypoint]
            self.save_waypoints(waypoints)
            print(f"Waypoint '{selected_waypoint}' removed successfully.")
            rospy.loginfo(f"Waypoint '{selected_waypoint}' removed successfully.")
        else:
            print("Invalid choice.")
            return

    def save_waypoints(self, waypoints):
        with open(self.waypoints_file, 'w') as f:
            yaml.safe_dump(waypoints, f)

    def load_waypoints(self):
        try:
            with open(self.waypoints_file, 'r') as f:
                waypoints = yaml.safe_load(f)
                if waypoints is None:
                    waypoints = {}
        except FileNotFoundError:
            waypoints = {}
        return waypoints

if __name__ == "__main__":
    try:
        RemoveWaypoint()
    except Exception as e:
        rospy.logerr(e)
