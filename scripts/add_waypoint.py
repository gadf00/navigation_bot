#!/usr/bin/env python3

import rospy
import yaml
from nav_msgs.msg import Odometry
from navigation_bot.msg import NavigationStatusMsg

class AddWaypoint:

    def __init__(self):
        rospy.init_node("add_waypoint", anonymous=True)
        self.waypoints_file = "/home/sium/catkin_ws/src/navigation_bot/scripts/waypoints.yaml"
        rospy.Subscriber("/navigation_status", NavigationStatusMsg, self.navigation_status_callback)
        self.navigation_status = "stopped"
        rospy.sleep(1)  # Attendere un momento per ricevere lo stato della navigazione
        self.show_menu()

    def navigation_status_callback(self, msg):
        self.navigation_status = msg.status

    def show_menu(self):
        while not rospy.is_shutdown():
            print("Choose how to add a waypoint or type 'back' to return:")
            print("1. Current position")
            print("2. Enter coordinates")

            choice = input("Enter the number of the option: ")
            if choice.lower() == 'back':
                rospy.signal_shutdown("User requested shutdown")
                break

            if choice == "1":
                if self.navigation_status == "moving":
                    rospy.logwarn("Cannot add waypoint of current position while robot is in motion.")
                else:
                    self.add_current_position()
            elif choice == "2":
                self.add_coordinates()
            else:
                print("Invalid choice. Please try again.")

    def add_current_position(self):
        name = input("Enter the name of the waypoint: ")
        gazebo_pose = rospy.wait_for_message("/odom", Odometry)
        point = [gazebo_pose.pose.pose.position.x, gazebo_pose.pose.pose.position.y, gazebo_pose.pose.pose.position.z]
        self.save_waypoint(name, point)

    def add_coordinates(self):
        name = input("Enter the name of the waypoint: ")
        x = float(input("Enter x coordinate: "))
        y = float(input("Enter y coordinate: "))
        z = float(input("Enter z coordinate: "))
        point = [x, y, z]
        self.save_waypoint(name, point)

    def save_waypoint(self, name, point):
        waypoints = self.load_waypoints()
        waypoints[name] = point
        with open(self.waypoints_file, 'w') as f:
            yaml.safe_dump(waypoints, f)
        rospy.loginfo(f"Waypoint '{name}' added successfully.")
        print(f"Waypoint '{name}' added successfully.")

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
        AddWaypoint()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(e)
