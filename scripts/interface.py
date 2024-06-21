#!/usr/bin/env python3

import rospy
from navigation_bot.msg import StartStatusMsg, OperationStatusMsg
from std_msgs.msg import String

class Interface:

    def __init__(self):
        rospy.init_node("interface", anonymous=True)
        self.operation_pub = rospy.Publisher("/operation", String, queue_size=10)
        rospy.Subscriber("/start_status", StartStatusMsg, self.status_callback)
        rospy.Subscriber("/operation_status", OperationStatusMsg, self.operation_status_callback)
        self.position_calculated = False
        self.operation_completed = False
        self.operation_message = ""
        self.show_interface()

    def status_callback(self, msg):
        if msg.started and msg.message == "Position calculated":
            self.position_calculated = True

    def show_interface(self):
        while not rospy.is_shutdown():
            if self.position_calculated:
                print("Choose an operation:")
                print("1. Navigation")
                print("2. Add Waypoint")
                print("3. Remove Waypoint")
                choice = input("Enter the number of the operation: ")

                operation_msg = String()
                if choice == "1":
                    operation_msg.data = "choose_waypoint"
                elif choice == "2":
                    operation_msg.data = "add_waypoint"
                elif choice == "3":
                    operation_msg.data = "remove_waypoint"
                else:
                    print("Invalid choice. Please try again.")
                    continue

                self.operation_pub.publish(operation_msg)
                self.wait_for_operation()

    def wait_for_operation(self):
        self.operation_completed = False
        while not self.operation_completed and not rospy.is_shutdown():
            rospy.sleep(1)  # Wait for the operation to complete

    def operation_status_callback(self, msg):
        if msg.completed:
            self.operation_completed = True
            self.operation_message = msg.message
            rospy.loginfo(msg.message)
            if "Navigation completed" in msg.message or "Navigation failed or cancelled" in msg.message:
                self.operation_pub.publish("choose_waypoint")

if __name__ == "__main__":
    try:
        Interface()
    except Exception as e:
        rospy.logerr(e)
