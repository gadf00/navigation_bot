#!/usr/bin/env python3

import rospy
from navigation_bot.msg import StartStatusMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class StartPosition:

    def __init__(self):
        rospy.init_node("start_position", anonymous=True)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.status_pub = rospy.Publisher("/start_status", StartStatusMsg, queue_size=10)
        self.set_pose()

    def set_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        gazebo_pose = rospy.wait_for_message("/odom", Odometry)
        pose_msg.pose.pose = gazebo_pose.pose.pose
        rospy.sleep(1)
        rospy.loginfo(f"Pose set to: {pose_msg}")
        self.pose_pub.publish(pose_msg)
        self.save_param(gazebo_pose.pose.pose.position)
        start_msg = StartStatusMsg()
        start_msg.started = True
        start_msg.message = "Position calculated"
        self.status_pub.publish(start_msg)
        rospy.signal_shutdown("StartPosition job done")

    def save_param(self, pose):
        rospy.set_param("pose/x", pose.x)
        rospy.set_param("pose/y", pose.y)
        rospy.set_param("pose/z", pose.z)

if __name__ == "__main__":
    try:
        StartPosition()
    except Exception as e:
        rospy.logerr(e)
