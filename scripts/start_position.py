#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from navigation_bot.msg import StartStatusMsg

class StartPosition:

    def __init__(self):
        rospy.init_node("start_position", anonymous=True)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=2)
        self.status_pub = rospy.Publisher("/start_status", StartStatusMsg, queue_size=2)
        self.get_position()

    def get_position(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        gazebo_pose = rospy.wait_for_message("/odom", Odometry)
        pose_msg.pose.pose = gazebo_pose.pose.pose
        rospy.sleep(1)
        rospy.loginfo("Pose set to: %s" % pose_msg)
        self.pose_pub.publish(pose_msg)
        self.save_param(gazebo_pose.pose.pose.position)
        rospy.loginfo("Pose Published, ready to start the system")

        # Pubblica un messaggio StartStatusMsg indicando che la posizione è stata calcolata
        status_msg = StartStatusMsg()
        status_msg.started = True
        status_msg.message = "Position calculated"
        self.status_pub.publish(status_msg)

        rospy.signal_shutdown("Start position set")

    def save_param(self, pose):
        rospy.set_param("start_pose/x", pose.x)
        rospy.set_param("start_pose/y", pose.y)
        rospy.set_param("start_pose/z", pose.z)

if __name__ == "__main__":
    try:
        StartPosition()
    except Exception as e:
        rospy.logerr(e)
