#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
# from std_msgs.msg import Header

def callback(msg):
	# print(msg.child_frame_id, msg.header.frame_id)

	msg.child_frame_id = "base_link"
	msg.header.frame_id = "odom"

	odomPub.publish(msg)


if __name__ == '__main__':
	rospy.init_node("odom_rectifier", anonymous=True)
	rospy.Subscriber("/vins_estimator/odometry", Odometry, callback)

	odomPub = rospy.Publisher("/odom_rect", Odometry, queue_size=1)

	rospy.spin()
