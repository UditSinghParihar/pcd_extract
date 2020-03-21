#!/usr/bin/env python

import roslib
import rospy
import tf
from nav_msgs.msg import Odometry


def pubTf(msg):
	br = tf.TransformBroadcaster()
	px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
	qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
	msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
	
	time = msg.header.stamp
	br.sendTransform((px, py, pz), (qx, qy, qz, qw), time, "base_link", "odom")



if __name__ == '__main__':
	rospy.init_node('tfBroadcaster')
	rospy.Subscriber('raw_odom', Odometry, pubTf)
	rospy.spin()