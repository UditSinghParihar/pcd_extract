#!/usr/bin/env python

import roslib
import rospy
import tf2_ros
import numpy as np
import math
from squaternion import euler2quat, quat2euler, Quaternion
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callImgTf(msg):
	global imgCnt
	imgCnt = imgCnt + 1
	if(imgCnt%5 != 0):
		return

	bridge = CvBridge()

	try:
		cvImage = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")	
	except CvBridgeError as e:
		print(e)

	global imgId
	dirc = "/home/cair/backup/floor_loop/slam_images4/"
	imgName = dirc + "frame{:06d}.jpg".format(imgId)
	imgId = imgId + 1
	cv2.imwrite(imgName, cvImage)

	px, py, pz = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
	qx, qy, qz, qw = trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, \
	trans.transform.rotation.w

	q = Quaternion(qw, qx, qy, qz)
	roll, pitch, yaw = quat2euler(*q, degrees=True)
	line = str(px) + " " + str(py) + " " + str(yaw) + "\n"
	fileP.write(line)


if __name__ == '__main__':
	rospy.init_node("main", anonymous=True)
	fileP = open('poses4.txt', 'w')

	imgId = 0
	imgCnt = 0

	rospy.Subscriber('camera/color/image_raw', Image, callImgTf)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform("odom", "base_link", rospy.Time(0))
			
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass

		rate.sleep()

	fileP.close()
