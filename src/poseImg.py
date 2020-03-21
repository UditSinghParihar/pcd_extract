#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from squaternion import euler2quat, quat2euler, Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from semantic_mapper.msg import SemLabel
import numpy as np


def callPose(msg):
	px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
	qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
	msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

	q = Quaternion(qw, qx, qy, qz)
	roll, pitch, yaw = quat2euler(*q, degrees=True)

	line = str(px) + " " + str(py) + " " + str(yaw) + "\n"
	fileP.write(line)
	# print(line)


def callImg(msg):
	# global imgCnt
	# imgCnt = imgCnt + 1
	# if(imgCnt%2 != 0):
	# 	return

	bridge = CvBridge()

	try:
		cvImage = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")	
	except CvBridgeError as e:
		print(e)

	global imgId
	dirc = "/home/cair/backup/floor_loop/slam_images6/rgb/"
	imgName = dirc + "rgb{:06d}.jpg".format(imgId)
	imgId = imgId + 1
	cv2.imwrite(imgName, cvImage)

	# cv2.imshow("imageWindow", cvImage)
	# cv2.waitKey(1)


def callDepth(msg):
	bridge = CvBridge()

	try:
		cvImage = bridge.imgmsg_to_cv2(msg, "passthrough")
	except CvBridgeError as e:
		print(e)

	global depthId
	dirc = "/home/cair/backup/floor_loop/slam_images6/depth/"
	imgName = dirc + "depth{:06d}.png".format(depthId)
	depthId = depthId + 1
	cv2.imwrite(imgName, cvImage)

	# cv2.imshow("imageWindow", cvImage)
	# cv2.waitKey(1)


def callPoseImg(poseMsg, imgMsg):
	global freq
	freq = freq + 1
	if(freq%2 != 0):
		return

	callPose(poseMsg)
	callImg(imgMsg)


def callPoseImgDepth(poseMsg, imgMsg, depthMsg):
	global freq
	freq = freq + 1
	if(freq%2 != 0):
		return

	callPose(poseMsg)
	callImg(imgMsg)
	callDepth(depthMsg)


def callPoseLbl(msg, lblMsg):
	px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
	qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
	msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

	q = Quaternion(qw, qx, qy, qz)
	roll, pitch, yaw = quat2euler(*q, degrees=True)

	line = str(px) + " " + str(py) + " " + str(yaw) + " " + str(lblMsg.lvl) + "\n"
	fileP.write(line)
	# print(line)


def main1():
	rospy.Subscriber('pose', Odometry, callPose)


def main2():
	rospy.Subscriber('camera/color/image_raw', Image, callImg)


def main3():
	poseSub = Subscriber('RosAria/pose', Odometry)
	imgSub = Subscriber('camera/color/image_raw', Image)

	ats = ApproximateTimeSynchronizer([poseSub, imgSub], queue_size=1, slop=0.2)
	ats.registerCallback(callPoseImg)


def main4():
	poseSub = Subscriber('RosAria/pose', Odometry)
	imgSub = Subscriber('camera/color/image_raw', Image)
	depthSub = Subscriber('camera/aligned_depth_to_color/image_raw', Image)

	ats = ApproximateTimeSynchronizer([poseSub, imgSub, depthSub], queue_size=1, slop=0.2)
	ats.registerCallback(callPoseImgDepth)


def main5():
	poseSub = Subscriber('raw_odom', Odometry)
	lblSub = Subscriber('semantic_label', SemLabel)

	ats = ApproximateTimeSynchronizer([poseSub, lblSub], queue_size=1, slop=0.2)
	ats.registerCallback(callPoseLbl)


if __name__ == '__main__':
	rospy.init_node("main", anonymous=True)
	fileP = open('/home/cair/backup/floor_loop/poses6.txt', 'w')

	imgId = 0
	imgCnt = 0
	freq = 0
	depthId = 0

	# main1()
	# main2()
	# main3()
	main4()
	# main5()

	rospy.spin()

	fileP.close()
