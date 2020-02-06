#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from squaternion import euler2quat, quat2euler, Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


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
	bridge = CvBridge()

	try:
		cvImage = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")	
	except CvBridgeError as e:
		print(e)

	global imgId
	dirc = "/home/cair/backup/floor_loop/images/"
	imgName = dirc + "frame{:06d}.jpg".format(imgId)
	imgId = imgId + 1
	cv2.imwrite(imgName, cvImage)

	# cv2.imshow("imageWindow", cvImage)
	# cv2.waitKey(1)


def callPoseImg(poseMsg, imgMsg):
	callPose(poseMsg)
	callImg(imgMsg)


def callPoseImgDepth(poseMsg, imgMsg, depthMsg):
	callPose(poseMsg)
	callImg(imgMsg)


def main1():
	rospy.Subscriber('pose', Odometry, callPose)


def main2():
	rospy.Subscriber('camera/color/image_raw', Image, callImg)


def main3():
	poseSub = Subscriber('pose', Odometry)
	imgSub = Subscriber('camera/color/image_raw', Image)

	ats = ApproximateTimeSynchronizer([poseSub, imgSub], queue_size=1, slop=0.2)
	ats.registerCallback(callPoseImg)


def main4():
	poseSub = Subscriber('pose', Odometry)
	imgSub = Subscriber('camera/color/image_raw', Image)
	depthSub = Subscriber('', Image)

	ats = ApproximateTimeSynchronizer([poseSub, imgSub, depthSub], queue_size=1, slop=0.2)
	ats.registerCallback(callPoseImgDepth)


if __name__ == '__main__':
	rospy.init_node("main", anonymous=True)
	fileP = open('corridor_pose.txt', 'w')
	
	imgId = 0
	
	# main1()
	# main2()
	# main3()
	main4()
	
	rospy.spin()

	fileP.close()
