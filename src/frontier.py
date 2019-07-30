#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy as np
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber
from msg._SubmapList import SubmapList
from tf2_ros import TFMessage

def callback(occMsg, poseMsg):
	print("Recived messages.")
	print("Resloution: %f, Width: %f, Height: %f, Origin: %f" % occMsg.info.resloution, occMsg.info.width, occMsg.info.height, occMsg.info.origin)


def main():
	rospy.init_node("main", anonymous=True)
	occSub = Subscriber("/map", OccupancyGrid)
	poseSub = Subscriber("/submap_list", SubmapList)
	# tfSub = Subscriber("/tf", TFMessage)

	ats = ApproximateTimeSynchronizer([occSub, poseSub], queue_size=5, slop=0.1)
	ats.registerCallback(callback)
	rospy.spin()

if __name__ == '__main__':
	main()