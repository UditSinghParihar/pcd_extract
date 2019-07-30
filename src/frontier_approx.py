#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf2_ros
import numpy as np

def occToNumpy(msg):
	data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
	return data


def numpyToOcc(grid):
	msg = OccupancyGrid()
	msg.header.frame_id = "map"
	msg.data = grid.ravel()
	msg.info = MapMetaData()
	msg.info.height = grid.shape[0]
	msg.info.width = grid.shape[1]
	msg.info.resolution = 0.05

	return msg


def callback(msg):
	print("Resloution: %f, Width: %f, Height: %f" % (msg.info.resolution, msg.info.width, msg.info.height))
	print("x: %f"%trans.transform.translation.x)
	print("y: %f"%trans.transform.translation.y)

	grid = occToNumpy(msg)
	print(grid.size, grid.shape)

	miniMsg = numpyToOcc(grid)
	mapPub.publish(miniMsg)


if __name__ == '__main__':
	rospy.init_node("main", anonymous=True)
	
	rospy.Subscriber("/map", OccupancyGrid, callback)
	mapPub = rospy.Publisher("/miniMap", OccupancyGrid, queue_size=1)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass

		rate.sleep()