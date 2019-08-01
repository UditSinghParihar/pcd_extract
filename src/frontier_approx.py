#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf2_ros
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
import math


def occToNumpy(msg):
	data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
	return data


def numpyToOcc(grid, origMsg):
	msg = OccupancyGrid()
	msg.header.frame_id = "map"
	msg.data = grid.ravel()
	msg.info = MapMetaData()
	msg.info.height = grid.shape[0]
	msg.info.width = grid.shape[1]
	msg.info.resolution = 0.05
	msg.info.origin = origMsg.info.origin

	return msg

cnt = 0

def callback(msg):
	print("Resloution: %f, Width: %f, Height: %f" % (msg.info.resolution, msg.info.width, msg.info.height))
	# print(msg.info.origin)

	grid = occToNumpy(msg)

	res = msg.info.resolution
	cellX = int(math.floor((trans.transform.translation.y - msg.info.origin.position.y)/res))
	cellY = int(math.floor((trans.transform.translation.x - msg.info.origin.position.x)/res))
	width = 60
	tempGrid = grid[cellX-width : cellX+width, cellY-width : cellY+width]
	grid = np.full(grid.shape, -1, dtype=int)
	grid[cellX-width : cellX+width,  cellY-width: cellY+width] = tempGrid

	# np.save(params["dir"]+"file"+str(cnt), tempGrid)
	global cnt
	cnt = cnt+1

	# nums = set(grid.ravel())
	# print(nums)

	miniMsg = numpyToOcc(grid, msg)
	mapPub.publish(miniMsg)

	position = Pose(Point(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z), Quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
	mark = Marker(type = Marker.SPHERE, id=0, lifetime=rospy.Duration(10), pose=position, scale=Vector3(0.5, 0.5, 0.5), header=Header(frame_id="map"), color=ColorRGBA(2.0, 0.0, 0.0, 0.8))

	markerPub.publish(mark)


if __name__ == '__main__':
	params = {"dir" : "/home/cair/backup/rapyuta3/frontiers_all/"}

	rospy.init_node("main", anonymous=True)
	
	rospy.Subscriber("/map", OccupancyGrid, callback)
	mapPub = rospy.Publisher("/miniMap", OccupancyGrid, queue_size=1)
	markerPub = rospy.Publisher('/visualizationMarker', Marker, queue_size=5)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass

		rate.sleep()