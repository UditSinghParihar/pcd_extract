#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import math
from bresenham import bresenham as bre
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid, MapMetaData
import struct
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA


def numpyToOcc(grid, res):
	msg = OccupancyGrid()
	msg.header.frame_id = "map"
	msg.data = grid.ravel()
	msg.info = MapMetaData()
	msg.info.height = grid.shape[0]
	msg.info.width = grid.shape[1]
	msg.info.resolution = res
	msg.info.origin = Pose(Point(0, 0, 0), Quaternion(0,0,0,1))

	return msg


# def write_pgm(image, filename):
# 	""" Write grayscale image in PGM format to file.
# 	"""
# 	height, width = image.shape
# 	maxval = image.max()
# 	with open(filename, 'wb') as f:
# 		f.write('P2 \n{} {} \n{}\n'.format(width, height, maxval))
# 		# not sure if next line works universally, but seems to work on my mac
# 		f.write(bytearray(image))


# cnt = 0

# def scanToGrid(points, robot):
# 	width = 250; res = 0.08; free = 0; occ = 100; unk = -1 # (1,0,-1) == (0, 100, -1); res = 0.05
# 	grid = np.full((2*width, 2*width), unk, dtype=int)

# 	pointsRobot = points - robot
# 	# print(max(pointsRobot[:,0]), min(pointsRobot[:,0]), max(pointsRobot[:,1]), min(pointsRobot[:,1]))

# 	for p in pointsRobot:
# 		xCell = int(math.floor(p[1]/res) + width)
# 		yCell = int(math.floor(p[0]/res) + width)

# 		ray = bre(width, width, xCell, yCell)
# 		for cell in ray:
# 			grid[cell[0], cell[1]] = free

# 		grid[xCell, yCell] = occ

# 	grid[width, width] = occ
	
# 	gridMsg = numpyToOcc(grid, res)
# 	mapPub1.publish(gridMsg)

# 	global cnt
# 	# write_pgm(grid, params["dir"]+"file"+str(cnt)+".pgm")
# 	# np.save(params["dir"]+"file"+str(cnt), grid)
# 	cnt = cnt+1


# def callback(msg):
# 	# print("Recieved callback.")

# 	# for p in pc2.read_points(msg, skip_nans=True):
# 	# 	print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
# 	# 	break

# 	points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
# 	# print(points.shape)
# 	# print("x: %f  y : %f  z : %f" % (points[0][0],  points[0][1], points[0][2]))

# 	robot = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]).reshape(1, 3)
# 	scanToGrid(points, robot)
# 	# print(robot[0, 0], robot[0, 1], robot[0, 2])


cnt2 = 0

def scanToGrid2(points, robot):
	width = 250; res = 0.08; free = 0; occ = 100; unk = -1 # (1,0,-1) == (0, 100, -1) ; res = 0.05
	grid = np.full((2*width, 2*width), unk, dtype=int)

	pointsRobot = points - robot[0, 0:3]
	# print(max(pointsRobot[:,0]), min(pointsRobot[:,0]), max(pointsRobot[:,1]), min(pointsRobot[:,1]))

	for p in pointsRobot:
		xCell = int(math.floor(p[1]/res) + width)
		yCell = int(math.floor(p[0]/res) + width)

		ray = bre(width, width, xCell, yCell)
		for cell in ray:
			grid[cell[0], cell[1]] = free

		grid[xCell, yCell] = occ

	grid[width, width] = occ
	
	gridMsg = numpyToOcc(grid, res)
	mapPub2.publish(gridMsg)

	# position = Pose(Point(robot[0, 0], robot[0, 1], robot[0, 2]), Quaternion(robot[0, 3], robot[0, 4], robot[0, 5], robot[0, 6]))
	# mark = Marker(type = Marker.SPHERE, id=0, lifetime=rospy.Duration(10), pose=position, scale=Vector3(0.5, 0.5, 0.5), header=Header(frame_id="map"), color=ColorRGBA(2.0, 0.0, 0.0, 0.8))
	# markerPub.publish(mark)
	
	global cnt2
	# write_pgm(grid, params["dir"]+"file"+str(cnt)+".pgm")
	np.save(params["dir"]+"file"+str(cnt2), grid)
	cnt2 = cnt2+1


ind = 0
acc = 8
pcs = np.empty((acc, 2000, 3)); ls = np.arange(acc); rs = np.zeros((acc, 1, 7)) 

def registerCb(msg):
	# print("Recieved callback.")

	robot = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]).reshape(1, 7)
	
	points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
	(r, c) = points.shape
	# print(r, c)
	
	global ind; global pcs; global ls; global rs

	off = 10
	if(ind < off):
		for i in range(acc):
			pcs[i, 0:r, :] = points; ls[i] = r; rs[i] = robot
	
	for i in range(acc-1):
		if(ind == (off+i)):
			for i in range(acc-(i+1)):
				pcs[i, 0:r, :] = points; ls[i] = r; rs[i] = robot

	if(ind > (off+(acc-2))):
		for i in range(acc-1, 0, -1):
			pcs[i, 0:ls[i-1], :] = pcs[i-1, 0:ls[i-1], :]; ls[i] = ls[i-1]; rs[i] = rs[i-1]			
		pcs[0, 0:r, :] = points; ls[0] = r; rs[0] = robot

	comb = np.vstack((pcs[0, 0:ls[0], :], pcs[1, 0:ls[1], :]))
	for i in range(2, acc):
		comb = np.vstack((comb, pcs[i, 0:ls[i], :]))

	ind += 1

	scanToGrid2(comb, rs[acc-1])
	# print(robot[0, 0], robot[0, 1], robot[0, 2])


if __name__ == '__main__':
	params = {"dir" : "/home/cair/backup/rapyuta3/frontiers/"}
	rospy.init_node("main", anonymous=True)

	# rospy.Subscriber("scan_matched_points2", PointCloud2, callback)
	# mapPub1 = rospy.Publisher("/localMap1", OccupancyGrid, queue_size=1)
	
	rospy.Subscriber("scan_matched_points2", PointCloud2, registerCb)
	mapPub2 = rospy.Publisher("/localMap2", OccupancyGrid, queue_size=1)

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

	# rospy.spin()