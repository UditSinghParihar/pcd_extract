#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import math
from bresenham import bresenham as bre
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
import struct


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

cnt = 0

def write_pgm(image, filename):
	""" Write grayscale image in PGM format to file.
	"""
	height, width = image.shape
	maxval = image.max()
	with open(filename, 'wb') as f:
		f.write('P2 \n{} {} \n{}\n'.format(width, height, maxval))
		# not sure if next line works universally, but seems to work on my mac
		f.write(bytearray(image))


def scanToGrid(points, robot):
	width = 400; res = 0.05; free = 0; occ = 100; unk = -1 # (1,0,-1) == (0, 100, -1)
	grid = np.full((2*width, 2*width), unk, dtype=int)

	pointsRobot = points - robot
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
	mapPub.publish(gridMsg)

	global cnt
	# write_pgm(grid, params["dir"]+"file"+str(cnt)+".pgm")
	# np.save(params["dir"]+"file"+str(cnt), grid)
	cnt = cnt+1


ind = 0
p1 = np.empty((2000, 3)); l1 = -1; r1 = np.zeros((1, 3)) 
p2 = np.empty((2000, 3)); l2 = -1; r2 = np.zeros((1, 3))
p3 = np.empty((2000, 3)); l3 = -1; r3 = np.zeros((1, 3))

def registerCb(msg):
	# print("Recieved callback.")

	robot = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]).reshape(1, 3)

	points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
	(r, c) = points.shape
	print(r, c)
	# print("x: %f  y : %f  z : %f" % (points[0, 0],  points[0, 1], points[0, 2]))
	global ind; global p1; global p2; global p3; global l1; global l2; global l3; global r1; global r2; global r3
	
	off = 10
	if(ind < off):
		p1[0:r, :] = points; l1 = r; r1 = robot
		p2[0:r, :] = points; l2 = r; r2 = robot
		p3[0:r, :] = points; l3 = r; r3 = robot
	if(ind == off):
		p1[0:r, :] = points; l1 = r; r1 = robot
		p2[0:r, :] = points; l2 = r; r2 = robot
	if(ind == (off+1)):
		p1[0:r, :] = points; l1 = r; r1 = robot
	if(ind > (off+1)):
		p3[0:l2, :] = p2[0:l2, :]; l3 = l2; r3 = r2
		p2[0:l1, :] = p1[0:l1, :]; l2 = l1; r2 = r1
		p1[0:r, :] = points; l1 = r; r1 = robot

	comb = np.vstack((p1[0:l1, :], p2[0:l2, :], p3[0:l3, :]))

	ind += 1

	scanToGrid(comb, r3)
	# print(robot[0, 0], robot[0, 1], robot[0, 2])


def callback(msg):
	print("Recieved callback.")

	# for p in pc2.read_points(msg, skip_nans=True):
	# 	print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
	# 	break

	points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
	# print(points.shape)
	# print("x: %f  y : %f  z : %f" % (points[0][0],  points[0][1], points[0][2]))

	robot = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]).reshape(1, 3)
	scanToGrid(points, robot)
	# print(robot[0, 0], robot[0, 1], robot[0, 2])




if __name__ == '__main__':
	params = {"dir" : "/home/cair/backup/rapyuta3/frontiers_pgm/"}
	rospy.init_node("main", anonymous=True)

	# rospy.Subscriber("scan_matched_points2", PointCloud2, callback)
	rospy.Subscriber("scan_matched_points2", PointCloud2, registerCb)
	mapPub = rospy.Publisher("/localMap", OccupancyGrid, queue_size=1)

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