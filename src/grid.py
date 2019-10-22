#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import math


class Map(object):
	def __init__(self):
		rospy.init_node('Grid')
		
		self.res = 0.25
		self.w = 140
		self.h = 140
		self.thres = 30

		self.map = np.full((self.w, self.h), -1)

		# self.grid = OccupancyGrid()
		# self.grid.header.frame_id ='laser_link'
		# self.grid.info.resolution = self.res
		# self.grid.info.width = self.w
		# self.grid.info.height = self.h
		# self.grid.info.origin.position.x = -self.w/2
		# self.grid.info.origin.position.y = 0
		# self.grid.data = []

		self.pub = rospy.Publisher('/localMap', OccupancyGrid, queue_size=1)
		rospy.Subscriber('/scan', LaserScan, self.laserCb)

	def numpyToOcc(self, grid):
		msg = OccupancyGrid()
		msg.header.frame_id = "base_link"
		msg.data = grid.ravel()
		msg.info = MapMetaData()
		msg.info.height = self.h
		msg.info.width = self.w
		msg.info.resolution = self.res
		msg.info.origin = Pose(Point(0, 0, 0), Quaternion(0,0,0,1))

		return msg

	def laserCb(self, msg):
		print("Angle min: %f and Angle max: %f" % (msg.angle_min, msg.angle_max))
		print("Total scans: %f" % len(msg.ranges))
		print("Range min: %f and Range max: %f" % (msg.range_min, msg.range_max))

		angle = msg.angle_min
		for dt in msg.ranges:
			if (dt > -self.thres and dt < self.thres):
				x = dt*math.cos(angle) + (self.thres/2)
				y = dt*math.sin(angle)

				xCell = math.ceil((1/self.res)*x)
				yCell = math.ceil((1/self.res)*y)
				self.map[int(xCell)][int(yCell)] = 0

				angle += msg.angle_increment

	def mapPub(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			msg = self.numpyToOcc(self.map)
			self.pub.publish(msg) 
			rate.sleep()

if __name__ == '__main__':
	obj = Map()
	obj.mapPub()
	rospy.spin()
