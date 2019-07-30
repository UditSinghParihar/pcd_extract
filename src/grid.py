#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class Map(object):
	def __init__(self):
		rospy.init_node('Grid')
		
		self.res = 0.25
		self.w = 120
		self.h = 120
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

		self.pub = rospy.Publisher('/localMap', LaserScan, queue_size=1)
		rospy.Subscriber('/scan', LaserScan, self.laserCb)

	def laserCb(self, msg):
		print("Angle min: %f and Angle max: %f" % (msg.angle_min, msg.angle_max))
		print("Total scans: %f" % len(msg.ranges))
		print("Range min: %f and Range max: %f" % (msg.range_min, msg.range_max))

		angle = msg.angle_min
		for dt in msg.ranges:
			if (dt > -thres and dt < thres):
				x = dt*math.cos(angle) + (thres/2)
				y = dt*math.sin(angle)

				xCell = math.ceil((1/res)*x)
				yCell = math.ceil((1/res)*y)
				self.map[xCell][yCell] = 0

				angle += msg.angle_increment

	# def mapPub(self):
	# 	rate = rospy.Rate(10)
	# 	while not rospy.is_shutdown():
	# 		self.pub.publish(self.grid) 
	# 		rate.sleep()

if __name__ == '__main__':
	obj = Map()
	# obj.mapPub()
	rospy.spin()
