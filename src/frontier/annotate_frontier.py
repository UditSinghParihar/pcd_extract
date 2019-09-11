from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
# import cv2
import glob
import os
import pandas as pd
import scipy.spatial
from matplotlib import pyplot as plt
import sys
import subprocess

if __name__ == '__main__':
	file_occ = sys.argv[1]
	occ_size = 128
	
	occ_map = np.loadtxt(file_occ,delimiter=' ')
	
	occx = []
	occy = []
	unx = []
	uny = []
	frontx = []
	fronty = []
	map_front = np.zeros([occ_size,occ_size])
	for j in range(occ_size):
		for k in range(occ_size):
			if occ_map[j][k] == -1:
				if j-1 >=0 and occ_map[j-1][k] == 1:
					frontx.append(j-1)
					fronty.append(k)
					map_front[frontx[-1]][fronty[-1]] = 1
				if k-1 >=0 and occ_map[j][k-1] == 1:
					frontx.append(j)
					fronty.append(k-1)
					map_front[frontx[-1]][fronty[-1]] = 1
				if j+1 <occ_size and occ_map[j+1][k] == 1:
					frontx.append(j+1)
					fronty.append(k)
					map_front[frontx[-1]][fronty[-1]] = 1
				if k+1 <occ_size and occ_map[j][k+1] == 1:
					frontx.append(j)
					fronty.append(k+1)
					map_front[frontx[-1]][fronty[-1]] = 1
				# if len(frontx)!=0:	
					
				unx.append(j)
				uny.append(k)
			if occ_map[j][k] == 0:
				occx.append(j)
				occy.append(k)
			# if un_map[j][k] == 0:
				


	centx = []
	centy = []
	calcf = np.ones([occ_size,occ_size])
	seenf = np.ones([occ_size,occ_size])
	for j in range(occ_size):
		for k in range(occ_size):
			if seenf[j][k]==1:
				if map_front[j][k] == 1:
					xadd = [j]
					yadd = [k]
					lenvec = len(xadd)
					prevx,prevy = j,k
					calcf[prevx][prevy] = 0
					seenf[j][k] = 0
					ex_list = []
					while True:
						if prevx-1 >=0 and map_front[prevx-1][prevy] == 1 and calcf[prevx-1][prevy] == 1:
							xadd.append(prevx-1)
							yadd.append(prevy)
							calcf[xadd[-1]][yadd[-1]] = 0
							ex_list.append([prevx-1,prevy])
						if prevy-1 >=0 and map_front[prevx][prevy-1] == 1 and calcf[prevx][prevy-1] == 1:
							xadd.append(prevx)
							yadd.append(prevy-1)
							calcf[xadd[-1]][yadd[-1]] = 0
							ex_list.append([prevx,prevy-1])
						if prevx+1 <occ_size and map_front[prevx+1][prevy] == 1 and calcf[prevx+1][prevy] == 1:
							xadd.append(prevx+1)
							yadd.append(prevy)
							calcf[xadd[-1]][yadd[-1]] = 0
							ex_list.append([prevx+1,prevy])
						if prevy+1 <occ_size and map_front[prevx][prevy+1] == 1 and calcf[prevx][prevy+1] == 1:
							xadd.append(prevx)
							yadd.append(prevy+1)
							calcf[xadd[-1]][yadd[-1]] = 0
							ex_list.append([prevx,prevy+1])
						if prevx+1 <occ_size and prevy+1<occ_size and map_front[prevx+1][prevy+1] == 1 and calcf[prevx+1][prevy+1] == 1:
							xadd.append(prevx+1)
							yadd.append(prevy+1)
							calcf[xadd[-1]][yadd[-1]] = 0
							ex_list.append([prevx+1,prevy+1])
						if prevx+1 <occ_size and prevy-1>=0 and map_front[prevx+1][prevy-1] == 1 and calcf[prevx+1][prevy-1] == 1:
							xadd.append(prevx+1)
							yadd.append(prevy-1)
							ex_list.append([prevx+1,prevy-1])
						if prevx-1 >=0 and prevy-1>=0 and map_front[prevx-1][prevy-1] == 1 and calcf[prevx-1][prevy-1] == 1:
							xadd.append(prevx-1)
							yadd.append(prevy-1)
							calcf[xadd[-1]][yadd[-1]] = 0
							ex_list.append([prevx-1,prevy-1])
						if prevy-1 >=0 and prevy+1<occ_size and map_front[prevx-1][prevy+1] == 1 and calcf[prevx-1][prevy+1] == 1:
							xadd.append(prevx-1)
							yadd.append(prevy+1)
							calcf[xadd[-1]][yadd[-1]] = 0
							ex_list.append([prevx-1,prevy+1])

						if len(ex_list)==0:
							break
						else:
							lenvec = len(xadd)
							prevx,prevy = ex_list.pop(-1)
							seenf[prevx][prevy] = 0

					centx.append(sum(xadd)/len(xadd))
					centy.append(sum(yadd)/len(yadd))

	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.set_title('click on points')
	plt.plot(occ_size/2,occ_size/2,'.c')
	plt.plot(occx,occy,'.b')
	plt.plot(unx,uny,'.r')

	plt.axis([0,128,0,128])
	center_line = np.array([[63,0],[63,127]])
	plt.plot(center_line[:,0],center_line[:,1])
	frontiers, = ax.plot(centx,centy,'og',picker=5)

	plt.show()
