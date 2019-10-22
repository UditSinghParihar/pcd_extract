from sys import argv, exit
import matplotlib.pyplot as plt
import math
import numpy as np
import os


def read(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []
	LBL = []

	for line in A:
		(x, y, theta, lbl) = line.split(' ')
		X.append(float(x))
		Y.append(float(y))
		THETA.append(float(theta))
		LBL.append(int(lbl.rstrip('\n')))

	return (X, Y, THETA, LBL)


def draw(X, Y, LBL):
	X0 = []; Y0 = []; X1 = []; Y1 = []; X2 = []; Y2 =[]; X3 = []; Y3 = [];
	
	for i in range(len(LBL)):
		if LBL[i] == 0:
			X0.append(X[i])
			Y0.append(Y[i])

		elif LBL[i] == 1:
			X1.append(X[i])
			Y1.append(Y[i])

		elif LBL[i] == 2:
			X2.append(X[i])
			Y2.append(Y[i])

		elif LBL[i] == 3:
			X3.append(X[i])
			Y3.append(Y[i])

	plt.plot(X0, Y0, 'ro', label='Rackspace')
	plt.plot(X1, Y1, 'bo', label='Corridor')
	plt.plot(X2, Y2, 'go', label='Trisection')
	plt.plot(X3, Y3, 'yo', label='Intersection')
	
	plt.xlim(-50, 20)
	plt.ylim(-35, 10)
	plt.show(block=False)
	plt.pause(0.001)

	plt.clf()


if __name__ == '__main__':
	fileName = "/home/cair/Desktop/tf_label_draw.txt"

	i = 0
	while(True):
		if(i%5 == 0):
			cmd = "cp /home/cair/Desktop/tf_label.txt /home/cair/Desktop/tf_label_draw.txt"
			os.system(cmd)

			(X, Y, THETA, LBL) = read(fileName)
			if(len(X) > 470 and len(X) < 750):
				X = X[470:-1]; Y = Y[470:-1]; THETA = THETA[470:-1]; LBL = LBL[470:-1]
			draw(X, Y, LBL)

		i += 1
