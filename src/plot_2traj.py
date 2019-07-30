from sys import argv
import matplotlib.pyplot as plt

def readLabels(fileName):
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
		LBL.append(float(lbl.rstrip('\n')))

	return (X, Y, THETA, LBL)


def readPoses(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for line in A:
		(x, y, theta) = line.split(' ')
		X.append(float(x))
		Y.append(float(y))
		THETA.append(float(theta.rstrip('\n')))

	return (X, Y, THETA)


if __name__ == '__main__':
	fileName = str(argv[1])

	(X, Y, THETA, LBL) = readLabels(fileName)
	
	X0 = []; Y0 = []; X1 = []; Y1 = []; X2 = []; Y2 =[]; X3 = []; Y3 = [];
	
	for i in xrange(len(LBL)):
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

	fig = plt.figure()
	ax = plt.subplot(2,1,1)

	ax.plot(X0, Y0, 'ro', label='Rackspace')
	ax.plot(X1, Y1, 'bo', label='Corridor')
	ax.plot(X2, Y2, 'go', label='Trisection')
	ax.plot(X3, Y3, 'yo', label='Intersection')
	
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	ax.axis('scaled')
	# plt.xlim(-2, 2)
	# plt.ylim(-16, 5)
	# plt.gca().set_aspect('equal', adjustable='box')
	
	fileName = str(argv[2])

	(X, Y, THETA) = readPoses(fileName)

	ax = plt.subplot(2,1,2)
	ax.plot(X, Y)
	ax.axis('scaled')

	plt.show()
