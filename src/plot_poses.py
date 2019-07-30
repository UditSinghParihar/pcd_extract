from sys import argv
import matplotlib.pyplot as plt

def read(fileName):
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

	(X, Y, THETA) = read(fileName)

	plt.plot(X, Y)
	plt.axis('scaled')
	# plt.xlim(-2, 2)
	# plt.ylim(-16, 4)
	# plt.gca().set_aspect('equal', adjustable='box')
	plt.show()