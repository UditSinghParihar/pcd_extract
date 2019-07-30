from sys import argv
import matplotlib.pyplot as plt
import plotly.plotly as py
import plotly.tools as tls

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
		LBL.append(float(lbl.rstrip('\n')))

	return (X, Y, THETA, LBL)

if __name__ == '__main__':
	fileName = str(argv[1])
	(X, Y, THETA, LBL) = read(fileName)

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
	ax.set_title('Dense labeled trajectory')
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
	# ax.axis('scaled')
	plt.xlim(-15, 15)
	plt.ylim(-25, 5)
	plt.gca().set_aspect('equal', adjustable='box')

	X0 = []; Y0 = []; X1 = []; Y1 = []; X2 = []; Y2 =[]; X3 = []; Y3 = [];
	st = end = 0;
	for i in xrange(1, len(LBL)):
		if LBL[i] == LBL[i-1]:
			end = i
			continue

		mid = st + (end - st)/2
		if LBL[mid] == 0:
			X0.append(X[mid])
			Y0.append(Y[mid])

		elif LBL[mid] == 1:
			X1.append(X[mid])
			Y1.append(Y[mid])

		elif LBL[mid] == 2:
			X2.append(X[mid])
			Y2.append(Y[mid])

		elif LBL[mid] == 3:
			X3.append(X[mid])
			Y3.append(Y[mid])

		st = end + 1
		end = st

	ax = plt.subplot(2,1,2)
	ax.plot(X0, Y0, 'ro', label='Rackspace')
	ax.plot(X1, Y1, 'bo', label='Corridor')
	ax.plot(X2, Y2, 'go', label='Trisection')
	ax.plot(X3, Y3, 'yo', label='Intersection')
	ax.plot(X,Y,'k')
	ax.set_title('Meta nodes trajectory')
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
	# ax.axis('scaled')
	plt.xlim(-15, 15)
	plt.ylim(-25, 5)
	plt.gca().set_aspect('equal', adjustable='box')

	# X0 = []; Y0 = []; X1 = []; Y1 = []; X2 = []; Y2 =[]; X3 = []; Y3 = [];
	# st = end = 0;
	# for i in xrange(1, 270):
	# 	if LBL[i] == LBL[i-1]:
	# 		end = i
	# 		continue

	# 	mid = st + (end - st)/2
	# 	if LBL[mid] == 0:
	# 		X0.append(X[mid])
	# 		Y0.append(Y[mid])

	# 	elif LBL[mid] == 1:
	# 		X1.append(X[mid])
	# 		Y1.append(Y[mid])

	# 	elif LBL[mid] == 2:
	# 		X2.append(X[mid])
	# 		Y2.append(Y[mid])

	# 	elif LBL[mid] == 3:
	# 		X3.append(X[mid])
	# 		Y3.append(Y[mid])

	# 	st = end + 1
	# 	end = st

	# ax = plt.subplot(4,1,3)
	# ax.plot(X0, Y0, 'ro', label='Rackspace')
	# ax.plot(X1, Y1, 'bo', label='Corridor')
	# ax.plot(X2, Y2, 'go', label='Trisection')
	# ax.plot(X3, Y3, 'yo', label='Intersection')
	# ax.plot(X[0:270],Y[0:270])
	# ax.set_title('Meta nodes going trajectory')
	# ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
	# ax.axis('scaled')
	
	# X0 = []; Y0 = []; X1 = []; Y1 = []; X2 = []; Y2 =[]; X3 = []; Y3 = [];
	# st = end = 270;
	# for i in xrange(270, len(LBL)):
	# 	if LBL[i] == LBL[i-1]:
	# 		end = i
	# 		continue

	# 	mid = st + (end - st)/2
	# 	if LBL[mid] == 0:
	# 		X0.append(X[mid])
	# 		Y0.append(Y[mid])

	# 	elif LBL[mid] == 1:
	# 		X1.append(X[mid])
	# 		Y1.append(Y[mid])

	# 	elif LBL[mid] == 2:
	# 		X2.append(X[mid])
	# 		Y2.append(Y[mid])

	# 	elif LBL[mid] == 3:
	# 		X3.append(X[mid])
	# 		Y3.append(Y[mid])

	# 	st = end + 1
	# 	end = st

	# ax = plt.subplot(4,1,4)
	# ax.plot(X0, Y0, 'ro', label='Rackspace')
	# ax.plot(X1, Y1, 'bo', label='Corridor')
	# ax.plot(X2, Y2, 'go', label='Trisection')
	# ax.plot(X3, Y3, 'yo', label='Intersection')
	# ax.plot(X[270:-1],Y[270:-1])
	# ax.set_title('Meta nodes coming trajectory')
	# ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
	# ax.axis('scaled')
	
	# plotly_fig = tls.mpl_to_plotly( fig )
	# plotly_fig['layout']['width'] = 1500
	# plotly_fig['layout']['height'] = 900
	# plot_url = py.plot(plotly_fig)

	plt.show()
