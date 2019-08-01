import numpy as np
import sys
from matplotlib import pyplot as plt
from os import listdir
from os.path import isfile, join


def draw(arr):
	plt.imshow(arr, interpolation='nearest')
	plt.plot(arr.shape[0]/2, arr.shape[1]/2, 'ro', markersize=15)
	plt.xlim(0, arr.shape[0])
	plt.ylim(0, arr.shape[1])
	plt.show(block=False)
	plt.pause(0.3)
	plt.close()


def readFiles(dir):
	onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
	lst = []
	for file in onlyfiles:
		num = ""
		for ele in file:
			if ele.isdigit():
				num += ele
		lst.append(int(num))

	lst.sort()
	files = [dir+'/'+"file"+str(num)+".npy" for num in lst]

	return files


def thresh(file):
	arr = np.load(file)
	
	# draw(arr)
	crop = np.copy(arr)
	arr[np.where(crop>58)] = 100 # 100/0
	arr[np.where((crop<58) & (crop != -1))] = 50 # 50/1
	draw(arr)
	# np.save("/home/cair/Desktop/frontier", arr)

	return arr

if __name__ == '__main__':
	if(len(sys.argv) != 2):
		print("Usage: %s directory" % sys.argv[0])
		sys.exit(1)

	files = readFiles(sys.argv[1])

	for file in files[50:-1]:
		thresh(file)

	# grid = thresh(files[250])

