import numpy as np

if __name__ == '__main__':
	a = np.full((4,6), -1, dtype=int)
	print(a)
	b = np.array([[2, 3, 4], [5, 6, 7]])
	print(b, b.shape)
	x = 1; y = 1
	a[x:x+b.shape[0], y:y+b.shape[1]] = b
	print(a)