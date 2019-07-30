import matplotlib.pyplot as plt

if __name__ == '__main__':	
	X0 = []; X1 = []; X2 = [];
	LBL = [0,0,0,0,1,1,1,2,2,2,2,2,0,1,1,2,0,1,1]

	st = 0; end = 0; 
	for i in xrange(1, len(LBL)):
		if LBL[i] == LBL[i-1]:
			end = i
			continue

		print(st, end)
		mid = st + (end - st)/2
		if LBL[mid] == 0:
			X0.append(mid)

		elif LBL[mid] == 1:
			X1.append(mid)

		elif LBL[mid] == 2:
			X2.append(mid)

		st = end + 1
		end = st

	print(X0, X1, X2)
