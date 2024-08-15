import numpy as np
A = np.loadtxt('./csv1/history_eight_50_r_expert.txt')
B = []
for i in range(A.shape[0]):
    B.append(np.square(A[i][0]) + np.square(A[i][1]) + np.square(A[i][2]))
    if(np.square(A[i][0]) + np.square(A[i][1]) + np.square(A[i][2]) > np.square(0.2)):
        print(i)
print(max(B))