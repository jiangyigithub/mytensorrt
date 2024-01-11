import numpy as np
K = np.array([[4, 3, 7 ],
              [9, 0, 10],
              [0, 7, 6 ]])
K_1 = np.linalg.inv(K)
print(K_1)

# [[ 70. -31. -30.]
#  [ 54. -24. -23.]
#  [-63.  28.  27.]]

R= np.array([[4, 3, 7 ],
              [9, 0, 10],
              [0, 7, 6 ]])

T=np.array([3,4,5])
RT_augmented = np.eye(4)
#
RT_augmented[:3, :3] = R
RT_augmented[:3, 3] = T
print(RT_augmented)

# RT_augmented = | 1  0  0  0 |
#                | 0  1  0  0 |
#                | 0  0  1  0 |
#                | 0  0  0  1 |


