import numpy as np

Q = np.array([0,30,60,90])
DP = np.array([0,71,1120,3236])

coeffs = np.polyfit(Q, DP, 2)
print(coeffs)
