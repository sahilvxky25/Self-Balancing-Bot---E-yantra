from scipy.linalg import solve_continuous_are
import numpy as np
A = np.array([[0, 1, 0, 0],
                  [0, 0, -9.81, 0],
                  [0, 0, 0, 1],
                  [0, 0, 24.52, 0]])

B = np.array([[0],
              [1],
              [0],
              [1]])

Q = np.diag([100, 1, 300, 1])
R = np.array([[1]])

# Solve Riccati equation
P = solve_continuous_are(A, B, Q, R)

# LQR gain
K = np.linalg.inv(R) @ B.T @ P
print("LQR Gain K:", K)