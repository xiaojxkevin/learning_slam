import numpy as np

A = np.array([[ 0.01385606, -0.95102843, -0.30879272],
            [ 0.90567889, -0.11893541,  0.40693994],
            [-0.42373784, -0.28530563,  0.85967839]], dtype=np.float64)
u = np.array([0, 1.12647, -1.40540])

print(np.linalg.det(A))

vc = np.array([0.163505, 0.36787, 0.296543], dtype=np.float64).reshape((3, 1))
e1, e2, e3 = A[:, 0].reshape((3, 1)), A[:, 1].reshape((3, 1)), A[:, 2].reshape((3, 1))
ux, uy, uz = u[0], u[1], u[2]

print(np.linalg.norm(e1) + np.linalg.norm(e2) + np.linalg.norm(e3))

vc_est = uy * e2 + uz * e3
vc_est /= np.linalg.norm(vc_est)
vc_gt = (np.eye(3) - e1 @ e1.T) @ vc
vc_gt /= np.linalg.norm(vc_gt)

print(vc_est.T, vc_gt.T)
result = np.arccos(np.sum(vc_est * vc_gt))
print(np.sum(vc_est * vc_gt))
print(result)