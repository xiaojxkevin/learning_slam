import numpy as np
from scipy.linalg import expm

K = np.array([[320, 0, 320],
             [0, 320, 240],
             [0, 0, 1]], dtype=np.float64)
omega = np.array([-0.242109, 0.0857362, 0.0507106], dtype=np.float64)
MAX_ITERATION = int(1e6)
EPSILON = 1e-3
N = 42

def skew_func(a):
    A = np.array([[0, -a[2], a[1]],
                [a[2], 0, -a[0]],
                [-a[1], a[0], 0]])
    return A
    
def so3ToSO3(xi):
    return expm(skew_func(xi))

def main():
    file_path = "/home/jinxi/codes/learning_slam/event_motion/data/data_package3.txt"
    data = np.genfromtxt(file_path, dtype=np.float64, delimiter=",", skip_header=2)
    pixel_coords = data[:, :2]
    time_stamps = data[:, -1].reshape((-1, 1))
    num_samples = pixel_coords.shape[0]
    f_directions = np.concatenate([pixel_coords, np.ones((num_samples, 1))], axis=1) @ np.linalg.inv(K).T
    f_directions /= np.linalg.norm(f_directions, axis=1).reshape((-1, 1))
    for i in range(num_samples):
        xi = time_stamps[i] * omega
        rotation = so3ToSO3(xi)
        f_directions[i] = rotation @ f_directions[i]
    mat_A = np.concatenate([time_stamps*f_directions, f_directions], axis=1)
    
    valid = None
    flag = True
    for i in range(MAX_ITERATION):
        choices = np.random.choice(num_samples, 5)
        selected_A = mat_A[choices]
        if np.linalg.matrix_rank(selected_A) < 5:
            continue
        _, S, Vh = np.linalg.svd(selected_A, full_matrices=True, compute_uv="True")
        x_hat = Vh.T[:, -1]
        res = np.abs(mat_A @ x_hat)
        valid = (res < EPSILON)
        if np.count_nonzero(valid) >= N:
            flag = False
            print(i)
            break
            
    if np.linalg.matrix_rank(mat_A[valid]) < 5 or flag:
        assert False, "No valid result!"
    _, S, Vh = np.linalg.svd(mat_A[valid], full_matrices=False, compute_uv="True")
    x_hat = Vh.T[:, -1]
    x_hat /= np.linalg.norm(x_hat[3:])
    x1, x2 = x_hat[:3], x_hat[3:]
    print(f"x hat: {x_hat}")
    e2 = x2.copy()
    uz = np.sum(x1 * x2)
    uy = np.linalg.norm(np.cross(x1, x2))
    e1 = np.cross(x1, x2) / uy
    e3 = np.cross(e1, e2)
    
    R_l = np.concatenate([e1, e2, e3]).reshape((3, 3)).T
    u_l = np.array([0, uy, uz]).reshape((3, 1))
    print(f"Rotation matrix for line expressed in inference frame:\n{R_l}")
    print(f"Velocity of the cam in line frame:\n{u_l.T}")


if __name__ == "__main__":
    main()
    # from math import comb
    # print(comb(50, 5))