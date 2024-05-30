import numpy as np
from scipy.linalg import expm

K = np.array([[320, 0, 320],
             [0, 320, 240],
             [0, 0, 1]], dtype=np.float64)
omega = np.array([-0.242109, 0.0857362, 0.0507106], dtype=np.float64)
MAX_ITERATION = int(1e5)
EPSILON = 1e-5
N = 30

def skew_func(a):
    A = np.array([[0, -a[2], a[1]],
                [a[2], 0, -a[0]],
                [-a[1], a[0], 0]])
    return A
    
def so3ToSO3(xi):
    return expm(skew_func(xi))

def find_R_and_u(x_hat):
    new_hat = x_hat / np.linalg.norm(x_hat[3:])
    x1, x2 = new_hat[:3], new_hat[3:]
    e2 = x2.copy()
    uz = np.sum(x1 * x2)
    uy = np.linalg.norm(np.cross(x1, x2))
    e1 = np.cross(x1, x2) / uy
    e3 = np.cross(e1, e2)
    R_l = np.concatenate([e1, e2, e3]).reshape((3, 3)).T
    u_l = np.array([0, uy, uz]).reshape((3, 1))
    return R_l, u_l

def ang_repo_err(R_l, u_l, t, f):
    e1, e2, e3 = R_l[:, 0], R_l[:, 1], R_l[:, 2]
    uy, uz = u_l[1], u_l[2]
    p1 = t * (uy*e2 + uz*e3)
    d1 = f
    m1 = np.cross(p1, d1)
    p2 = -e3
    d2 = e1
    m2 = np.cross(p2, d2)

    d = np.cross(d1, d2)
    ortho1 = (-np.cross(m2, np.cross(d2, d)) + float(np.dot(m2, d)) * d1) / np.linalg.norm(d)**2
    numerator = (np.dot(d1, m2) + np.dot(d2, m1)) / np.linalg.norm(d)
    denominator = np.linalg.norm(p1 - ortho1)
    alpha = numerator / denominator
    sigma = alpha if np.dot(p1-p2, d1-d2) >= 0 else -alpha

    return sigma

def vis(data):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # Ensure 3D plotting toolkit is imported
    y_px = data[:50, 0]
    y_py = data[:50, 1]
    t = data[:50, -1]
    n_px = data[50:, 0]
    n_py = data[50:, 1]
    n_t = data[50:, -1]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(y_px, y_py, t, c='b', marker='o', label="Clean")
    ax.scatter(n_px, n_py, n_t, c='r', marker='x', label="Noisy")
    ax.set_zlabel('Time')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    def on_key(event):
        if event.key == ' ':
            plt.close(fig)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()

def main():
    file_path = "/home/jinxi/codes/learning_slam/event_motion/data/data_package3.txt"
    data = np.genfromtxt(file_path, dtype=np.float64, delimiter=",", skip_header=2)
    vis(data)
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
    
    R_l, u_l = None, None
    rots, vels, errors = [], [], []
    for i in range(MAX_ITERATION):
        choices = np.random.choice(num_samples, 5)
        selected_A = mat_A[choices]
        _, S, Vh = np.linalg.svd(selected_A, full_matrices=True, compute_uv="True")
        if S[4] < 1e-9:
            continue 
        x_hat = Vh.T[:, -1]
        res = mat_A @ x_hat
        valid = (np.abs(res) < EPSILON)
        R_l, u_l = find_R_and_u(x_hat)
        if np.count_nonzero(valid) >= N:
            _, S, Vh = np.linalg.svd(mat_A[valid], full_matrices=False, compute_uv="True")
            x_hat = Vh.T[:, -1]
            R_l, u_l = find_R_and_u(x_hat)
            errors.append(np.linalg.norm(mat_A[valid] @ x_hat))
            rots.append(R_l)
            vels.append(u_l)
    
    if len(errors) == 0:
        assert False, f"No valid result!"
    idx = np.argmin(errors)
    R_l, u_l, err = rots[idx], vels[idx], errors[idx]

    print("Error: ", err)
    print(f"Rotation matrix for line expressed in inference frame:\n{R_l}")
    print(f"Velocity of the cam in line frame:\n{u_l.T}")


if __name__ == "__main__":
    main()