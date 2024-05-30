import numpy as np

K = np.array([[320, 0, 320],
             [0, 320, 240],
             [0, 0, 1]], dtype=np.float64)

# With no self-rotation
def main():
    file_path = "/home/jinxi/codes/learning_slam/event_motion/data/data_package1.txt"
    data = np.genfromtxt(file_path, dtype=np.float64, delimiter=",", skip_header=2)
    time_stamps = data[:, -1].reshape((-1, 1))
    pixel_coords = data[:, :2]

    # import matplotlib.pyplot as plt
    # x_coords = pixel_coords[:, 0]
    # y_coords = pixel_coords[:, 1]
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(x_coords, y_coords, time_stamps, c='r', marker='o')
    # ax.set_zlabel('Time')
    # ax.set_xlabel('X Coordinate')
    # ax.set_ylabel('Y Coordinate')
    # plt.show()
    # assert False, f"{x_coords.min(), x_coords.max(), y_coords.min(), y_coords.max()}"

    num_samples = pixel_coords.shape[0]
    # assert False, f"{np.concatenate([pixel_coords, np.ones((num_samples, 1))], axis=1).shape}"
    f_directions = np.concatenate([pixel_coords, np.ones((num_samples, 1))], axis=1) @ np.linalg.inv(K).T
    f_directions /= np.linalg.norm(f_directions, axis=1).reshape((-1, 1))
    # print(f_directions.shape, f_directions[0])
    mat_A = np.concatenate([time_stamps*f_directions, f_directions], axis=1)
    _, S, Vh = np.linalg.svd(mat_A, full_matrices=True, compute_uv="True")
    x_hat = Vh.T[:, -1]
    # Normalize the vector to recover the scale
    x_hat /= np.linalg.norm(x_hat[3:])
    x1, x2 = x_hat[:3], x_hat[3:]
    # print(f"x hat: {x_hat}")
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