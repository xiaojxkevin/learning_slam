import numpy as np
from scipy.spatial.transform import Rotation as R

#############################################
GT_PATH = "/home/jinxi/codes/learning_slam/pose-graph/scale-aware/data/scale_jump_circle3/GT.txt"
EDGE_PATH = "/home/jinxi/codes/learning_slam/pose-graph/scale-aware/data/scale_jump_circle3/edges.txt"
NODE_PATH = "/home/jinxi/codes/learning_slam/pose-graph/scale-aware/data/scale_jump_circle3/nodes.txt"
#############################################

def gt2tum():
    data = np.genfromtxt(GT_PATH)
    n = data.shape[0]
    time_stamps = data[:, 0].reshape((-1, 1))
    translations = data[:, 1:4]
    quats = data[:, 4:-1]
    scales = data[:, -1].reshape((-1, 1))
    rotations = R.from_quat(quats).as_matrix()
    T = np.asarray([np.eye(4) for _ in range(n)])
    print(T.shape, rotations.shape)
    T[:, :3, :3] = rotations * scales[..., None]
    T[:, :3, 3] = translations
    T = np.linalg.inv(T)
    print(T.shape)
    new_trans = T[:, :3, 3]
    new_quat = R.from_matrix(T[:, :3, :3]).as_quat()
    print(new_trans.shape, new_quat.shape)
    new_data = np.concatenate([time_stamps, new_trans, new_quat], axis=1)
    print(new_data.shape)
    np.savetxt("./gt__tum.txt", new_data, fmt="%.6f")

def edge2tum(is_jump=False):
    data = np.genfromtxt(EDGE_PATH, skip_header=1)
    n = data.shape[0]
    translations = data[:, 2:5]
    quats = data[:, 5:-1]
    scales = data[:, -1].reshape((-1, 1))
    if is_jump:
        scales = np.where(scales == -1, 1, scales)
    rotations = R.from_quat(quats).as_matrix()
    T = np.asarray([np.eye(4) for _ in range(n + 1)])
    print(T.shape, rotations.shape)
    T[1:, :3, :3] = rotations * scales[..., None]
    T[1:, :3, 3] = translations
    for i in range(1, n):
        T[i] = T[i - 1] @ np.linalg.inv(T[i])
    new_trans = T[:, :3, 3]
    new_quat = R.from_matrix(T[:, :3, :3]).as_quat()
    print(new_trans.shape, new_quat.shape)
    new_data = np.concatenate([np.arange(n+1).reshape((-1, 1)), new_trans, new_quat], axis=1)
    print(new_data.shape)
    np.savetxt("./edges__tum.txt", new_data, fmt="%.6f")

def node2tum():
    data = np.genfromtxt(NODE_PATH, skip_header=1)
    n = data.shape[0]
    time_stamps = data[:, 0].reshape((-1, 1))
    translations = data[:, 1:4]
    quats = data[:, 4:-1]
    scales = data[:, -1].reshape((-1, 1))
    rotations = R.from_quat(quats).as_matrix()
    T = np.asarray([np.eye(4) for _ in range(n)])
    print(T.shape, rotations.shape)
    T[:, :3, :3] = rotations * scales[..., None]
    T[:, :3, 3] = translations
    T = np.linalg.inv(T)
    print(T.shape)
    new_trans = T[:, :3, 3]
    new_quat = R.from_matrix(T[:, :3, :3]).as_quat()
    print(new_trans.shape, new_quat.shape)
    new_data = np.concatenate([time_stamps, new_trans, new_quat], axis=1)
    print(new_data.shape)
    np.savetxt("./nodes__tum.txt", new_data, fmt="%.6f")


if __name__ == "__main__":
    # gt2tum()
    edge2tum(True)
    # node2tum()

