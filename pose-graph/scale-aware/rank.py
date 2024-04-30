import numpy as np
import matplotlib.pyplot as plt

def A(N, B, index_set, V):
    A = np.zeros((3 * B + 3, 3 * N + B))
    # horizontal concatenation
    for line, index_pair in enumerate(index_set):
        i, j = index_pair
        A[3*line:3*(line+1), 3*i:3*(i+1)] = np.eye(3)
        A[3*line:3*(line+1), 3*j:3*(j+1)] = -np.eye(3)
        # print(A[3*line:3*(line+1), 3*N+line].shape, V[line].shape)
        A[3*line:3*(line+1), 3*N+line] = -V[line]
    A[3*B:3*B+3, 0:3] = np.eye(3)
    # np.savetxt("./A.txt", A, fmt="%.3f")
    print(f"The shape of A is {A.shape}")
    return np.linalg.matrix_rank(A)

def jump3():
    N, B = 3, 3
    index_set = [(0, 1), (1, 2), (2, 0)]
    # node: 26, 51, 76
    p0 = np.array([7.296913, 0.000000, 4.845520])
    p1 = np.array([6.766561, 0.000000, 0.178822])
    p2 = np.array([-8.224671, 0.000000, 8.629881])
    V = [p0-p1, p1-p2, p2-p0]
    rank = A(N, B, index_set, V)
    return rank

def jump4(to_plot=False):
    N, B = 4, 4
    index_set = [(0, 1), (1, 2), (2, 3), (3, 0)]
    # node: 26, 51, 76, 101
    p0 = np.array([4.833157, 0.000000, 5.309019])
    p1 = np.array([7.347957, 0.000000, 2.338091])
    p2 = np.array([-2.275300, 0.000000, -7.650070])
    p3 = np.array([-6.282206, 0.000000, -3.430303])
    V = [p0-p1, p1-p2, p2-p3, p3-p0]

    if (to_plot):
        segments = [(p0, p1), (p1, p2), (p2, p3), (p3, p0)]
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for start, end in segments:
            ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], 'o-')
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        plt.show()

    rank = A(N, B, index_set, V)
    return rank

if __name__ == "__main__":
    print(jump3())
    print(jump4(True))