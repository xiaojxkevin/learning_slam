import numpy as np

if __name__ == "__main__":
    file_path = "/home/jinxi/codes/learning_slam/event_motion/data/data_package2.txt"
    data = np.genfromtxt(file_path, dtype=np.float64, delimiter=",", skip_header=2)
    px = data[:, 0]
    py = data[:, 1]
    time_stamps = data[:, -1]
    N = 200
    new_px = 320 * np.random.normal(loc=0, scale=1, size=N)
    new_py = 240 * np.random.normal(loc=0, scale=1, size=N)
    new_t = np.random.normal(loc=0, scale=0.08, size=N)
    new_data = np.stack([new_px, new_py, np.zeros(N), new_t], axis=1)
    # print(new_data.shape)

    header = "Camera Angular Velocity: -0.242109 0.0857362 0.0507106\nEvent info: (x y p t)"
    np.savetxt("/home/jinxi/codes/learning_slam/event_motion/data/new_data_3.txt", 
               np.concatenate([data, new_data], axis=0), fmt="%.8f", delimiter=",", header=header)
