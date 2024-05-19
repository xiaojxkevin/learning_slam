import numpy as np

def main():
    depth_path = "/home/jinxi/codes/learning_slam/direct_odom/data/depth.txt"
    rgb_path = "/home/jinxi/codes/learning_slam/direct_odom/data/rgb.txt"
    depth_times = np.genfromtxt(depth_path, skip_header=3)[:, 0].astype(np.float64)
    rgb_times = np.genfromtxt(rgb_path, skip_header=3)[:, 0].astype(np.float64)
    l = len(depth_times)
    align_pair = []
    i, j = 0, 0
    while i < l and j < l:
        td = depth_times[i]
        tr = rgb_times[j]
        delta_t = td - tr
        if delta_t < -DT:
            i += 1
            continue
        elif delta_t >= -DT and delta_t < DT:
            align_pair.append([td, tr])
            i += 1
        j += 1

    align_pair = np.asarray(align_pair, dtype=np.float64)
    print(f"There are {align_pair.shape[0]} pairs.")
    np.savetxt("/home/jinxi/codes/learning_slam/direct_odom/pairs.txt", align_pair, fmt="%.6f")

if __name__ == "__main__":
    # [depth, rgb] on each row
    DT = 0.015
    main()