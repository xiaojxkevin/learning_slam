import numpy as np

def generate_poses(num_poses):
    poses = []
    radius = 1.0
    angle_increment = np.radians(30)

    for i in range(num_poses):
        angle = i * angle_increment
        x = radius * np.cos(angle) - 1
        y = radius * np.sin(angle)
        z = 0.0  # Assuming the robot is moving on a flat surface
        # Quaternion representing rotation about z-axis
        qx = 0.0
        qy = 0.0
        qz = np.sin(angle / 2)
        qw = np.cos(angle / 2)
        poses.append([i, x, y, z, qx, qy, qz, qw])

    poses.append([12, 0, 0, 0, 0, 0, 0, 0])
    return poses

def main():
    poses = generate_poses(12)
    poses = np.asarray(poses, dtype=np.float32)
    np.savetxt("./pose-graph/project/poses/gt_tum.txt", poses, fmt="%.6f")

if __name__ == "__main__":
    main()
