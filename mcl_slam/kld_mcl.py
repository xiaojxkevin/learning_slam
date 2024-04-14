import yaml
import numpy as np
from os.path import join
from math import pi, cos, radians, sin, atan2, sqrt
from scipy.stats import norm
from random import normalvariate, random, randint, seed, choices
from PIL import Image, ImageDraw
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from multiprocessing import Pool
from time import time

CONFIG_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/config.yaml"
BAG_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/data/hw2_data"
MAP_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/data/gridmap.png"
SAVE_RGB_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/results/rgb"
SAVE_GRAY_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/results/gray"
IM_RESOLUTION = 0.05
IM_H, IM_W = 300, 300
MAP_Y_OFFSITE = 40  # (pixel) 2 / 0.05
Z_MAX = 12.0
Z_MIN = 0.15000000596046448
DEGREE_INCR = 0.01745329238474369

##################################################### config
class Config(object):
    """
    a1: rot2rot | a2: rot2trans | a3: trans2trans | a4: trans2rot
    """
    def __init__(self) -> None:
        with open(CONFIG_PATH, "r") as stream:
            data_loader = yaml.safe_load(stream)
        self.particle_num = data_loader["particle_num"]
        self.num_ray = data_loader["ray_num"]
        self.ray_interval = 360 // self.num_ray
        self.occ_threshold = data_loader["occ_threshold"]
        self.min_odom_angle = radians(data_loader["min_odom_angle"])
        self.min_odom_dis = data_loader["min_odom_distance"]

        self.a1 = data_loader["rot2rot_noise"]
        self.a4 = data_loader["trans2rot_noise"]
        self.a3 = data_loader["trans2trans_noise"]
        self.a2 = data_loader["rot2trans_noise"]

        self.z_hit = data_loader["z_hit"]
        self.z_short = data_loader["z_short"]
        self.z_max = data_loader["z_max"]
        self.z_rand = data_loader["z_rand"]
        self.sigma_hit = data_loader["sigma_hit"]
        self.lambda_short = data_loader["lambda_short"]
config = Config()
##################################################### map
def get_map():
    with Image.open(MAP_PATH, "r") as im:
        gridmap = np.asarray(im.getdata(band=None), dtype=np.uint8).reshape(IM_H, IM_W)
        rgb_im = im.convert("RGB")
        return gridmap, rgb_im
GRID_MAP, RGB_IMG = get_map()
#####################################################
class Robot(object):
    z:list = None
    def __init__(self, x:float=None, y:float=None, theta:float=None) -> None:
        if (x and y and theta):
            self.x, self.y, self.theta = x, y, theta
            return
        self.theta = 2 * pi * random() - pi
        while True:
            px, py = randint(50, 250), randint(50, 250) # experimental results
            if GRID_MAP[py, px] > config.occ_threshold:
                self.x, self.y = px*IM_RESOLUTION-7.5, (py-MAP_Y_OFFSITE)*IM_RESOLUTION-7.5
                break

    def set_state(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta
    
    def get_state(self):
        return np.array([self.x, self.y, self.theta], dtype=np.float32)

    def move(self, relative_state:np.ndarray) -> None:
        """book page 136"""
        x, y, theta = relative_state[0], relative_state[1], relative_state[2]
        rot1 = atan2(y, x)
        trans = sqrt(x**2 + y**2)
        rot2 = theta - rot1
        rot1 += normalvariate(0, config.a1 * rot1**2 + config.a2 * trans**2)
        trans += normalvariate(0, config.a3 * trans**2 + config.a4 * (rot1**2 + rot2**2))
        rot2 += normalvariate(0, config.a1 * rot2**2 + config.a2 * trans**2)
        x = trans * cos(rot1)
        y = trans * sin(rot1)
        theta = rot1 + rot2
        dt = np.array([[cos(theta), -sin(theta), x],
                  [sin(theta), cos(theta), y],
                  [0, 0, 1]], dtype=np.float32)
        t = np.array([[cos(self.theta), -sin(self.theta), self.x],
                  [sin(self.theta), cos(self.theta), self.y],
                  [0, 0, 1]], dtype=np.float32)
        new_t = t @ dt
        self.x, self.y, self.theta = new_t[0, 2], new_t[1, 2], self.theta + theta

    def compute_z_hit_normalizer(self, mu:np.ndarray):
        sigma = config.sigma_hit
        z = np.linspace(Z_MIN, Z_MAX, 1000)
        prob = 1 / (np.sqrt(2 * np.pi) * sigma) * np.exp(-0.5 * ((z - mu[:, np.newaxis]) / sigma)**2)
        eta = np.sum(prob, axis=1) * (Z_MAX - Z_MIN) / 999
        return eta

    def measure_prob(self, scan_ranges) -> float:
        """book page 153"""
        px, py = robot2img(self.x, self.y)
        t = v2t(self.get_state())
        ranges = np.asarray([scan_ranges[i] for i in range(360) if i % config.ray_interval == 0], dtype=np.float32)
        ranges = np.where(ranges == np.inf, 12., ranges) # reset all inf to max range of rays, shape [num_rays,]
        angles = np.arange(-pi + DEGREE_INCR, pi, config.ray_interval * DEGREE_INCR)
        # scans are in the robot frame, and scan = [x, y]
        scan = np.stack([-ranges * np.cos(angles) + 0.2, ranges * np.sin(angles), np.ones(angles.shape)])
        scan = t @ scan
        directions = np.arctan2(scan[1, :] - self.y, scan[0, :] - self.x)
        steps = np.zeros((config.num_ray, 1), dtype=np.int32)
        valid = np.full((config.num_ray,), True, dtype=np.bool_)
        for i in range(240): # Z_MAXs / IM_RESOLUTION
            coord_x = np.clip(px + i * np.cos(directions), 0, IM_H - 1).astype(np.int32)
            coord_y = np.clip(py + i * np.sin(directions), 0, IM_H - 1).astype(np.int32)
            reach_occpy = (GRID_MAP[coord_y, coord_x] < config.occ_threshold)
            update_indices = (reach_occpy & valid)
            steps[update_indices] = i
            valid[update_indices] = False
        z_expected = (steps * IM_RESOLUTION).reshape(config.num_ray)
        #################################
        """p_hit shape (K,)"""
        eta_hits = self.compute_z_hit_normalizer(z_expected)
        prob = 1 / (np.sqrt(2 * np.pi) * config.sigma_hit) * \
               np.exp(-0.5 * ((ranges - z_expected) / config.sigma_hit)**2)
        prob[(ranges < Z_MIN) | (ranges > Z_MAX)] = 0
        p_hit = prob / eta_hits
        """p_short shape (K,)"""
        eta_short = np.exp(-config.lambda_short * Z_MIN) - np.exp(-config.lambda_short * z_expected)
        prob = config.lambda_short * np.exp(-config.lambda_short * ranges)
        prob[(ranges < Z_MIN) | (ranges > z_expected)] = 0
        p_short = prob / eta_short
        """p_max shape (K,)"""
        p_max = np.where((ranges < Z_MIN) | (ranges >= Z_MAX), 1., 0.)
        """p_rand shape (K,)"""
        p_rand = np.where((ranges >= Z_MIN) & (ranges < Z_MAX), 1 / (Z_MAX-Z_MIN), 0.)
        p = config.z_hit * p_hit + config.z_short * p_short + config.z_max * p_max + config.z_rand * p_rand
        return float(np.prod(p))
#####################################################
def calculate_kld_upper_bound(k, epsilon:float, delta:float):
    z_1d = norm.pdf(1 - delta)
    Mx = (k - 1) / (2 * epsilon) * (1 - 2 / (9 * (k - 1)) + sqrt(2 / (9 * (k - 1))) * z_1d)**3
    return Mx
#####################################################
def robot2img(x:float, y:float):
    px = int((x+7.5) / IM_RESOLUTION)
    py = int((y+7.5) / IM_RESOLUTION + MAP_Y_OFFSITE)
    px = px if px <= IM_H - 1 else IM_H - 1
    px = px if px >= 0 else 0
    py = py if py <= IM_W - 1 else IM_W - 1
    py = py if py >= 0 else 0
    return px, py

def msg2state(msg):
    pose = msg.pose.pose
    x, y = pose.position.x, pose.position.y
    qz, qw = pose.orientation.z, pose.orientation.w
    theta = 2 * np.arctan2(qz, qw)
    return np.array([x, y, theta], dtype=np.float32)

def v2t(state:np.ndarray):
    theta = state[2]
    t = np.array([[cos(theta), -sin(theta), state[0]],
                  [sin(theta), cos(theta), state[1]],
                  [0, 0, 1]], dtype=np.float32)
    return t

def calculate_relative_state(state:np.ndarray, last_state:np.ndarray):
    theta, last_theta = state[2], last_state[2]
    t = v2t(state)
    last_t = v2t(last_state)
    relative_t = np.linalg.inv(last_t) @ t
    relative_state = np.array([relative_t[0, 2], relative_t[1, 2], theta - last_theta], np.float32)
    return relative_state

def excess_threshold(relative_state:np.ndarray) -> bool:
    d = relative_state[0]**2 + relative_state[1]**2
    return d >= config.min_odom_dis**2 or abs(relative_state[2]) >= config.min_odom_angle

def visualize(save_name:str, scan_ranges=None, particles:list[Robot]=None, state:np.ndarray=None):
    new_img = RGB_IMG.copy()
    draw = ImageDraw.Draw(new_img)
    if state is not None:
        px, py = robot2img(state[0], state[1])
        draw.line((px, py, px + 100 * np.cos(state[2]), py + 100 * np.sin(state[2])), fill=(0,255,0)) # heading
    if scan_ranges is not None:
        t = v2t(state)
        ranges = np.asarray([scan_ranges[i] for i in range(360) if i % config.ray_interval == 0], dtype=np.float32)
        ranges = np.where(ranges == np.inf, 0.0, ranges)
        angles = np.arange(-pi + DEGREE_INCR, pi, config.ray_interval * DEGREE_INCR)
        scan = np.stack([-ranges * np.cos(angles) + 0.2, ranges * np.sin(angles), np.full(angles.shape, 1.)])
        scan = t @ scan
        for i in range(config.num_ray):
            x = (scan[0,i]+7.5)/IM_RESOLUTION
            y = ((scan[1,i]+7.5)/IM_RESOLUTION+MAP_Y_OFFSITE)
            draw.ellipse((x, y, x+1, y+1), fill=(0,0,255))
    if particles is not None:
        for particle in particles:
            px, py = robot2img(particle.x, particle.y)
            draw.ellipse((px-1, py-1, px+1, py+1), fill=(225, 0, 0))
    new_img.save(save_name)

def main():
    ##################################################
    epsilon = 0.1  # 误差容忍度
    delta = 0.1  # 置信度
    ##################################################
    particles = [Robot() for _ in range(config.particle_num)]
    last_triggered_state = np.array([0, 0, 0], dtype=np.float32)
    prev_state = last_triggered_state
    needs_check_scan = False
    weights = [1./config.particle_num] * config.particle_num
    scan_ranges = None
    visualize(join(SAVE_RGB_PATH, "0000.png"), particles=particles)
    start = time()
    with Reader(BAG_PATH) as reader:
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages()):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            if connection.topic == "/scan":
                needs_check_scan = True
                scan_ranges = msg.ranges
                continue
            # Now the topic must be "/odom"
            current_state = msg2state(msg)
            r2 = calculate_relative_state(current_state, last_triggered_state)
            r1 = calculate_relative_state(prev_state, last_triggered_state)
            if needs_check_scan and (excess_threshold(r1) or excess_threshold(r2)):
                m, Mx, k = 0, 0, 0
                H = np.zeros((IM_H, IM_W), dtype=np.bool_)
                new_particles = []
                new_weights = []
                new_state = (current_state + prev_state) / 2.
                relative_state = calculate_relative_state(new_state, last_triggered_state)
                while m < Mx or m < 100:
                    idx = choices(range(len(weights)), weights=weights)[0]
                    particle = Robot(particles[idx].x, particles[idx].y, particles[idx].theta)
                    particle.move(relative_state)
                    w = particle.measure_prob(scan_ranges)
                    new_particles.append(particle)
                    new_weights.append(w)
                    bin_x, bin_y = robot2img(particle.x, particle.y)
                    if not H[bin_y, bin_x]:
                        k += 1
                        H[bin_y, bin_x] = True
                        if k > 1:
                            Mx = calculate_kld_upper_bound(k, epsilon, delta)
                            Mx = 1000 if Mx >= 1000 else Mx
                    m += 1
                    # print(Mx, k)
                particles = new_particles
                weights = new_weights / np.sum(new_weights)
                #######################
                max_index = np.argmax(weights)
                save_name = join(SAVE_RGB_PATH, "{:04d}.png".format(i))
                visualize(save_name, scan_ranges=scan_ranges, particles=particles, 
                          state=particles[max_index].get_state())
                # break
                #######################
                last_triggered_state = new_state
            needs_check_scan = False
            prev_state = current_state
    end = time()
    duration = end - start
    print("Time costs: {:.6f}".format(duration))

if __name__ == "__main__":
    np.random.seed(2021533005)
    seed(2021533005)
    main()