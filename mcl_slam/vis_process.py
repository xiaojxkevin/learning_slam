from os.path import join
import numpy as np
import yaml
from PIL import Image, ImageDraw
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from pprint import pprint
from multiprocessing import Pool

CONFIG_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/config.yaml"
BAG_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/data/hw2_data"
MAP_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/data/gridmap.png"
SAVE_RGB_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/results/rgb"
SAVE_GRAY_PATH = "/home/jinxi/codes/learning_slam/mcl_slam/results/gray"

##################################################### configs
def read_yaml() -> dict:
    with open(CONFIG_PATH, "r") as stream:
        data_loader = yaml.safe_load(stream)
        return data_loader
config = read_yaml()
IM_RESOLUTION = 0.05
IM_H, IM_W = 300, 300
MAP_Y_OFFSITE = 40  # (pixel) 2 / 0.05
MIN_ODOM_ANGLE = np.deg2rad(config["min_odom_angle"])
MIN_ODOM_DIS = config["min_odom_distance"]
NUM_RAY = config["ray_num"]
RAY_INTERVAL = 360 // NUM_RAY
##################################################### map
def get_map():
    with Image.open(MAP_PATH, "r") as im:
        gridmap = np.asarray(im.getdata(band=None), dtype=np.uint8).reshape(IM_H, IM_W)
        im = im.convert("RGB")
        return gridmap, im
GRID_MAP, RGB_IMG = get_map()
#####################################################

def find_state(msg) -> np.ndarray:
    pose = msg.pose.pose
    x, y = pose.position.x, pose.position.y
    qz, qw = pose.orientation.z, pose.orientation.w
    theta = 2 * np.arctan2(qz, qw)
    return np.array([x, y, theta], dtype=np.float32)

def excess_threshold(state:np.ndarray, last_state:np.ndarray):
    return np.abs(state[2] - last_state[2]) >= MIN_ODOM_ANGLE or \
                ((state[0]-last_state[0])**2 + (state[1]-last_state[1])**2 >= MIN_ODOM_DIS**2)

def save_GRAY_odom(id:int, x:float, y:float, theta:float):
    file_name = join(SAVE_GRAY_PATH, "{:04d}.png".format(id))
    px = int(np.clip(np.ceil(x / IM_RESOLUTION), a_min=0, a_max=IM_H-1))
    py = int(np.clip(np.ceil(y / IM_RESOLUTION + MAP_Y_OFFSITE), a_min=0, a_max=IM_W-1))
    data = GRID_MAP.copy()
    data[px-2:px+2, py-2:py+2] = 0
    for i in range(30):
        data[px+int(i*(np.cos(theta))), py+int(i*(np.sin(theta)))] = 0
    new_img = Image.new("L", (IM_H, IM_W))
    new_img.putdata(data.flatten().tolist())
    new_img.save(file_name)
    

def save_RGB(id:int, state:np.ndarray, ranges:list):
    file_name = join(SAVE_RGB_PATH, "{:04d}.png".format(id))
    x, y, theta = state[0], state[1], state[2]
    ranges = np.asarray([ranges[i] for i in range(360) if i % RAY_INTERVAL == 0], dtype=np.float32)
    ranges = np.where(ranges == np.inf, 0, ranges)
    angles = np.arange(-np.pi, np.pi, 2*np.pi/NUM_RAY)
    scan = np.stack([-ranges*np.cos(angles)+0.2, ranges*np.sin(angles)])
    px = np.clip(np.ceil((x+7.5) / IM_RESOLUTION), a_min=0, a_max=IM_H-1)
    py = np.clip(np.ceil((y+7.5) / IM_RESOLUTION + MAP_Y_OFFSITE), a_min=0, a_max=IM_W-1)
    new_img = RGB_IMG.copy()
    draw = ImageDraw.Draw(new_img)
    # draw.ellipse((py-1, px-1, py+1, px+1), fill=(0, 0, 0))
    for i in range(NUM_RAY):
        draw.line((py, px, (scan[1, i]+7.5)/IM_RESOLUTION+MAP_Y_OFFSITE, (scan[0, i]+7.5)/IM_RESOLUTION), fill=(0,0,255))
    draw.line((py, px, py+np.ceil(100*np.sin(theta)), px+np.ceil(100*np.cos(theta))), fill=(0,255,0))
    new_img.save(file_name)


if __name__ == "__main__":
    # pprint(config)
    last_state = np.array([0., 0., 0.], dtype=np.float32)
    needs_check = False
    scan_ranges = None
    prev_state = None
    with Reader(BAG_PATH) as reader:
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages()):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            if connection.topic == "/scan":
                needs_check = True
                scan_ranges = msg.ranges
                continue
            # Now the topic must be "/odom"
            current_state = find_state(msg)
            if needs_check and (excess_threshold(current_state, last_state) or excess_threshold(prev_state, last_state)):
                new_state = (current_state + prev_state) / 2.
                save_RGB(i, new_state, scan_ranges)
                last_state = current_state
                # break
            needs_check = False
            prev_state = current_state


