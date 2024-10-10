# About the .bag file

## Problem

When I run `ros2 bag info monte_carlo_localization/data/hw2_data.bag`, an error occurs: `[ERROR] [1712283787.892686442] [rosbag2_storage]: No storage id specified, and no plugin found that could open URI`.

## Solution

- First of all, it could not be the reason of I did not install `ros bag`, since I can do some simple things with `ros2 bag`.
- I think it is that the `.bag` file is used in **ROS1**, since for ros2, the bag contains `.yaml` and `.db3`. Therefore, I may need to open this .bag file in **ROS1**.
- install [rosbags](https://pypi.org/project/rosbags/) in python, and use `rosbags-convert` to convert `.bag` file to format in **ROS2**. What I did is `rosbags-convert monte_carlo_localization/data/hw2_data.bag --dst monte_carlo_localization/data/hw2_data `
- And now we only keep the file format in **ROS2**
  
## Bag Info

```bash
Files:             hw2_data.db3
Bag size:          3.8 MiB
Storage id:        sqlite3
Duration:          73.539s
Start:             Mar 18 2019 19:37:33.693 (1552909053.693)
End:               Mar 18 2019 19:38:47.232 (1552909127.232)
Messages:          2062
Topic information: Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 554 | Serialization Format: cdr
                   Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 1508 | Serialization Format: cdr
```