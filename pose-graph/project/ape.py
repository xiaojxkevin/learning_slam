from evo.core import metrics
from evo.core.units import Unit
import pprint
import numpy as np

from evo.tools import plot
import matplotlib.pyplot as plt

from evo.tools import file_interface

def calculate_ape(ref_file:str, est_file:str):
    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)
    pose_relation = metrics.PoseRelation.full_transformation
    data = (traj_ref, traj_est)
    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data(data)
    ape_stats = ape_metric.get_all_statistics()
    pprint.pprint(ape_stats)

if __name__ == "__main__":
    ref = "./pose-graph/project/poses/gt_tum.txt"
    # est = "./pose-graph/project/poses/slam_tum.txt"
    est = "./pose-graph/project/poses/opt_tum.txt"

    calculate_ape(ref, est)