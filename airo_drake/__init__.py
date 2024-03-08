# init files are still required in modern python!
# https://peps.python.org/pep-0420/ introduced implicit namespace packages
# but for building and many toolings, you still need to have __init__ files (at least in the root of the package).
# e.g. if you remove this init file and try to build with pip install .
# you won't be able to import the dummy module.
from airo_drake.building.finish import finish_build
from airo_drake.building.floor import add_floor
from airo_drake.building.manipulator import X_URBASE_ROSBASE, X_URTOOL0_ROBOTIQ, add_manipulator
from airo_drake.building.meshcat import add_meshcat
from airo_drake.path.analysis import (
    calculate_joint_path_distances,
    calculate_joint_path_length,
    calculate_joint_path_outlier_threshold,
    create_paths_from_closest_solutions,
    find_closest_configuration,
    joint_path_has_large_jumps,
)
from airo_drake.scene import DualArmScene, SingleArmScene
from airo_drake.time_parametrization.toppra import time_parametrize_toppra
from airo_drake.trajectory.discretize import discretize_drake_joint_trajectory
from airo_drake.visualization.frame import visualize_frame
from airo_drake.visualization.joints import animate_joint_configurations, animate_joint_trajectory

__all__ = [
    "add_floor",
    "add_manipulator",
    "X_URBASE_ROSBASE",
    "X_URTOOL0_ROBOTIQ",
    "add_meshcat",
    "finish_build",
    "SingleArmScene",
    "DualArmScene",
    "visualize_frame",
    "animate_joint_configurations",
    "animate_joint_trajectory",
    "time_parametrize_toppra",
    "discretize_drake_joint_trajectory",
    "calculate_joint_path_outlier_threshold",
    "calculate_joint_path_length",
    "joint_path_has_large_jumps",
    "find_closest_configuration",
    "create_paths_from_closest_solutions",
    "calculate_joint_path_distances",
]
