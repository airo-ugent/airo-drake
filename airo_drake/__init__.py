# init files are still required in modern python!
# https://peps.python.org/pep-0420/ introduced implicit namespace packages
# but for building and many toolings, you still need to have __init__ files (at least in the root of the package).
# e.g. if you remove this init file and try to build with pip install .
# you won't be able to import the dummy module.


# We disable isort for the entire __init__.py file, because the order of imports is important here.
# If a function depends on another function, the dependent function should be imported after the function it depends on.
# isort: skip_file
from airo_drake.building.finish import finish_build
from airo_drake.building.floor import add_floor
from airo_drake.building.manipulator import X_URBASE_ROSBASE, X_URTOOL0_ROBOTIQ, add_manipulator
from airo_drake.building.mobile_platform import add_mobile_platform, attach_mobile_platform_to_world
from airo_drake.building.meshcat import add_meshcat
from airo_drake.building.wall import add_wall
from airo_drake.path.analysis import (
    calculate_joint_path_distances,
    calculate_joint_path_length,
    calculate_joint_path_outlier_threshold,
    find_closest_configuration,
    joint_path_has_large_jumps,
)
from airo_drake.path.processing import (
    calculate_joint_paths,
    calculate_path_ik_solutions,
    calculate_valid_joint_paths,
    create_paths_from_closest_solutions,
)
from airo_drake.scene import DualArmScene, SingleArmScene, MobilePlatformWithSingleArmScene
from airo_drake.trajectory.timing import shift_drake_trajectory_in_time
from airo_drake.trajectory.concatenate import concatenate_drake_trajectories
from airo_drake.time_parametrization.toppra import time_parametrize_toppra, time_parametrize_toppra_mobile_platform
from airo_drake.trajectory.discretize import discretize_drake_joint_trajectory, discretize_drake_pose_trajectory
from airo_drake.trajectory.interpolate import joint_trajectory_to_drake
from airo_drake.visualization.frame import visualize_frame
from airo_drake.visualization.joints import (
    animate_dual_joint_trajectory,
    animate_joint_configurations,
    animate_joint_trajectory,
)
from airo_drake.visualization.mobile_platform import animate_mobile_platform_trajectory
from airo_drake.collision.collision_checking import (
    filter_collisions_between_all_body_pairs,
    list_collisions_between_bodies,
)

__all__ = [
    "add_floor",
    "add_wall",
    "add_manipulator",
    "add_mobile_platform",
    "attach_mobile_platform_to_world",
    "X_URBASE_ROSBASE",
    "X_URTOOL0_ROBOTIQ",
    "add_meshcat",
    "finish_build",
    "SingleArmScene",
    "DualArmScene",
    "MobilePlatformWithSingleArmScene",
    "visualize_frame",
    "animate_joint_configurations",
    "animate_joint_trajectory",
    "animate_dual_joint_trajectory",
    "animate_mobile_platform_trajectory",
    "time_parametrize_toppra",
    "time_parametrize_toppra_mobile_platform",
    "discretize_drake_joint_trajectory",
    "discretize_drake_pose_trajectory",
    "joint_trajectory_to_drake",
    "shift_drake_trajectory_in_time",
    "concatenate_drake_trajectories",
    "calculate_joint_path_outlier_threshold",
    "calculate_joint_path_length",
    "joint_path_has_large_jumps",
    "find_closest_configuration",
    "calculate_joint_path_distances",
    "create_paths_from_closest_solutions",
    "calculate_path_ik_solutions",
    "calculate_joint_paths",
    "calculate_valid_joint_paths",
    "filter_collisions_between_all_body_pairs",
    "list_collisions_between_bodies",
]
