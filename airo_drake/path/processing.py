import numpy as np
from airo_typing import (
    InverseKinematicsFunctionType,
    JointConfigurationCheckerType,
    JointConfigurationType,
    JointPathType,
    PosePathType,
)
from loguru import logger

from airo_drake import joint_path_has_large_jumps
from airo_drake.path.analysis import find_closest_configuration


def calculate_path_ik_solutions(
    tcp_path: PosePathType, inverse_kinematics_fn: InverseKinematicsFunctionType
) -> list[list[JointConfigurationType]]:
    path_ik_solutions = [inverse_kinematics_fn(tcp_pose) for tcp_pose in tcp_path]
    return path_ik_solutions


def create_paths_from_closest_solutions(
    path_joint_solutions: list[list[JointConfigurationType]],
) -> list[JointPathType]:
    """Constructs paths by iteratively connecting closest joint configurations."""

    paths = []
    start_configurations = path_joint_solutions[0]
    for start_configuration in start_configurations:
        path = [start_configuration]
        for joint_solutions in path_joint_solutions[1:]:
            # logger.info(f"len(joint_solutions): {len(joint_solutions)}")
            if len(joint_solutions) == 0:
                logger.warning("Could not create paths, one of the states has no joint solutions.")
                return []
            closest_config = find_closest_configuration(path[-1], joint_solutions)
            path.append(closest_config)
        paths.append(np.array(path))
    return paths


def calculate_joint_paths(
    tcp_path: PosePathType, inverse_kinematics_fn: InverseKinematicsFunctionType
) -> list[JointPathType]:
    path_ik_solutions = calculate_path_ik_solutions(tcp_path, inverse_kinematics_fn)
    joint_path_candidates = create_paths_from_closest_solutions(path_ik_solutions)
    joint_paths = [path for path in joint_path_candidates if not joint_path_has_large_jumps(path)]
    return joint_paths


def calculate_valid_joint_paths(
    tcp_path: PosePathType,
    inverse_kinematics_fn: InverseKinematicsFunctionType,
    is_state_valid_fn: JointConfigurationCheckerType,
) -> list[JointPathType]:

    joint_paths = calculate_joint_paths(tcp_path, inverse_kinematics_fn)

    # Only keep paths where all joint configurations are valid
    valid_joint_paths = []

    for joint_path in joint_paths:
        if all(is_state_valid_fn(joint_configuration) for joint_configuration in joint_path):
            valid_joint_paths.append(joint_path)

    return valid_joint_paths
