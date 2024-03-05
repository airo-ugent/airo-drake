import numpy as np
from airo_typing import JointPathContainer, SingleArmTrajectory
from pydrake.trajectories import Trajectory


def discretize_drake_joint_trajectory(joint_trajectory: Trajectory, steps: int = 100) -> SingleArmTrajectory:
    positions = []
    times_uniform = np.linspace(joint_trajectory.start_time(), joint_trajectory.end_time(), steps)
    for t in times_uniform:
        position = joint_trajectory.value(t).squeeze()
        positions.append(position)
    joint_path = JointPathContainer(positions=np.array(positions))
    joint_trajectory_discretized = SingleArmTrajectory(times_uniform, joint_path)
    return joint_trajectory_discretized
