import numpy as np
from airo_typing import JointPathContainer, PoseTrajectory, SingleArmTrajectory
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


def discretize_drake_pose_trajectory(pose_trajectory: Trajectory, steps: int = 100) -> PoseTrajectory:
    poses = []
    times_uniform = np.linspace(pose_trajectory.start_time(), pose_trajectory.end_time(), steps)
    for t in times_uniform:
        pose = pose_trajectory.value(t).squeeze()
        poses.append(np.array(pose))

    poses_array = np.array(poses)
    pose_trajectory_discretized = PoseTrajectory(times_uniform, poses_array)
    return pose_trajectory_discretized
