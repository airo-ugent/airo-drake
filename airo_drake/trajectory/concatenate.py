from pydrake.trajectories import CompositeTrajectory, Trajectory

from airo_drake import shift_drake_trajectory_in_time


def concatenate_drake_trajectories(trajectories: list[Trajectory]) -> CompositeTrajectory:
    """Concatenate multiple trajectories in time. The start time will be the start time of the first trajectory.

    This is achieved by shifting the times of the trajectories to make them continuous.

    Args:
        trajectories: A list of Drake trajectories to be concatenated.

    Returns:
        A CompositeTrajectory representing the concatenated sequence.
    """

    if not trajectories:
        raise ValueError("Cannot concatenate an empty list of trajectories.")

    offset = 0.0
    shifted_trajectories = []
    for trajectory in trajectories:
        shifted_trajectories.append(shift_drake_trajectory_in_time(trajectory, offset))
        offset += trajectory.end_time() - trajectory.start_time()

    return CompositeTrajectory(shifted_trajectories)  # type: ignore
