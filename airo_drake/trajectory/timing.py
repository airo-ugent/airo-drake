from pydrake.trajectories import PathParameterizedTrajectory, PiecewisePolynomial, Trajectory


def shift_drake_trajectory_in_time(trajectory: Trajectory, time_shift: float) -> PathParameterizedTrajectory:
    """Shifts a Drake trajectory in time by a specified amount.

    Args:
        trajectory: The Drake trajectory to be shifted.
        time_shift: The amount of time (in seconds) to shift the trajectory forward.

    Returns:
        A new PathParameterizedTrajectory representing the shifted trajectory.
    """
    start = trajectory.start_time()
    end = trajectory.end_time()
    time_shift_trajectory = PiecewisePolynomial.FirstOrderHold([start + time_shift, end + time_shift], [[start, end]])
    shifted_trajectory = PathParameterizedTrajectory(trajectory, time_shift_trajectory)

    return shifted_trajectory
