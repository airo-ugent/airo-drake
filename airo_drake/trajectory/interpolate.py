from airo_typing import SingleArmTrajectory
from pydrake.trajectories import PiecewisePolynomial


def joint_trajectory_to_drake(joint_trajectory: SingleArmTrajectory) -> PiecewisePolynomial:
    """Converts an airo-mono SingleArmTrajectory to a Drake PiecewisePolynomial trajectory, using linear interpolation between configurations.

    Args:
        joint_trajectory: The discretized trajectory to be converted (and thus interpolated).

    Returns:
        A Drake trajectory representing the equivalent joint motion,
            using first-order hold (linear interpolation).

    Raises:
        ValueError: If the input SingleArmTrajectory does not contain position data.
    """
    positions = joint_trajectory.path.positions
    times = joint_trajectory.times
    if positions is None:
        raise ValueError("joint_trajectory has no positions.")

    # FirstOrderHold is linear interpolation
    joint_trajectory_interpolated = PiecewisePolynomial.FirstOrderHold(times, positions.T)

    return joint_trajectory_interpolated
