import numpy as np
from airo_typing import JointPathType
from pydrake.multibody.optimization import CalcGridPointsOptions, Toppra
from pydrake.multibody.plant import MultibodyPlant
from pydrake.trajectories import PathParameterizedTrajectory, PiecewisePolynomial, Trajectory


def time_parametrize_toppra(
    plant: MultibodyPlant,
    joints: JointPathType | Trajectory,
    joint_speed_limit: float = 2.0,  # Max 180 degrees/s ~ 3.14 rad/s
    joint_acceleration_limit: float = 4.0,  # UR recommends < 800 degrees/s^2 ~ 13.9 rad/s^2
) -> PathParameterizedTrajectory:
    """Recalculate the timing of a path or trajectory to respect joint speed and acceleration limits using TOPP-RA.

    Args:
        plant: The MultibodyPlant for the robot.
        joints: A joint path or trajectory.
        joint_speed_limit: The maximum joint speed in rad/s.
        joint_acceleration_limit: The maximum joint acceleration in rad/s^2.
    """
    if isinstance(joints, Trajectory):
        joint_trajectory = joints
        n_dofs = joint_trajectory.value(0).shape[0]
    else:
        joints = np.array(joints).squeeze()
        n_dofs = joints.shape[1]
        times_dummy = np.linspace(0.0, 1.0, len(joints))  # TOPP-RA will calculate the actual times
        joint_trajectory = PiecewisePolynomial.FirstOrderHold(times_dummy, joints.T)

    gridpoints = Toppra.CalcGridPoints(joint_trajectory, CalcGridPointsOptions())

    acceleration_limits_lower = np.array([-joint_acceleration_limit] * n_dofs)
    acceleration_limits_upper = np.array([joint_acceleration_limit] * n_dofs)
    velocity_limits_lower = np.array([-joint_speed_limit] * n_dofs)
    velocity_limits_upper = np.array([joint_speed_limit] * n_dofs)

    toppra = Toppra(joint_trajectory, plant, gridpoints)
    toppra.AddJointAccelerationLimit(acceleration_limits_lower, acceleration_limits_upper)
    toppra.AddJointVelocityLimit(velocity_limits_lower, velocity_limits_upper)
    time_parametrization = toppra.SolvePathParameterization()

    if time_parametrization is None:
        raise ValueError("TOPP-RA failed to find a valid time parametrization.")

    joint_trajectory = PathParameterizedTrajectory(joint_trajectory, time_parametrization)

    return joint_trajectory
