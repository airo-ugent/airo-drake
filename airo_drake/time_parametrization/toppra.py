import numpy as np
from airo_typing import JointPathType
from pydrake.multibody.optimization import CalcGridPointsOptions, Toppra
from pydrake.multibody.plant import MultibodyPlant
from pydrake.trajectories import PathParameterizedTrajectory, PiecewisePolynomial, Trajectory


def time_parametrize_toppra_mobile_platform(
    plant: MultibodyPlant,
    poses: JointPathType,  # TODO type
    linear_velocity_limit: float = 1.0,
    angular_velocity_limit: float = np.pi / 8,
    linear_acceleration_limit: float = 0.25,
    angular_acceleration_limit: float = 0.025,
) -> PathParameterizedTrajectory:
    """Recalculate the timing of a path or trajectory to respect linear and angular velocity and acceleration limits
    using TOPP-RA.

    Args:
        plant: The MultibodyPlant for the robot.
        poses: A pose path or trajectory.
        linear_velocity_limit: The limit on the linear velocity in m/s.
        angular_velocity_limit: The limit on the angular velocity of the platform in rad/s.
        linear_acceleration_limit: The limit on the linear acceleration in m/s^2.
        angular_acceleration_limit: The limit on the angular acceleration in rad/s^2.

    Returns:
        A time parameterized trajectory."""
    if isinstance(poses, Trajectory):
        pose_trajectory = poses
    else:
        poses = np.array(poses).squeeze()
        times_dummy = np.linspace(0.0, 1.0, len(poses))  # TOPP-RA will calculate the actual times
        pose_trajectory = PiecewisePolynomial.FirstOrderHold(times_dummy, poses.T)

    gridpoints = Toppra.CalcGridPoints(pose_trajectory, CalcGridPointsOptions())

    acceleration_limits_lower = np.array(
        [-linear_acceleration_limit, -linear_acceleration_limit, -angular_acceleration_limit]
    )
    acceleration_limits_upper = -acceleration_limits_lower
    velocity_limits_lower = np.array([-linear_velocity_limit, -linear_velocity_limit, -angular_velocity_limit])
    velocity_limits_upper = -velocity_limits_lower

    toppra = Toppra(pose_trajectory, plant, gridpoints)
    toppra.AddJointAccelerationLimit(acceleration_limits_lower, acceleration_limits_upper)
    toppra.AddJointVelocityLimit(velocity_limits_lower, velocity_limits_upper)
    time_parametrization = toppra.SolvePathParameterization()

    if time_parametrization is None:
        raise ValueError("TOPP-RA failed to find a valid time parametrization.")

    pose_trajectory_ = PathParameterizedTrajectory(pose_trajectory, time_parametrization)

    return pose_trajectory_


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
