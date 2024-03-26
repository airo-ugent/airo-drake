import numpy as np
from airo_typing import DualArmTrajectory, JointPathType, SingleArmTrajectory
from loguru import logger
from pydrake.geometry import Meshcat
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagram
from pydrake.systems.framework import Context
from pydrake.trajectories import PiecewisePolynomial, Trajectory

from airo_drake import joint_trajectory_to_drake


def animate_joint_trajectory(
    meshcat: Meshcat,
    robot_diagram: RobotDiagram,
    arm_index: ModelInstanceIndex,
    joint_trajectory: SingleArmTrajectory | Trajectory,
    context: Context | None = None,
) -> None:
    """Publish a recorded animation to meshcat where the robot arm follows the provided joint trajectory.

    Args:
        meshcat: The MeshCat instance to add the visualization to.
        robot_diagram: The robot diagram.
        arm_index: The index of the robot arm to animate.
        joint_trajectory: The joint trajectory to animate.
        context: The context to use for the animation. If None, a new default context will be used (with all other joint angles equal to 0).
    """
    if joint_trajectory is None:
        logger.warning("Asked to animate joint trajectory, but none provided.")

    if isinstance(joint_trajectory, SingleArmTrajectory):
        joint_trajectory = joint_trajectory_to_drake(joint_trajectory)

    assert isinstance(joint_trajectory, Trajectory)  # For mypy, but extra check we have a Drake Trajectory now

    if context is None:
        # Create a new context just for the recording
        context = robot_diagram.CreateDefaultContext()
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    fps = 60.0

    duration = joint_trajectory.end_time()

    # TODO can support trajectories that don't start at 0?
    if not np.isclose(joint_trajectory.start_time(), 0.0):
        logger.warning("Joint trajectory does not start at time 0.0, this is not officially supported.")

    n_frames = int(max(duration * fps, 1))
    frame_times = np.linspace(0.0, duration, n_frames)

    meshcat.StartRecording(set_visualizations_while_recording=False, frames_per_second=fps)

    for t in frame_times:
        context.SetTime(t)
        plant.SetPositions(plant_context, arm_index, joint_trajectory.value(t).squeeze())
        robot_diagram.ForcedPublish(context)

    meshcat.StopRecording()
    meshcat.PublishRecording()


def animate_joint_configurations(
    meshcat: Meshcat,
    robot_diagram: RobotDiagram,
    arm_index: ModelInstanceIndex,
    joint_configurations: list[np.ndarray] | JointPathType,
    duration: float = 4.0,
    time_per_configuration: float | None = None,
    context: Context | None = None,
) -> None:
    """Publish a recorded animation to meshcat where the robot arm cycles through the provided joint configurations.

    Args:
        meshcat: The MeshCat instance to add the visualization to.
        robot_diagram: The robot diagram.
        arm_index: The index of the robot arm to animate.
        joint_cofigurations: A list of joint configurations to animate.
        duration: Total duration of the animation.
        time_per_configuration: The time to spend (still) on each joint solution.  Will overwrite duration if provided.
        context: The context to use for the animation. If None, a new default context will be used (with all other joint angles equal to 0).
    """
    if joint_configurations is None or len(joint_configurations) == 0:
        logger.warning("Asked to animate joint configurations, but none provided.")
        return

    last_configuration = joint_configurations[-1].squeeze()
    joint_configurations = np.array(joint_configurations).squeeze()

    if time_per_configuration is not None:
        duration = time_per_configuration * len(joint_configurations)

    # Due to how ZeroOrderHold works, we need to duplicate the last configuration.
    # See note on how the second point is ignore when two points are provided:
    # https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_piecewise_polynomial.html#a06278ca4588ba1adb004b84fdfc62f3c
    joint_configurations = np.vstack([joint_configurations, last_configuration])

    joint_times = np.linspace(0.0, duration, len(joint_configurations))
    joints_interpolated = PiecewisePolynomial.ZeroOrderHold(joint_times, joint_configurations.T)

    animate_joint_trajectory(meshcat, robot_diagram, arm_index, joints_interpolated, context)


def animate_dual_joint_trajectory(
    meshcat: Meshcat,
    robot_diagram: RobotDiagram,
    arm_index_left: ModelInstanceIndex,
    arm_index_right: ModelInstanceIndex,
    joint_trajectory: DualArmTrajectory | Trajectory,
    degrees_of_freedom_left: int = 6,
    context: Context | None = None,
) -> None:
    """Publish a recorded animation to meshcat where the two robot arms follow the provided joint trajectories.

    Args:
        meshcat: The MeshCat instance to add the visualization to.
        robot_diagram: The robot diagram.
        arm_index_left: The index of the left robot arm to animate.
        arm_index_right: The index of the right robot arm to animate.
        joint_trajectory: The joint trajectory to animate.
        degrees_of_freedom_left: The number of degrees of freedom in the left arm, only used if joint_trajectory is a Drake Trajectory.
        context: The context to use for the animation. If None, a new default context will be used (with all other joint angles equal to 0).
    """
    if joint_trajectory is None:
        logger.warning("Asked to animate joint trajectory, but none provided.")  # can happen if e.g. planning fails

    is_stacked_drake_trajectory = False
    if isinstance(joint_trajectory, Trajectory):
        # Drake Trajectory, with all joint angles stacked
        is_stacked_drake_trajectory = True
        duration = joint_trajectory.end_time()
    else:
        positions_left = joint_trajectory.path_left.positions
        positions_right = joint_trajectory.path_right.positions
        times = joint_trajectory.times

        if positions_left is None or positions_right is None:
            logger.warning(
                "Asked to animate dual joint trajectory, where one of the path's positions is None is not supported yet."
            )
            return

        joint_trajectory_left = PiecewisePolynomial.FirstOrderHold(times, positions_left.T)
        joint_trajectory_right = PiecewisePolynomial.FirstOrderHold(times, positions_right.T)
        duration = joint_trajectory_left.end_time()

    if context is None:
        # Create a new context just for the recording
        context = robot_diagram.CreateDefaultContext()
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    fps = 60.0

    n_frames = int(max(duration * fps, 1))
    frame_times = np.linspace(0.0, duration, n_frames)

    meshcat.StartRecording(set_visualizations_while_recording=False, frames_per_second=fps)

    for t in frame_times:
        context.SetTime(t)
        if is_stacked_drake_trajectory:
            assert isinstance(joint_trajectory, Trajectory)  # For mypy
            plant.SetPositions(
                plant_context, arm_index_left, joint_trajectory.value(t)[:degrees_of_freedom_left].squeeze()
            )
            plant.SetPositions(
                plant_context, arm_index_right, joint_trajectory.value(t)[degrees_of_freedom_left:].squeeze()
            )
        else:
            plant.SetPositions(plant_context, arm_index_left, joint_trajectory_left.value(t).squeeze())
            plant.SetPositions(plant_context, arm_index_right, joint_trajectory_right.value(t).squeeze())
        robot_diagram.ForcedPublish(context)

    meshcat.StopRecording()
    meshcat.PublishRecording()
