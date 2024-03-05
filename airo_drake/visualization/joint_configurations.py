import numpy as np
from airo_typing import JointPathType
from loguru import logger
from pydrake.geometry import Meshcat
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagram
from pydrake.systems.framework import Context
from pydrake.trajectories import PiecewisePolynomial


def animate_joint_configurations(
    meshcat: Meshcat,
    robot_diagram: RobotDiagram,
    arm_index: ModelInstanceIndex,
    joint_configurations: list[np.ndarray] | JointPathType,
    duration: float = 4.0,
    time_per_configuration: float = None,
    context: Context | None = None,
) -> None:
    """Publish a recorded animation to meshcat where the robot arm cycles through the provided joint configurations.

    Args:
        meshcat: The MeshCat instance to add the visualization to.
        robot_diagram: The robot diagram.
        joint_cofigurations: A list of joint configurations to animate.
        duration: Total duration of the animation.
        time_per_configuration: The time to spend (still) on each joint solution.  Will overwrite duration if provided.
        context: The context to use for the animation. If None, a new default context will be used (with all other joint angles equal to 0).
    """
    if joint_configurations is None or len(joint_configurations) == 0:
        logger.warning("Asked to animate joint configurations, but none provided.")

    if context is None:
        # Create a new context just for the recording
        context = robot_diagram.CreateDefaultContext()
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    joint_configurations = np.array(joint_configurations).squeeze()

    if time_per_configuration is not None:
        duration = time_per_configuration * len(joint_configurations)

    joint_times = np.linspace(0.0, duration, len(joint_configurations))
    joints_interpolated = PiecewisePolynomial.ZeroOrderHold(joint_times, joint_configurations.T)

    fps = 60.0
    n_frames = int(max(duration * fps, 1))
    frame_times = np.linspace(0.0, duration, n_frames)

    meshcat.StartRecording(set_visualizations_while_recording=False, frames_per_second=fps)

    for t in frame_times:
        context.SetTime(t)
        plant.SetPositions(plant_context, arm_index, joints_interpolated.value(t).squeeze())
        robot_diagram.ForcedPublish(context)

    meshcat.StopRecording()
    meshcat.PublishRecording()
