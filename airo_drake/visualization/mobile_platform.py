import numpy as np
from loguru import logger
from pydrake.geometry import Meshcat
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagram
from pydrake.systems.framework import Context
from pydrake.trajectories import Trajectory


def animate_mobile_platform_trajectory(
    meshcat: Meshcat,
    robot_diagram: RobotDiagram,
    mobile_platform_index: ModelInstanceIndex,
    pose_trajectory: Trajectory,
    context: Context | None = None,
) -> None:
    """Publish a recorded animation to meshcat where the mobile platform follows the provided pose trajectory.

    Args:
        meshcat: The MeshCat instance to add the visualization to.
        robot_diagram: The robot diagram.
        mobile_platform_index: The index of the platform to animate.
        pose_trajectory: The joint trajectory to animate.
        context: The context to use for the animation. If None, a new default context will be used.
    """
    if pose_trajectory is None:
        logger.warning("Asked to animate pose trajectory, but none provided.")

    assert isinstance(pose_trajectory, Trajectory)  # For mypy, but extra check we have a Drake Trajectory now

    if context is None:
        # Create a new context just for the recording
        context = robot_diagram.CreateDefaultContext()
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    fps = 60.0

    duration = pose_trajectory.end_time()

    # TODO can support trajectories that don't start at 0?
    if not np.isclose(pose_trajectory.start_time(), 0.0):
        logger.warning("Pose trajectory does not start at time 0.0, this is not officially supported.")

    n_frames = int(max(duration * fps, 1))
    frame_times = np.linspace(0.0, duration, n_frames)

    meshcat.StartRecording(set_visualizations_while_recording=False, frames_per_second=fps)

    for t in frame_times:
        context.SetTime(t)
        plant.SetPositions(plant_context, mobile_platform_index, pose_trajectory.value(t).squeeze())
        robot_diagram.ForcedPublish(context)

    meshcat.StopRecording()
    meshcat.PublishRecording()
