import numpy as np
from loguru import logger
from pydrake.geometry import Meshcat
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagram


def animate_ik_solutions(
    meshcat: Meshcat,
    robot_diagram: RobotDiagram,
    arm_index: ModelInstanceIndex,
    joint_solutions: list[np.ndarray],
    time_per_solution: float = 1.0,
) -> None:
    """Publish a recorded animation to meshcat where the robot arm cycles through the provided joint solutions.

    Args:
        meshcat: The MeshCat instance to add the visualization to.
        robot_diagram: The robot diagram.
        joint_solutions: A list of joint configurations to animate.
        time_per_solution: The time to spend (still) on each joint solution.
    """
    if joint_solutions is None or len(joint_solutions) == 0:
        logger.warning("Asked to animate IK solutions, but no solutions provided.")

    # Create a new context just for the recording
    context = robot_diagram.CreateDefaultContext()
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    fps = 60.0
    meshcat.StartRecording(set_visualizations_while_recording=False, frames_per_second=fps)

    t = 0.0
    for joint_configuration in joint_solutions:
        for _ in range(int(time_per_solution * fps)):
            context.SetTime(t)
            plant.SetPositions(plant_context, arm_index, joint_configuration.squeeze())
            robot_diagram.ForcedPublish(context)
            t += 1.0 / fps

    meshcat.StopRecording()
    meshcat.PublishRecording()
