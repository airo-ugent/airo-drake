import airo_models
import numpy as np
from pydrake.math import RigidTransform
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagramBuilder


def add_floor(
    robot_diagram_builder: RobotDiagramBuilder,
    x_size: float = 2.0,
    y_size: float = 2.0,
    thickness: float = 0.2,
) -> ModelInstanceIndex:
    """Add a floor to the robot diagram builder. The floor is a box with the
    given dimensions and thickness. It is welded such that it's top surface is
    at z=0.

    Args:
        robot_diagram_builder: The robot diagram builder to which the floor will be added.
        x_size: The size of the floor in along the global x-axis.
        y_size: The size of the floor in along the global y-axis.
        thickness: The thickness of the floor (could be relevant for collision checking).

    Returns:
        The floor index.
    """
    plant = robot_diagram_builder.plant()
    parser = robot_diagram_builder.parser()
    parser.SetAutoRenaming(True)

    world_frame = plant.world_frame()

    floor_urdf_path = airo_models.box_urdf_path((x_size, y_size, thickness), "floor")
    floor_transform = RigidTransform(p=np.array([0, 0, -thickness / 2]))
    floor_index = parser.AddModels(floor_urdf_path)[0]
    floor_frame = plant.GetFrameByName("base_link", floor_index)

    plant.WeldFrames(world_frame, floor_frame, floor_transform)

    return floor_index
