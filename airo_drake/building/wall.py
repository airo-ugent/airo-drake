import airo_models
import numpy as np
from airo_typing import Vector3DType
from pydrake.math import RigidTransform
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagramBuilder


def add_wall(
    robot_diagram_builder: RobotDiagramBuilder,
    type: str = "XZ",
    height: float = 2.0,
    width: float = 2.5,
    thickness: float = 0.2,
    position: Vector3DType = np.zeros(3),
    name: str = "wall",
) -> ModelInstanceIndex:
    """Adds a wall to the robot diagram builder.

    The wall is represented as a box with the specified dimensions, type, and position.
    The wall is automatically welded to the world frame for a static setup.

    Args:
        robot_diagram_builder: The robot diagram builder to which the wall will be added.
        type: The orientation of the wall.  Must be 'XZ' (parallel to the X-Z plane)
              or 'YZ' (parallel to the Y-Z plane).
        height: The height of the wall.
        width: The width of the wall (relevant for its orientation).
        thickness: The thickness of the wall (could be relevant for collision checking).
        position: The 3D position (x, y, z) of the center of the wall.
        name: The name of the wall (for easy reference).

    Returns:
        The ModelInstanceIndex of the added wall.
    """
    plant = robot_diagram_builder.plant()
    parser = robot_diagram_builder.parser()
    parser.SetAutoRenaming(True)

    z_size = height

    if type == "XZ":
        x_size = width
        y_size = thickness
    elif type == "YZ":
        x_size = thickness
        y_size = width
    else:
        raise ValueError("Invalid wall type, must be 'XZ' or 'YZ'")

    wall_urdf_path = airo_models.box_urdf_path((x_size, y_size, z_size), name)
    wall_index = parser.AddModels(wall_urdf_path)[0]

    wall_transform = RigidTransform(p=position)
    world_frame = plant.world_frame()
    wall_frame = plant.GetFrameByName("base_link", wall_index)

    plant.WeldFrames(world_frame, wall_frame, wall_transform)

    return wall_index
