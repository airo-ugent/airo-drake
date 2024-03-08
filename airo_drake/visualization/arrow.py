import numpy as np
from airo_typing import RotationMatrixType, Vector3DType
from pydrake.geometry import Cylinder, Meshcat, MeshcatCone, Rgba
from pydrake.math import RigidTransform, RollPitchYaw


def orientation_with_Z_axis(Z: Vector3DType) -> RotationMatrixType:
    Z = np.array(Z)
    Z = Z / np.linalg.norm(Z)  # normalize just to be sure

    X = np.array([1, 0, 0])

    # handle case where direction is parallel to X
    if np.all(np.abs(Z - X) < 1e-6):
        X = np.array([0, 1, 0])

    # make X orthogonal to Z
    X = X - Z * np.dot(X, Z)
    X = X / np.linalg.norm(X)

    # make Y orthogonal to Z and X
    Y = np.cross(Z, X)

    return np.column_stack([X, Y, Z])


def visualize_arrow(
    meshcat: Meshcat,
    name: str,
    start: Vector3DType,
    direction: Vector3DType | None = None,
    end: Vector3DType | None = None,
    length: float = 1.0,
    radius: float = 0.005,
    color: Rgba | None = None,
) -> None:
    """Visualizes an arrow in meshcat.

    Args:
        meshcat: The MeshCat instance to add the visualization to.
        name: A name or path for the arrow in the MeshCat hierarchy, can be used to delete it later.
        start: The starting point of the arrow to visualization.
        direction: The direction of the arrow to visualize.
        end: The end point of the arrow to visualize, mutually exclusive with direction.
        length: Length of the arrow, not used if end is provided.
        radius: Radius of the cylinder.
        color: Color of the arrow.
    """
    start = np.array(start)

    if direction is None and end is None:
        raise ValueError("To visualize an arrow either direction or end must be provided.")

    if direction is not None and end is not None:
        raise ValueError("To visualize an arrow either direction or end must be provided, not both.")

    if direction is None:
        direction = np.array(end) - start
        length = float(np.linalg.norm(direction))

    direction = np.array(direction)

    cone_height = 4 * radius  # rule of thumb
    cone_width = cone_height / 2  # rule of thumb
    cylinder_height = length - cone_height

    orientation = orientation_with_Z_axis(direction)

    pose = np.identity(4)
    pose[:3, :3] = orientation
    pose[:3, 3] = start

    if color is None:
        color_from_direction = np.abs(direction)
        color_from_direction = color_from_direction / np.linalg.norm(color_from_direction)
        color_from_direction = color_from_direction / np.max(color_from_direction)
        color = Rgba(*color_from_direction, 1)  # type: ignore

    arrow_transform = RigidTransform(pose)
    meshcat.SetTransform(f"{name}/arrow/", arrow_transform)

    cylinder_transform = RigidTransform(p=[0, 0, cylinder_height / 2])  # type: ignore
    meshcat.SetObject(f"{name}/arrow/cylinder", Cylinder(radius, cylinder_height), color)
    meshcat.SetTransform(f"{name}/arrow/cylinder", cylinder_transform)

    cone_transform = RigidTransform(rpy=RollPitchYaw(np.deg2rad([180, 0, 0])), p=[0, 0, cylinder_height + cone_height])  # type: ignore
    meshcat.SetTransform(f"{name}/arrow/cone/", cone_transform)
    meshcat.SetObject(f"{name}/arrow/cone", MeshcatCone(cone_height, cone_width, cone_width), color)
