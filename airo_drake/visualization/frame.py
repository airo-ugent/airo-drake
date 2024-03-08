import numpy as np
from airo_typing import HomogeneousMatrixType
from pydrake.geometry import Cylinder, Meshcat, Rgba
from pydrake.math import RigidTransform, RotationMatrix


def visualize_frame(
    meshcat: Meshcat,
    name: str,
    transform: RigidTransform | HomogeneousMatrixType,
    length: float = 0.1,
    radius: float = 0.002,
    opacity: float = 1.0,
) -> None:
    """Visualizes a static frame in meshcat as an RGB (for xyz) colored triad.

    Args:
        transform: The 6D pose / transform of the frame to visualize.
        name: A name or path for the frame in the MeshCat hierarchy, can be used to delete it later.
        meshcat: The MeshCat instance to add the visualization to.
        length: Length of each axis in the triad.
        radius: Radius of each axis cylinder.
        opacity: Opacity for the axis colors.
    """

    meshcat.SetTransform(name, transform)
    axis_labels = ["x-axis", "y-axis", "z-axis"]
    axis_colors = [
        Rgba(1, 0, 0, opacity),
        Rgba(0, 1, 0, opacity),
        Rgba(0, 0, 1, opacity),
    ]
    axis_transforms = [
        RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2), [length / 2.0, 0, 0]),  # type: ignore
        RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2), [0, length / 2.0, 0]),  # type: ignore
        RigidTransform([0, 0, length / 2.0]),  # type: ignore
    ]

    for transform, color, label in zip(axis_transforms, axis_colors, axis_labels):
        meshcat.SetTransform(f"{name}/{label}", transform)
        meshcat.SetObject(f"{name}/{label}", Cylinder(radius, length), color)
