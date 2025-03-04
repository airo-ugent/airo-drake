from typing import List, Tuple

import airo_models
import numpy as np
from airo_typing import Vector2DType
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.tree import ModelInstanceIndex, PlanarJoint
from pydrake.planning import RobotDiagramBuilder


def add_mobile_platform(
    robot_diagram_builder: RobotDiagramBuilder,
    drive_positions: Tuple[Vector2DType, Vector2DType, Vector2DType, Vector2DType] = (
        np.array([1, -0.5]),
        np.array([1, 0.5]),
        np.array([-1, -0.5]),
        np.array([-1, 0.5]),
    ),
    battery_position: Vector2DType = np.array([0, 0.5]),
    cpu_position: Vector2DType = np.array([0, -0.5]),
    side_height: float = 0.43,
    side_length: float = 0.69,
    roof_width: float = 0.525,
    roof_thickness: float = 0.03,
    back_panel_offset: float = 0.03,
    brick_size: float = 0.233,
    wheel_from_floor_offset: float = 0.04,
) -> Tuple[ModelInstanceIndex, List[ModelInstanceIndex]]:
    """Add a mobile platform on wheels to the robot diagram builder.

    Currently, this method specifically loads the KELO Robile platform bricks.
    It has default values that correspond to the configuration of the platform at AIRO.
    In the future, this may change.

    Looks up the URDF files for the bricks and welds them together. Also adds the robot's frame, approximated with
    boxes. This method returns the model instance index of a frame, which can be used to weld the robot to the
    world frame, or to add a PlanarJoint to make it possible to move the robot around.

    Args:
        robot_diagram_builder: The robot diagram builder to which the platform will be added.
        drive_positions: A tuple of 2D positions of the drive bricks, relative to the brick size and root frame.
        battery_position: A 2D position of the battery brick, relative to the brick size and root frame.
        cpu_position: A 2D position of the CPU brick, relative to the brick size and root frame.
        side_height: The height of the mobile robot's side.
        side_length: The length (along X) of the mobile robot.
        roof_width: The width (along Y) of the mobile robot.
        roof_thickness: The thickness of the roof.
        back_panel_offset: The offset of the back panel relative to the back of the drive bricks.
        brick_size: The size (width and length) of a KELO brick.
        wheel_from_floor_offset: The height of the bricks from the floor due to the wheels.

    Returns:
        The mobile platform index and the indices of the parts that make up the mobile platform.
    """

    plant = robot_diagram_builder.plant()
    parser = robot_diagram_builder.parser()
    parser.SetAutoRenaming(True)

    mobi_urdf_path = airo_models.get_urdf_path("kelo_robile")

    drive_transforms = [
        RigidTransform(p=[brick_size * p[0], brick_size * p[1], wheel_from_floor_offset], rpy=RollPitchYaw([0, 0, 0]))  # type: ignore
        for p in drive_positions
    ]

    mobi_model_index = parser.AddModels(mobi_urdf_path)[0]
    robot_root_frame = plant.GetFrameByName("base_link", mobi_model_index)

    mobi_part_indices = []

    # robot_transform: relative to world
    # drive_transforms: relative to robot_transform
    for drive_index, drive_transform in enumerate(drive_transforms):
        brick_index = parser.AddModels(airo_models.get_urdf_path("kelo_robile_wheel"))[0]
        brick_frame = plant.GetFrameByName("base_link", brick_index)
        plant.WeldFrames(robot_root_frame, brick_frame, drive_transform)
        mobi_part_indices.append(brick_index)

    battery_transform = RigidTransform(
        p=np.array([brick_size * battery_position[0], brick_size * battery_position[1], wheel_from_floor_offset])
    )
    battery_index = parser.AddModels(airo_models.get_urdf_path("kelo_robile_battery"))[0]
    battery_frame = plant.GetFrameByName("base_link", battery_index)
    plant.WeldFrames(robot_root_frame, battery_frame, battery_transform)
    mobi_part_indices.append(battery_index)

    cpu_transform = RigidTransform(
        p=np.array([brick_size * cpu_position[0], brick_size * cpu_position[1], wheel_from_floor_offset])
    )

    cpu_index = parser.AddModels(airo_models.get_urdf_path("kelo_robile_cpu"))[0]
    cpu_frame = plant.GetFrameByName("base_link", cpu_index)
    plant.WeldFrames(robot_root_frame, cpu_frame, cpu_transform)
    mobi_part_indices.append(cpu_index)

    front_brick_position = np.max([brick_size * p[0] for p in drive_positions]) + brick_size / 2

    side_height_half = 0.5 * side_height
    side_length_half = 0.5 * side_length
    side_transforms = [
        RigidTransform(p=[front_brick_position - side_length_half, -brick_size, side_height_half + wheel_from_floor_offset]),  # type: ignore
        RigidTransform(p=[front_brick_position - side_length_half, brick_size, side_height_half + wheel_from_floor_offset]),  # type: ignore
    ]

    for side_transform in side_transforms:
        side_urdf_path = airo_models.box_urdf_path([side_length, 0.03, side_height], "side")
        side_index = parser.AddModels(side_urdf_path)[0]
        side_frame = plant.GetFrameByName("base_link", side_index)
        plant.WeldFrames(robot_root_frame, side_frame, side_transform)
        mobi_part_indices.append(side_index)

    roof_length = side_length
    roof_length_half = 0.5 * roof_length
    roof_thickness_half = 0.5 * roof_thickness
    roof_transform = RigidTransform(
        p=[front_brick_position - roof_length_half, 0, side_height + roof_thickness_half + wheel_from_floor_offset]
    )  # type: ignore

    roof_urdf_path = airo_models.box_urdf_path([roof_length, roof_width, wheel_from_floor_offset + 0.03], "roof")
    roof_index = parser.AddModels(roof_urdf_path)[0]
    roof_frame = plant.GetFrameByName("base_link", roof_index)
    plant.WeldFrames(robot_root_frame, roof_frame, roof_transform)
    mobi_part_indices.append(roof_index)

    # Thickness of 6cm: panel + buttons, e.g., emergency stop.
    back_urdf_path = airo_models.box_urdf_path(
        [0.06, roof_width, side_height + roof_thickness + wheel_from_floor_offset], "back"
    )
    back_index = parser.AddModels(back_urdf_path)[0]
    back_frame = plant.GetFrameByName("base_link", back_index)
    back_transform = RigidTransform(
        p=[front_brick_position - roof_length - back_panel_offset, 0, side_height_half + wheel_from_floor_offset]
    )  # type: ignore
    plant.WeldFrames(robot_root_frame, back_frame, back_transform)
    mobi_part_indices.append(back_index)

    return mobi_model_index, mobi_part_indices


def attach_mobile_platform_to_world(
    robot_diagram_builder: RobotDiagramBuilder, mobi_model_index: ModelInstanceIndex, static_platform: bool
) -> None:
    """Attach the mobile platform to the world frame with a planar joint. This allows the mobile platform to
    move around by setting the (x, y, theta) values of the joint.

    Args:
        robot_diagram_builder: The robot diagram builder to which the platform will be added.
        mobi_model_index: The mobile platform index.
        static_platform: If true, will weld. If False, will make a joint."""
    plant = robot_diagram_builder.plant()

    mobi_frame = plant.GetFrameByName("base_link", mobi_model_index)

    if not static_platform:
        platform_planar_joint = PlanarJoint("mobi_joint", plant.world_frame(), mobi_frame)
        plant.AddJoint(platform_planar_joint)
    else:
        plant.WeldFrames(plant.world_frame(), mobi_frame)
