import airo_models
import numpy as np
from airo_typing import HomogeneousMatrixType
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.tree import Frame, ModelInstanceIndex
from pydrake.planning import RobotDiagramBuilder

# X_URBASE_ROSBASE is the 180 rotation between ROS URDF base and the UR control box base
X_URBASE_ROSBASE = RigidTransform(rpy=RollPitchYaw([0, 0, np.pi]), p=[0, 0, 0])  # type: ignore

# 180 degree rotation and 1 cm (estimate) offset for the coupling / flange
X_URTOOL0_ROBOTIQ = RigidTransform(rpy=RollPitchYaw([0, 0, np.pi / 2]), p=[0, 0, 0.01])  # type: ignore


def add_manipulator(
    robot_diagram_builder: RobotDiagramBuilder,
    name: str,
    gripper_name: str,
    arm_transform: HomogeneousMatrixType | None = None,
    gripper_transform: HomogeneousMatrixType | None = None,
    static_arm: bool = False,
    static_gripper: bool = False,
    parent_frame: Frame | None = None,
) -> tuple[ModelInstanceIndex, ModelInstanceIndex]:
    """Add a manipulator (a robot arm with a gripper) to the robot diagram builder.
    Looks up the URDF files for the robot and gripper and welds them together.
    Also provides slightly opionatated default transforms for the welds.
    For example, we rotate the ROS UR URDFs 180 degrees. This enables us to send
    TCP poses in the Drake world frame to the UR control box.

    Args:
        robot_diagram_builder: The robot diagram builder to which the manipulator will be added.
        name: The name of the robot arm, must be known by airo-models
        gripper_name: The name of the gripper, must be known by airo-models
        arm_transform: The transform of the robot arm, if None, we use supply a robot-specific default.
        gripper_transform: The transform of the gripper, if None, we supply a default for the robot-gripper pair.
        static_arm: If True, will fix all arm joints to their default. Useful when you don't want the arm DoFs in the plant.
        static_gripper: If True, will fix all gripper joints to their default. Useful when you don't want the gripper DoFs in the plant.
        parent_frame: The parent frame to weld to. If None, we use the world frame of the plant.

    Returns:
        The robot and gripper index.
    """

    plant = robot_diagram_builder.plant()
    parser = robot_diagram_builder.parser()
    parser.SetAutoRenaming(True)

    # Load URDF files
    arm_urdf_path = airo_models.get_urdf_path(name)
    gripper_urdf_path = airo_models.get_urdf_path(gripper_name)

    if static_arm:
        arm_urdf = airo_models.urdf.read_urdf(arm_urdf_path)
        airo_models.urdf.make_static(arm_urdf)
        arm_urdf_path = airo_models.urdf.write_urdf_to_tempfile(arm_urdf, arm_urdf_path, prefix=f"{name}_static_")

    arm_index = parser.AddModels(arm_urdf_path)[0]

    if static_gripper:
        gripper_urdf = airo_models.urdf.read_urdf(gripper_urdf_path)
        airo_models.urdf.make_static(gripper_urdf)
        gripper_urdf_path = airo_models.urdf.write_urdf_to_tempfile(
            gripper_urdf, gripper_urdf_path, prefix=f"{gripper_name}_static_"
        )

    gripper_index = parser.AddModels(gripper_urdf_path)[0]

    # Weld some frames together
    world_frame = plant.world_frame()
    arm_frame = plant.GetFrameByName("base_link", arm_index)
    arm_tool_frame = plant.GetFrameByName("tool0", arm_index)
    gripper_frame = plant.GetFrameByName("base_link", gripper_index)

    if arm_transform is None:
        if name.startswith("ur"):
            arm_rigid_transform = X_URBASE_ROSBASE
        else:
            arm_rigid_transform = RigidTransform()
    else:
        arm_rigid_transform = RigidTransform(arm_transform)

    if gripper_transform is None:
        if name.startswith("ur") and gripper_name.startswith("robotiq"):
            gripper_rigid_transform = X_URTOOL0_ROBOTIQ
        else:
            gripper_rigid_transform = RigidTransform()
    else:
        gripper_rigid_transform = RigidTransform(gripper_transform)

    if parent_frame is None:
        parent_frame = world_frame
    plant.WeldFrames(parent_frame, arm_frame, arm_rigid_transform)
    plant.WeldFrames(arm_tool_frame, gripper_frame, gripper_rigid_transform)

    return arm_index, gripper_index
