from dataclasses import dataclass

from pydrake.geometry import Meshcat
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagram


@dataclass
class SingleArmScene:
    """The most important objects when using Drake with a single robot arm."""

    robot_diagram: RobotDiagram
    arm_index: ModelInstanceIndex
    gripper_index: ModelInstanceIndex | None = None
    meshcat: Meshcat | None = None


@dataclass
class DualArmScene:
    """The most important objects when using Drake with two robot arms."""

    robot_diagram: RobotDiagram
    arm_left_index: ModelInstanceIndex
    arm_right_index: ModelInstanceIndex
    gripper_left_index: ModelInstanceIndex | None = None
    gripper_right_index: ModelInstanceIndex | None = None
    meshcat: Meshcat | None = None
