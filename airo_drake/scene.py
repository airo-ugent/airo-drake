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
    arm_index_left: ModelInstanceIndex
    arm_index_right: ModelInstanceIndex
    gripper_index_left: ModelInstanceIndex | None = None
    gripper_index_right: ModelInstanceIndex | None = None
    meshcat: Meshcat | None = None
