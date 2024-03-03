from pydrake.planning import RobotDiagramBuilder

from airo_drake import SingleArmScene, add_floor, add_manipulator, finish_build


def test_single_arm_scene():
    robot_diagram_builder = RobotDiagramBuilder()

    arm_index, gripper_index = add_manipulator(robot_diagram_builder, "ur5e", "robotiq_2f_85")
    add_floor(robot_diagram_builder)

    robot_diagram, context = finish_build(robot_diagram_builder)

    scene = SingleArmScene(robot_diagram, arm_index, gripper_index)

    del robot_diagram_builder

    assert robot_diagram is not None
    assert context is not None
    assert scene is not None
    assert arm_index is not None
    assert gripper_index is not None
