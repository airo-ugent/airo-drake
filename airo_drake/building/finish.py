from pydrake.geometry import Meshcat
from pydrake.planning import RobotDiagram, RobotDiagramBuilder
from pydrake.systems.framework import Context
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig


def finish_build(
    robot_diagram_builder: RobotDiagramBuilder, meshcat: Meshcat | None = None
) -> tuple[RobotDiagram, Context]:
    """Finish building the robot diagram and create a context. The reason we create and force publish a context it so
    that you can see your robot models visualized in Meshcat. Also if a meshcat is provided, we enable some extra
    visualization such as contacts. We do this here because this can only be done after the plant is finalized.

    Note that after finishing the build, we can no longer add new objects to the Drake scene. However this needs to be
    done to be able to use many functionalities, e.g. collision checking. This is the standard workflow in Drake and a
    known "limitation".

    Args:
        robot_diagram_builder: The RobotDiagramBuilder object to which all models have already been added.
        meshcat: The Meshcat object.

    Returns:
        diagram: The robot diagram.
        context: A default context that you can use as you wish.
    """
    plant = robot_diagram_builder.plant()

    if meshcat is not None:
        builder = robot_diagram_builder.builder()
        plant.Finalize()
        config = VisualizationConfig(publish_contacts=True, enable_alpha_sliders=True)
        ApplyVisualizationConfig(config, builder=builder, plant=plant, meshcat=meshcat)

    robot_diagram: RobotDiagram = robot_diagram_builder.Build()  # type: ignore

    context = robot_diagram.CreateDefaultContext()
    robot_diagram.ForcedPublish(context)

    return robot_diagram, context
