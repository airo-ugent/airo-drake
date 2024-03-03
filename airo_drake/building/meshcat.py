from pydrake.geometry import Meshcat, MeshcatVisualizer, MeshcatVisualizerParams, Role
from pydrake.planning import RobotDiagramBuilder


def add_meshcat(robot_diagram_builder: RobotDiagramBuilder) -> Meshcat:
    scene_graph = robot_diagram_builder.scene_graph()
    builder = robot_diagram_builder.builder()

    # Adding Meshcat must be done before finalizing
    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Add visualizer for proximity/collision geometry
    collision_params = MeshcatVisualizerParams(role=Role.kProximity, prefix="collision", visible_by_default=False)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph.get_query_output_port(), meshcat, collision_params)

    return meshcat
