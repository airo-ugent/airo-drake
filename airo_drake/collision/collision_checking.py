import itertools
from typing import List, Tuple

from airo_typing import JointConfigurationType
from pydrake.all import BodyIndex
from pydrake.planning import RobotCollisionType, SceneGraphCollisionChecker


def filter_collisions_between_all_body_pairs(
    collision_checker: SceneGraphCollisionChecker,
    body_indices_one: List[BodyIndex],
    body_indices_two: List[BodyIndex],
    filtered: bool = True,
) -> None:
    """Enable or disable collision filtering between all pairs of the bodies listed in body_indices_one and body_indices_two.
    When collision filtering is enabled for two bodies, they are allowed to collide.

    Args:
        collision_checker: The collision checker instance to alter.
        body_indices_one: A list of body indices.
        body_indices_two: A list of body indices.
        filtered: Whether to filter collisions between these bodies."""

    body_combinations = itertools.product(body_indices_one, body_indices_two)
    for body_index_1, body_index_2 in body_combinations:
        collision_checker.SetCollisionFilteredBetween(body_index_1, body_index_2, filtered)


def list_collisions_between_bodies(
    collision_checker: SceneGraphCollisionChecker, q: JointConfigurationType
) -> List[Tuple[BodyIndex, BodyIndex, bool]]:
    """List all collisions between bodies for a joint configuration.
    The return type of this function is a list that contains all colliding bodies (if any) and a boolean value that indicates whether the collision is a self-collision.
    When the boolean value is false, the collision is an environment collision.

    Args:
        collision_checker: The collision checker instance to use.
        q: The joint configuration of the robot to check.

    Returns:
        A list of tuples of colliding body indices and a boolean value that is True when the collision is a self collision."""
    robot_clearance = collision_checker.CalcRobotClearance(q, 1.0)
    indices_one = robot_clearance.robot_indices()
    indices_two = robot_clearance.other_indices()
    collision_types = robot_clearance.collision_types()
    distances = robot_clearance.distances()

    return [
        (tup[0], tup[1], tup[2] != RobotCollisionType.kEnvironmentCollision)
        for tup in zip(indices_one, indices_two, collision_types, distances)
        if tup[3] <= 0.0
    ]
