"""Two-stage inverse kinematics for UR arms: analytic branch-pick then calibrated refine.

Analytic IK for UR arms (`ur-analytic-ik`) is fast and, crucially, enumerates the
(up to 8) closed-form solution branches so you can deterministically pick the one
nearest a seed configuration. But its DH model is
the *nominal* (manufacturer) one, so its solution is ~1-2 mm off on a real robot.

The fix is to keep analytic IK as the branch picker and add a second, numerical stage
that refines that solution against the robot's *calibrated* model:

1. **Seed** (`analytic_ik_model.inverse_kinematics_closest`): nominal-DH analytic IK
   picks the branch nearest `q_seed`.
2. **Refine** (`refine_calibrated_ik`): Drake `InverseKinematics` on the calibrated
   plant, seeded at the stage-1 branch with tight position/orientation constraints on
   the tool frame plus a stay-near-seed cost. A local solve from the chosen branch
   snaps to the accurate joints without flipping branches.

`two_stage_calibrated_ik` wires the two stages together.

The plant passed here must be the calibrated model built by `calibrated_dh_to_urdf`
and welded `base_link` -> world with an *identity* transform.
It contains only the arm's 6 joints; nothing to collide
with, which is why this model is IK-only.
"""

from typing import Any

import numpy as np
from airo_typing import HomogeneousMatrixType, JointConfigurationType
from pydrake.math import RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.solvers import IpoptSolver, SolverOptions
from pydrake.systems.framework import Context


def refine_calibrated_ik(
    plant: MultibodyPlant,
    plant_context: Context,
    arm_index: ModelInstanceIndex,
    tcp_pose: HomogeneousMatrixType,
    q_seed: JointConfigurationType,
    tool_frame_name: str = "tool0",
    position_tolerance: float = 1e-5,
    orientation_tolerance: float = 1e-4,
) -> JointConfigurationType | None:
    """Numerically refine a seed configuration against the calibrated model (stage 2).

    Solves Drake `InverseKinematics` on `plant` for the tool frame to reach `tcp_pose`,
    seeded at `q_seed` with a stay-near-seed cost so it stays on the seed's branch.

    Args:
        plant: the calibrated arm plant (see the module docstring).
        plant_context: a context for `plant` (its non-arm state, if any, is held fixed).
        arm_index: the arm's model instance in `plant`.
        tcp_pose: the target tool pose (4x4) in the world/base frame.
        q_seed: the seed joint configuration (6,), e.g. from analytic IK.
        tool_frame_name: the tool frame to constrain (`tool0` == DH frame 6).
        position_tolerance: half-width (m) of the box constraint on the tool position.
        orientation_tolerance: max angle (rad) between target and achieved orientation.

    Returns:
        The refined joint configuration (6,), or None if the solver did not converge.
    """
    q_seed = np.asarray(q_seed, dtype=float).reshape(-1)
    tcp_pose = np.asarray(tcp_pose, dtype=float)
    target_rotation = RotationMatrix(tcp_pose[:3, :3])
    target_position = tcp_pose[:3, 3]

    world_frame = plant.world_frame()
    tool_frame = plant.GetFrameByName(tool_frame_name, arm_index)

    inverse_kinematics = InverseKinematics(plant, plant_context)
    inverse_kinematics.AddPositionConstraint(
        tool_frame,
        [0, 0, 0],
        world_frame,
        target_position - position_tolerance,
        target_position + position_tolerance,
    )
    inverse_kinematics.AddOrientationConstraint(
        world_frame,
        target_rotation,
        tool_frame,
        RotationMatrix(),
        orientation_tolerance,
    )

    program = inverse_kinematics.prog()
    q = inverse_kinematics.q()
    program.SetInitialGuess(q, q_seed)
    program.AddQuadraticErrorCost(
        np.eye(q_seed.size), q_seed, q
    )  # stay on the seed's branch

    solver = IpoptSolver()
    options = SolverOptions()
    options.SetOption(solver.solver_id(), "print_level", 0)
    options.SetOption(solver.solver_id(), "sb", "yes")
    result = solver.Solve(program, None, options)
    if not result.is_success():
        return None
    return plant.GetPositionsFromArray(arm_index, result.GetSolution(q))


def two_stage_calibrated_ik(
    plant: MultibodyPlant,
    plant_context: Context,
    arm_index: ModelInstanceIndex,
    tcp_pose: HomogeneousMatrixType,
    analytic_ik_model: Any,
    q_seed: JointConfigurationType,
    tool_frame_name: str = "tool0",
    position_tolerance: float = 1e-5,
    orientation_tolerance: float = 1e-4,
) -> JointConfigurationType | None:
    """Analytic branch-pick (stage 1) then calibrated refine (stage 2).

    Args:
        plant, plant_context, arm_index: the calibrated arm plant (see module docstring).
        tcp_pose: the target tool pose (4x4) in the world/base frame.
        analytic_ik_model: a `ur-analytic-ik` robot module (e.g. `ur_analytic_ik.ur5e`)
            exposing `inverse_kinematics_closest(tcp_pose, *q_seed)`.
        q_seed: the seed joint configuration (6,) the analytic branch is picked nearest to
            (typically the robot's current configuration).
        tool_frame_name, position_tolerance, orientation_tolerance: forwarded to
            `refine_calibrated_ik`.

    Returns:
        The refined joint configuration (6,), or None if analytic IK found no solution
        for this pose or the refine step did not converge.
    """
    q_seed = np.asarray(q_seed, dtype=float).reshape(-1)
    analytic_solutions = analytic_ik_model.inverse_kinematics_closest(
        np.asarray(tcp_pose, dtype=float), *q_seed
    )
    if not analytic_solutions:
        return None
    branch = np.asarray(analytic_solutions[0], dtype=float).reshape(-1)
    return refine_calibrated_ik(
        plant,
        plant_context,
        arm_index,
        tcp_pose,
        branch,
        tool_frame_name=tool_frame_name,
        position_tolerance=position_tolerance,
        orientation_tolerance=orientation_tolerance,
    )
