"""Forward and inverse kinematics for a single-arm URDF, via Drake.

`inverse_kinematics_closest` is a local numerical solve, so it needs a seed near the
desired solution (see `q_seed`). `ur_calibration.CalibratedKinematics` extends this for
calibrated UR arms.

Only single-arm URDFs are supported.
"""

from dataclasses import dataclass

import numpy as np
from airo_typing import HomogeneousMatrixType, JointConfigurationType
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics as DrakeInverseKinematics
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.planning import RobotDiagramBuilder
from pydrake.solvers import IpoptSolver, SolverOptions
from pydrake.systems.framework import Context

from airo_drake.building.finish import finish_build
from airo_drake.scene import SingleArmScene

# A successful refine only nudges the seed by the calibration magnitude (sub-degree in
# practice). A larger move means the soft stay-near-seed cost lost the branch (a flip, or a
# poor seed). The default (~5 deg) sits well above a real nudge and below a branch flip.
_SEED_DEVIATION_THRESHOLD = float(np.deg2rad(5.0))

# Name of the fixed frame added at `tool0` when a `gripper_transform` is given.
GRIPPER_TCP_FRAME_NAME = "gripper_tcp"


@dataclass(frozen=True)
class KinematicsResult:
    """Result of a seeded numerical IK solve.

    The stay-near-seed cost is soft, so the solve can settle on a different branch than the
    seed; `is_close_to_seed` flags exactly that.

    Attributes:
        joint_configuration: the solved joint configuration.
        is_close_to_seed: whether every joint stayed within `seed_deviation_threshold` of the seed.
        max_seed_deviation: the largest per-joint move from the seed (radians, wrapped to [-pi, pi)).
    """

    joint_configuration: JointConfigurationType
    is_close_to_seed: bool
    max_seed_deviation: float


def _build_single_arm_scene(
    urdf: str,
    from_string: bool,
    base_link_name: str,
    base_transform: RigidTransform,
    gripper_transform: RigidTransform | None = None,
) -> tuple[SingleArmScene, Context]:
    """Build a single-arm, geometry-agnostic plant welded to the world, as a `SingleArmScene`.

    If `gripper_transform` is given, adds a `GRIPPER_TCP_FRAME_NAME` frame fixed to `tool0` at
    that offset, so `tool_frame_name=GRIPPER_TCP_FRAME_NAME` can be used for FK/IK on the gripper's
    TCP instead of the arm's flange.

    Returns the scene and a plant context. Keep both alive together: the plant and its
    context are owned by the scene's `robot_diagram`.
    """
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    parser = builder.parser()
    arm_index = (parser.AddModelsFromString(urdf, "urdf") if from_string else parser.AddModels(urdf))[0]  # type: ignore
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(base_link_name, arm_index), base_transform)
    if gripper_transform is not None:
        tool_frame = plant.GetFrameByName("tool0", arm_index)
        plant.AddFrame(FixedOffsetFrame(GRIPPER_TCP_FRAME_NAME, tool_frame, gripper_transform))
    robot_diagram, context = finish_build(builder)
    plant_context = plant.GetMyContextFromRoot(context)
    return SingleArmScene(robot_diagram=robot_diagram, arm_index=arm_index), plant_context


class Kinematics:
    """Forward and inverse kinematics for a single arm parsed from a URDF.

        kinematics = Kinematics.from_urdf_path(airo_models.get_urdf_path("ur5e"), base_transform=X_URBASE_ROSBASE)
        result = kinematics.inverse_kinematics_closest(tcp_pose, q_seed)

    `base_link_name`/`base_transform` set how the arm is welded to the world. Keep an instance
    alive while using it or its results: it owns the plant its computations run on.

    Pass `gripper_transform` to do FK/IK on a gripper's TCP instead of the arm's flange (`tool0`):
    it fixes a `GRIPPER_TCP_FRAME_NAME` ("gripper_tcp") frame to `tool0` at that offset, usable via
    `tool_frame_name=GRIPPER_TCP_FRAME_NAME`.

    Only single-arm URDFs are supported.
    """

    def __init__(self, scene: SingleArmScene, plant_context: Context) -> None:
        self._scene = scene
        self._plant_context = plant_context

    @classmethod
    def from_urdf_path(
        cls,
        urdf_path: str,
        base_link_name: str = "base_link",
        base_transform: HomogeneousMatrixType | None = None,
        gripper_transform: HomogeneousMatrixType | None = None,
    ) -> "Kinematics":
        """Build from a URDF file on disk (use this for mesh-bearing URDFs, so Drake resolves mesh paths)."""
        transform = RigidTransform() if base_transform is None else RigidTransform(base_transform)
        gripper = None if gripper_transform is None else RigidTransform(gripper_transform)
        return cls(*_build_single_arm_scene(urdf_path, False, base_link_name, transform, gripper))

    @classmethod
    def from_urdf_string(
        cls,
        urdf_string: str,
        base_link_name: str = "base_link",
        base_transform: HomogeneousMatrixType | None = None,
        gripper_transform: HomogeneousMatrixType | None = None,
    ) -> "Kinematics":
        """Build from URDF content already in memory."""
        transform = RigidTransform() if base_transform is None else RigidTransform(base_transform)
        gripper = None if gripper_transform is None else RigidTransform(gripper_transform)
        return cls(*_build_single_arm_scene(urdf_string, True, base_link_name, transform, gripper))

    def forward_kinematics(self, q: JointConfigurationType, tool_frame_name: str = "tool0") -> HomogeneousMatrixType:
        """`tool_frame_name`'s pose (4x4) in world at joint configuration `q`."""
        plant = self._scene.robot_diagram.plant()
        plant.SetPositions(self._plant_context, self._scene.arm_index, np.asarray(q, dtype=float).reshape(-1))
        frame = plant.GetFrameByName(tool_frame_name, self._scene.arm_index)
        return plant.CalcRelativeTransform(self._plant_context, plant.world_frame(), frame).GetAsMatrix4()

    def _refinement_seed(
        self, tcp_pose: HomogeneousMatrixType, q_seed: JointConfigurationType, tool_frame_name: str
    ) -> "JointConfigurationType | None":
        """The joint configuration to seed the numerical refine at, or None if none exists.

        Subclasses override this to derive a better seed from `tcp_pose` (e.g. an analytic
        branch-pick); the base uses the caller's seed directly. `tool_frame_name` is the frame
        `tcp_pose` is expressed for, needed by subclasses whose seed source targets a fixed frame
        (e.g. `tool0`) that may differ from it.
        """
        return q_seed

    def inverse_kinematics_closest(
        self,
        tcp_pose: HomogeneousMatrixType,
        q_seed: JointConfigurationType,
        tool_frame_name: str = "tool0",
        position_tolerance: float = 1e-5,
        orientation_tolerance: float = 1e-4,
        seed_deviation_threshold: float = _SEED_DEVIATION_THRESHOLD,
    ) -> "KinematicsResult | None":
        """Numerically solve for the joint configuration reaching `tcp_pose`, closest to `q_seed`.

        Constrains `tool_frame_name` to `tcp_pose` (within the tolerances) and adds a
        stay-near-seed cost so the solve stays on the seed's branch.

        Returns a `KinematicsResult`, or None if no seed exists or the solver did not converge.
        """
        tcp_pose = np.asarray(tcp_pose, dtype=float)
        seed = self._refinement_seed(tcp_pose, np.asarray(q_seed, dtype=float).reshape(-1), tool_frame_name)
        if seed is None:
            return None

        plant = self._scene.robot_diagram.plant()
        arm_index = self._scene.arm_index
        world_frame = plant.world_frame()
        tool_frame = plant.GetFrameByName(tool_frame_name, arm_index)

        inverse_kinematics = DrakeInverseKinematics(plant, self._plant_context)
        inverse_kinematics.AddPositionConstraint(
            tool_frame,
            np.zeros(3),
            world_frame,
            tcp_pose[:3, 3] - position_tolerance,
            tcp_pose[:3, 3] + position_tolerance,
        )
        inverse_kinematics.AddOrientationConstraint(
            world_frame,
            RotationMatrix(tcp_pose[:3, :3]),
            tool_frame,
            RotationMatrix(),
            orientation_tolerance,
        )

        program = inverse_kinematics.prog()
        q = inverse_kinematics.q()
        program.SetInitialGuess(q, seed)
        program.AddQuadraticErrorCost(np.eye(seed.size), seed, q)  # type: ignore  # stay on the seed's branch

        solver = IpoptSolver()
        options = SolverOptions()
        options.SetOption(solver.solver_id(), "print_level", 0)
        options.SetOption(solver.solver_id(), "sb", "yes")
        result = solver.Solve(program, None, options)
        if not result.is_success():
            return None
        q_solution = plant.GetPositionsFromArray(arm_index, result.GetSolution(q))

        # The stay-near-seed cost is soft, so report how far the solve actually stayed from the seed.
        per_joint_deviation = np.abs((q_solution - seed + np.pi) % (2 * np.pi) - np.pi)  # wrap to [-pi, pi)
        max_deviation = float(np.max(per_joint_deviation))
        return KinematicsResult(
            joint_configuration=q_solution,
            is_close_to_seed=max_deviation <= seed_deviation_threshold,
            max_seed_deviation=max_deviation,
        )
