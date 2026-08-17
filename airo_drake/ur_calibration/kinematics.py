"""Calibrated forward/inverse kinematics for UR arms, from a robot's calibrated DH parameters."""

from typing import Any

import numpy as np
from airo_typing import HomogeneousMatrixType, JointConfigurationType
from pydrake.math import RigidTransform

from airo_drake.kinematics import GRIPPER_TCP_FRAME_NAME, Kinematics, _build_single_arm_scene
from airo_drake.ur_calibration.conversion import calibrated_dh_to_urdf


class CalibratedKinematics(Kinematics):
    """Kinematics for a UR arm from its calibrated DH parameters.

        calibrated_kinematics = CalibratedKinematics(dh, "ur5e")
        result = calibrated_kinematics.inverse_kinematics_closest(tcp_pose, q_seed)

    Built from a calibrated DH dict (see `calibrated_dh_to_urdf`) rather than a URDF file; the
    DH model is welded to the world with an identity transform.

    Pass `analytic_ik_model` (a `ur-analytic-ik` robot module, e.g. `ur_analytic_ik.ur5e`) to
    seed `inverse_kinematics_closest` with an analytic branch-pick before the numerical refine:
    analytic IK reliably picks the joint-configuration branch, and the refine then corrects the
    ~1-2 mm error of its nominal DH. Pass `gripper_transform` for FK/IK on a gripper's TCP, as in
    `Kinematics`; the analytic branch-pick still works, converting the TCP target back to `tool0`
    (the frame `analytic_ik_model` expects) before seeding.

    This model is for kinematics only! It has no visual or collision geometry; use
    `add_manipulator`'s mesh model for collision checking and visualization.
    """

    def __init__(
        self,
        dh: dict,
        robot_name: str = "ur",
        analytic_ik_model: Any | None = None,
        gripper_transform: HomogeneousMatrixType | None = None,
    ) -> None:
        gripper_rigid_transform = None if gripper_transform is None else RigidTransform(gripper_transform)
        scene, plant_context = _build_single_arm_scene(
            calibrated_dh_to_urdf(dh, robot_name), True, "base_link", RigidTransform(), gripper_rigid_transform
        )
        super().__init__(scene, plant_context)
        self._analytic_ik_model = analytic_ik_model
        self._gripper_transform = gripper_rigid_transform

    def _refinement_seed(
        self, tcp_pose: HomogeneousMatrixType, q_seed: JointConfigurationType, tool_frame_name: str
    ) -> "JointConfigurationType | None":
        """Analytic branch-pick nearest `q_seed` if an `analytic_ik_model` was given, else `q_seed`.

        `analytic_ik_model` always solves for `tool0`'s pose, so if `tcp_pose` targets the gripper
        TCP frame instead, it's converted back to `tool0` first.

        Returns None when analytic IK finds no solution for `tcp_pose`.
        """
        if self._analytic_ik_model is None:
            return q_seed
        tool0_pose = tcp_pose
        if tool_frame_name == GRIPPER_TCP_FRAME_NAME and self._gripper_transform is not None:
            tool0_pose = tcp_pose @ self._gripper_transform.inverse().GetAsMatrix4()
        solutions = self._analytic_ik_model.inverse_kinematics_closest(tool0_pose, *q_seed)
        if not solutions:
            return None
        return np.asarray(solutions[0], dtype=float).reshape(-1)
