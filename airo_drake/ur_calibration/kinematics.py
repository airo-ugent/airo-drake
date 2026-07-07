"""Calibrated forward/inverse kinematics for UR arms, from a robot's calibrated DH parameters."""

from typing import Any

import numpy as np
from airo_typing import HomogeneousMatrixType, JointConfigurationType
from pydrake.math import RigidTransform

from airo_drake.kinematics import Kinematics, _build_single_arm_scene
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
    ~1-2 mm error of its nominal DH.

    This model is for kinematics only! It has no visual or collision geometry; use
    `add_manipulator`'s mesh model for collision checking and visualization.
    """

    def __init__(self, dh: dict, robot_name: str = "ur", analytic_ik_model: Any | None = None) -> None:
        scene, plant_context = _build_single_arm_scene(
            calibrated_dh_to_urdf(dh, robot_name), True, "base_link", RigidTransform()
        )
        super().__init__(scene, plant_context)
        self._analytic_ik_model = analytic_ik_model

    def _refinement_seed(
        self, tcp_pose: HomogeneousMatrixType, q_seed: JointConfigurationType
    ) -> "JointConfigurationType | None":
        """Analytic branch-pick nearest `q_seed` if an `analytic_ik_model` was given, else `q_seed`.

        Returns None when analytic IK finds no solution for `tcp_pose`.
        """
        if self._analytic_ik_model is None:
            return q_seed
        solutions = self._analytic_ik_model.inverse_kinematics_closest(tcp_pose, *q_seed)
        if not solutions:
            return None
        return np.asarray(solutions[0], dtype=float).reshape(-1)
