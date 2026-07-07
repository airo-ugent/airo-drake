"""Tests for the general, any-URDF `Kinematics` class (no calibration, no analytic seed).

`test_calibrated_kinematics.py` exercises the calibrated-DH/analytic-seed specialization
(`CalibratedKinematics`); these tests exercise the general base class directly, on a plain,
nominal `airo_models` URDF, to confirm it's correct standalone public API.
"""

import airo_models
import numpy as np

from airo_drake import X_URBASE_ROSBASE, X_URTOOL0_ROBOTIQ, Kinematics
from airo_drake.kinematics import GRIPPER_TCP_FRAME_NAME

_HOME = np.deg2rad([0, -90, 90, -90, -90, 0])


def _ur5e_kinematics() -> Kinematics:
    return Kinematics.from_urdf_path(airo_models.get_urdf_path("ur5e"), base_transform=X_URBASE_ROSBASE)


def test_inverse_kinematics_closest_reaches_target_seeded_at_true_configuration():
    """A refine seeded near the true configuration hits the target to sub-micron precision."""
    kinematics = _ur5e_kinematics()
    q_true = _HOME + np.deg2rad([5, -8, 6, -4, 7, -3])
    X_target = kinematics.forward_kinematics(q_true)

    result = kinematics.inverse_kinematics_closest(X_target, q_seed=q_true)

    assert result is not None
    assert result.is_close_to_seed
    position_error_mm = 1000.0 * float(
        np.linalg.norm(kinematics.forward_kinematics(result.joint_configuration)[:3, 3] - X_target[:3, 3])
    )
    assert position_error_mm < 0.05


def test_inverse_kinematics_closest_returns_none_for_unreachable_target():
    """A target far outside the arm's reach must not silently return a bogus solution."""
    kinematics = _ur5e_kinematics()
    X_unreachable = np.eye(4)
    X_unreachable[:3, 3] = [10.0, 10.0, 10.0]  # 10 m away, far beyond a UR5e's ~0.85 m reach

    result = kinematics.inverse_kinematics_closest(X_unreachable, q_seed=_HOME)

    assert result is None


def test_gripper_transform_offsets_forward_kinematics_from_tool0():
    """`gripper_transform` fixes a `gripper_tcp` frame to `tool0` at exactly that offset."""
    kinematics = Kinematics.from_urdf_path(
        airo_models.get_urdf_path("ur5e"), base_transform=X_URBASE_ROSBASE, gripper_transform=X_URTOOL0_ROBOTIQ
    )
    q = _HOME + np.deg2rad([3, -4, 5, -6, 7, -8])

    X_tool0 = kinematics.forward_kinematics(q, tool_frame_name="tool0")
    X_gripper_tcp = kinematics.forward_kinematics(q, tool_frame_name=GRIPPER_TCP_FRAME_NAME)
    X_expected = X_tool0 @ X_URTOOL0_ROBOTIQ.GetAsMatrix4()

    assert np.abs(X_gripper_tcp - X_expected).max() < 1e-9


def test_inverse_kinematics_closest_reaches_target_on_gripper_tcp_frame():
    """IK against `tool_frame_name=GRIPPER_TCP_FRAME_NAME` solves for the gripper's TCP, not the flange."""
    kinematics = Kinematics.from_urdf_path(
        airo_models.get_urdf_path("ur5e"), base_transform=X_URBASE_ROSBASE, gripper_transform=X_URTOOL0_ROBOTIQ
    )
    q_true = _HOME + np.deg2rad([5, -8, 6, -4, 7, -3])
    X_target = kinematics.forward_kinematics(q_true, tool_frame_name=GRIPPER_TCP_FRAME_NAME)

    result = kinematics.inverse_kinematics_closest(X_target, q_seed=q_true, tool_frame_name=GRIPPER_TCP_FRAME_NAME)

    assert result is not None
    position_error_mm = 1000.0 * float(
        np.linalg.norm(
            kinematics.forward_kinematics(result.joint_configuration, tool_frame_name=GRIPPER_TCP_FRAME_NAME)[:3, 3]
            - X_target[:3, 3]
        )
    )
    assert position_error_mm < 0.05
