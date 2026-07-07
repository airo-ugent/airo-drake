"""DH to URDF conversion for Universal Robots arms.

The UR controller exposes its per-robot *calibrated* DH parameters on the primary
interface (see `extraction.py`). This module creates a URDF with these calibrated parameters
for use with Drake's numerical IK.

Note: unlike the ROS-normalized `airo_models` UR URDFs, the intermediate link frames
here are the raw DH frames, so `base_link` must be welded to the world with an
*identity* transform (not `airo_drake.X_URBASE_ROSBASE`).
"""

import numpy as np
from airo_typing import HomogeneousMatrixType, JointConfigurationType

_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
_LINK_NAMES = [
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
]

_INERTIAL_XML = """    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>"""


def _rot_x(angle: float) -> HomogeneousMatrixType:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def _rot_z(angle: float) -> HomogeneousMatrixType:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def _trans_x(offset: float) -> HomogeneousMatrixType:
    transform = np.eye(4)
    transform[0, 3] = offset
    return transform


def _trans_z(offset: float) -> HomogeneousMatrixType:
    transform = np.eye(4)
    transform[2, 3] = offset
    return transform


def dh_matrix(theta: float, d: float, a: float, alpha: float) -> HomogeneousMatrixType:
    """Single DH link transform at joint value 0 (theta is the offset)."""
    return _rot_z(theta) @ _trans_z(d) @ _trans_x(a) @ _rot_x(alpha)


def fk_dh(dh: dict, q: JointConfigurationType) -> HomogeneousMatrixType:
    """Base->flange forward kinematics from raw DH arrays. q: 6 joint values."""
    transform = np.eye(4)
    for i in range(6):
        transform = transform @ (
            _rot_z(dh["theta"][i] + q[i])
            @ _trans_z(dh["d"][i])
            @ _trans_x(dh["a"][i])
            @ _rot_x(dh["alpha"][i])
        )
    return transform


def dh_to_origins(dh: dict) -> list[HomogeneousMatrixType]:
    """Return 7 homogeneous origins: O_1..O_6 (joints) then O_tool (flange).

    See the module docstring: this is the DH product regrouped so each O_i lines up
    with `ur_description`'s own per-link convention (`alpha_{i-1}`/`a_{i-1}` lead,
    `d_i`/`theta_i` trail), not just the overall base->flange pose.
    """
    theta, a, d, alpha = (
        np.asarray(dh[key], dtype=float) for key in ("theta", "a", "d", "alpha")
    )
    origins = []
    alpha_prev, a_prev = 0.0, 0.0
    for i in range(6):
        origins.append(
            _rot_x(alpha_prev) @ _trans_x(a_prev) @ _trans_z(d[i]) @ _rot_z(theta[i])
        )
        alpha_prev, a_prev = alpha[i], a[i]
    origins.append(
        _rot_x(alpha_prev) @ _trans_x(a_prev)
    )  # O_tool: fixed wrist_3 -> flange
    return origins


def mat_to_xyz_rpy(transform: HomogeneousMatrixType) -> tuple[np.ndarray, np.ndarray]:
    """URDF rpy convention: R = Rz(yaw) Ry(pitch) Rx(roll) (extrinsic xyz)."""
    xyz = transform[:3, 3]
    rotation = transform[:3, :3]
    sy = np.hypot(rotation[0, 0], rotation[1, 0])
    if sy > 1e-9:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:  # gimbal lock: pitch = +/- 90 deg
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0.0
    return xyz, np.array([roll, pitch, yaw])


def xyz_rpy_to_mat(xyz: np.ndarray, rpy: np.ndarray) -> HomogeneousMatrixType:
    roll, pitch, yaw = rpy
    rot_y_pitch = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch), 0],
            [0, 1, 0, 0],
            [-np.sin(pitch), 0, np.cos(pitch), 0],
            [0, 0, 0, 1],
        ]
    )
    transform = _rot_z(yaw) @ rot_y_pitch @ _rot_x(roll)
    transform[:3, 3] = xyz
    return transform


def _format_floats(values: np.ndarray) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def _origin_xml(transform: HomogeneousMatrixType) -> str:
    xyz, rpy = mat_to_xyz_rpy(transform)
    return f'<origin xyz="{_format_floats(xyz)}" rpy="{_format_floats(rpy)}"/>'


def calibrated_dh_to_urdf(dh: dict, robot_name: str = "ur") -> str:
    """Return a URDF string reproducing the DH forward kinematics exactly.

    The URDF has no visual or collision geometry and exists only for
    calibrated FK/IK.
    Use the nominal `airo_models` mesh model for collision checking and
    visualization instead.

    Args:
        dh: dict with keys "theta", "a", "d", "alpha" (each a list/array of 6 floats),
            as returned by `extraction.read_calibrated_dh` or `io.load_calibrated_dh`.
        robot_name: used for the URDF's robot name (`f"{robot_name}_calibrated"`).

    Returns:
        A URDF (xml) string with `base_link` (DH frame 0) and `tool0` (DH frame 6).
    """
    origins = dh_to_origins(dh)
    link_names = ["base_link"] + _LINK_NAMES
    parent_names = ["base_link"] + _LINK_NAMES[:-1]
    lines = ['<?xml version="1.0"?>', f'<robot name="{robot_name}_calibrated">']
    for k, name in enumerate(link_names):
        lines.append(f'  <link name="{name}">\n{_INERTIAL_XML}\n  </link>')
        if k == 0:
            continue
        i = k - 1  # joint i connects link k-1 -> k
        lines.append(
            f'  <joint name="{_JOINT_NAMES[i]}" type="revolute">\n'
            f'    <parent link="{parent_names[i]}"/>\n'
            f'    <child link="{_LINK_NAMES[i]}"/>\n'
            f"    {_origin_xml(origins[i])}\n"
            '    <axis xyz="0 0 1"/>\n'
            '    <limit lower="-6.283185" upper="6.283185" effort="150" velocity="3.14"/>\n'
            "  </joint>"
        )
    # fixed wrist_3 -> flange (tool0), DH frame 6
    lines.append('  <link name="tool0"/>')
    lines.append(
        '  <joint name="wrist_3-flange" type="fixed">\n'
        '    <parent link="wrist_3_link"/>\n'
        '    <child link="tool0"/>\n'
        f"    {_origin_xml(origins[6])}\n"
        "  </joint>"
    )
    lines.append("</robot>\n")
    return "\n".join(lines)
