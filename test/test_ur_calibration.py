import numpy as np

from airo_drake.ur_calibration.conversion import (
    calibrated_dh_to_urdf,
    dh_to_origins,
    fk_dh,
    mat_to_xyz_rpy,
    xyz_rpy_to_mat,
)

PI = np.pi

# Nominal DH (UR official). a2, a3 negative per UR convention.
_NOMINAL_DH = {
    "ur5e": {
        "theta": [0] * 6,
        "a": [0, -0.425, -0.3922, 0, 0, 0],
        "d": [0.1625, 0, 0, 0.1333, 0.0997, 0.0996],
        "alpha": [PI / 2, 0, 0, PI / 2, -PI / 2, 0],
    },
    "ur3e": {
        "theta": [0] * 6,
        "a": [0, -0.24355, -0.2132, 0, 0, 0],
        "d": [0.15185, 0, 0, 0.13105, 0.08535, 0.0921],
        "alpha": [PI / 2, 0, 0, PI / 2, -PI / 2, 0],
    },
}


def _fk_from_written_urdf_origins(dh: dict, q: np.ndarray) -> np.ndarray:
    """FK reconstructed from the xyz/rpy values that would actually be written to the URDF."""
    origins = dh_to_origins(dh)
    xyz_rpy = [mat_to_xyz_rpy(origin) for origin in origins]
    transform = np.eye(4)
    for i in range(6):
        origin = xyz_rpy_to_mat(*xyz_rpy[i])  # recomposed from text, as Drake would parse it
        transform = transform @ origin @ _rot_z(q[i])
    transform = transform @ xyz_rpy_to_mat(*xyz_rpy[6])  # fixed flange origin
    return transform


def _rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def _assert_urdf_fk_matches_dh_fk(dh: dict, n_samples: int = 200) -> None:
    rng = np.random.default_rng(0)
    for _ in range(n_samples):
        q = rng.uniform(-PI, PI, size=6)
        error = np.abs(fk_dh(dh, q) - _fk_from_written_urdf_origins(dh, q)).max()
        assert error < 1e-9, f"URDF FK diverged from DH FK by {error:.2e}"


def test_urdf_fk_matches_dh_fk_for_nominal_ur5e():
    _assert_urdf_fk_matches_dh_fk(_NOMINAL_DH["ur5e"])


def test_urdf_fk_matches_dh_fk_for_nominal_ur3e():
    _assert_urdf_fk_matches_dh_fk(_NOMINAL_DH["ur3e"])


def test_urdf_fk_matches_dh_fk_for_randomized_dh():
    rng = np.random.default_rng(7)
    for _ in range(3):
        dh = {
            "theta": rng.uniform(-PI, PI, 6).tolist(),
            "a": rng.uniform(-0.5, 0.5, 6).tolist(),
            "d": rng.uniform(-0.3, 0.3, 6).tolist(),
            "alpha": rng.uniform(-PI, PI, 6).tolist(),
        }
        _assert_urdf_fk_matches_dh_fk(dh, n_samples=200)


def test_calibrated_dh_to_urdf_has_no_visual_or_collision_geometry():
    # This URDF is for tool0 IK only -- it must not be usable for collision checking
    # or visualization by mistake (see the module docstring's "Which model for what").
    urdf = calibrated_dh_to_urdf(_NOMINAL_DH["ur5e"], "ur5e")
    assert "<collision>" not in urdf
    assert "<visual>" not in urdf
