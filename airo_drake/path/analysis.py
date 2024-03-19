import numpy as np
from airo_typing import JointConfigurationType, JointPathType


def find_closest_configuration(
    reference_configuration: JointConfigurationType, candidates: list[JointConfigurationType]
) -> JointConfigurationType:
    """Finds the closest configuration to a reference configuration within a set of candidates."""
    candidates_array = np.array(candidates).squeeze()
    distances = np.linalg.norm(candidates_array - reference_configuration, axis=1)  # Vectorized calculation
    closest_index = np.argmin(distances)
    return candidates_array[closest_index]


def calculate_joint_path_distances(path: JointPathType) -> np.ndarray:
    """Calculate the distances between consecutive joint configurations in a joint path.

    Args:
        path: A path of joint configurations.

    Returns:
        An array of distances between consecutive joint configurations (1 shorter than path).
    """
    return np.linalg.norm(np.diff(path, axis=0), axis=1)


def calculate_joint_path_length(path: JointPathType) -> float:
    """Calculate the length of a joint path.

    Args:
        path: A path of joint configurations.

    Returns:
        The length of the joint path.
    """
    return np.sum(calculate_joint_path_distances(path))


def calculate_joint_path_outlier_threshold(distances: np.ndarray, iqr_multiplier: float = 20.0) -> float:
    """Calculate a threshold for detecting unusually large jumps in a joint path.

    We use a one-sided IQR-based (interquartile range) method:
    * One-sided because we don't care about unusually small jumps.
    * IQR-based instead of using standard deviation because it's less sensitive to outliers.

    Jumps between consecutive IK solutions can happen for a few reasons:
    * Singularities: Near robot singularities, small changes in the end-effector
    pose can lead to large, discontinuous changes in the joint configurations required.
    * Incompleteness: the IK solver might not find the solution that is closest to the previous one.

    Args:
        distance: The distances between consecutive joint configurations.
        iqr_multiplier: A multiplier for the interquartile range.

    Returns:
        A threshold for detecting unusually large jumps in joint configuration distances.
    """
    q1, q3 = np.percentile(distances, [25, 75])
    interquartile_range = q3 - q1
    threshold = q3 + (iqr_multiplier * interquartile_range)
    return threshold


def joint_path_has_large_jumps(path: JointPathType, iqr_multiplier: float = 20.0) -> bool:
    """Check a joint path for unusually large jumps in joint configuration distances.

    See the docstring for `calculate_joint_path_iqr_threshold` for more details.

    Args:
        path: A path of joint configurations.
        iqr_multiplier: A multiplier for the interquartile range.

    Returns:
        A boolean indicating whether the joint path contains unusually large jumps.
    """
    distances = calculate_joint_path_distances(path)
    threshold = calculate_joint_path_outlier_threshold(distances, iqr_multiplier)
    return bool(np.any(distances > threshold))
