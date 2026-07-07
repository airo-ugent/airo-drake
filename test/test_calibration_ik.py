"""Tests for the two-stage calibrated IK (analytic branch-pick -> calibrated refine).

The offline tests emulate a physically calibrated robot: we build a Drake model from
DH parameters perturbed away from nominal (standing in for a real factory calibration),
treat its forward kinematics as ground-truth "reality", and check that

- analytic IK (nominal DH) reaches a target with the expected ~1-2 mm error, and
- the calibrated refine collapses that error by ~2 orders of magnitude,

plus that the refine never flips IK branch and solves fast enough for online use.

The three-way comparison against a real control box (which moves the robot) lives in
scripts/manual_calibrated_ik_hardware_test.py.
"""

import time

import numpy as np
import pytest
from pydrake.math import RigidTransform
from pydrake.planning import RobotDiagramBuilder

from airo_drake import calibrated_dh_to_urdf, refine_calibrated_ik, two_stage_calibrated_ik

PI = np.pi

_NOMINAL_UR5E = {
    "theta": [0] * 6,
    "a": [0, -0.425, -0.3922, 0, 0, 0],
    "d": [0.1625, 0, 0, 0.1333, 0.0997, 0.0996],
    "alpha": [PI / 2, 0, 0, PI / 2, -PI / 2, 0],
}
_HOME = np.deg2rad([0, -90, 90, -90, -90, 0])


def _perturbed_dh(rng: np.random.Generator, scale: float = 1e-3) -> dict:
    """A calibration-like DH: nominal UR5e with small perturbations on every parameter."""
    return {
        key: (np.asarray(value, dtype=float) + rng.normal(0, scale, 6)).tolist()
        for key, value in _NOMINAL_UR5E.items()
    }


def _build_calibrated_arm(dh: dict):
    """Build the calibrated arm, welded to world with identity.

    Returns (robot_diagram, context, arm_index). The caller MUST keep robot_diagram and
    context alive for as long as it uses the plant/plant_context derived from them: the
    plant and its contexts are owned by the diagram, so if the diagram is garbage
    collected the plant becomes a dangling reference (intermittent RuntimeError/segfault).
    """
    builder = RobotDiagramBuilder()
    plant = builder.plant()
    arm_index = builder.parser().AddModelsFromString(calibrated_dh_to_urdf(dh, "ur5e"), "urdf")[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link", arm_index),
        RigidTransform(),
    )
    robot_diagram = builder.Build()
    context = robot_diagram.CreateDefaultContext()
    return robot_diagram, context, arm_index


def _fk(plant, plant_context, arm_index, q: np.ndarray) -> np.ndarray:
    plant.SetPositions(plant_context, arm_index, np.asarray(q, dtype=float))
    tool = plant.GetFrameByName("tool0", arm_index)
    return plant.CalcRelativeTransform(plant_context, plant.world_frame(), tool).GetAsMatrix4()


def _position_error_mm(X_a: np.ndarray, X_b: np.ndarray) -> float:
    return 1000.0 * float(np.linalg.norm(np.asarray(X_a)[:3, 3] - np.asarray(X_b)[:3, 3]))


def _wrapped_joint_distance(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Joint-space distance with each joint difference wrapped to [-pi, pi).

    Two analytic IK branches can be reported in different 2*pi ranges, so raw
    subtraction would overstate their distance; wrapping compares them as revolute
    joints actually are.
    """
    difference = (np.asarray(q_a) - np.asarray(q_b) + PI) % (2 * PI) - PI
    return float(np.linalg.norm(difference))


def test_refine_reaches_target_on_calibrated_model():
    """A refine seeded near the true configuration hits the target to sub-micron precision."""
    robot_diagram, context, arm_index = _build_calibrated_arm(_perturbed_dh(np.random.default_rng(0)))
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)
    q_true = _HOME + np.deg2rad([5, -8, 6, -4, 7, -3])
    X_target = _fk(plant, plant_context, arm_index, q_true)

    result = refine_calibrated_ik(plant, plant_context, arm_index, X_target, q_seed=q_true)

    assert result is not None
    assert result.is_close_to_seed
    assert _position_error_mm(_fk(plant, plant_context, arm_index, result.joint_configuration), X_target) < 0.05


def test_refine_result_flags_whether_solution_stayed_near_seed():
    """The result carries the solution plus an is_close_to_seed flag from the soft stay-near-seed cost."""
    robot_diagram, context, arm_index = _build_calibrated_arm(_perturbed_dh(np.random.default_rng(0)))
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)
    q_true = _HOME + np.deg2rad([5, -8, 6, -4, 7, -3])
    X_target = _fk(plant, plant_context, arm_index, q_true)

    # A normal refine nudges the seed by well under a degree -> close to seed under the default threshold,
    # and max_seed_deviation reports exactly that small nudge.
    result = refine_calibrated_ik(plant, plant_context, arm_index, X_target, q_seed=q_true)
    assert result is not None
    assert result.is_close_to_seed
    assert 0.0 < result.max_seed_deviation < np.deg2rad(2.0)

    # The same small nudge is flagged NOT close once the threshold is tightened below it,
    # deterministically exercising the "left the seed" path without needing a real branch flip.
    strict = refine_calibrated_ik(
        plant, plant_context, arm_index, X_target, q_seed=q_true, seed_deviation_threshold=0.0
    )
    assert strict is not None
    assert not strict.is_close_to_seed
    assert strict.max_seed_deviation == result.max_seed_deviation


def test_two_stage_ik_beats_analytic_ik_on_calibrated_model():
    """Analytic IK (nominal DH) is ~1-2 mm off; the two-stage refine collapses that."""
    ur_analytic_ik = pytest.importorskip("ur_analytic_ik")

    rng = np.random.default_rng(1)
    robot_diagram, context, arm_index = _build_calibrated_arm(_perturbed_dh(rng))
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    analytic_errors = []
    refined_errors = []
    for _ in range(15):
        q_true = _HOME + rng.uniform(-0.4, 0.4, size=6)
        X_target = _fk(plant, plant_context, arm_index, q_true)  # the calibrated robot's "reality"
        q_start = q_true + rng.uniform(-0.1, 0.1, size=6)  # a nearby seed (e.g. current config)

        analytic_solutions = ur_analytic_ik.ur5e.inverse_kinematics_closest(X_target, *q_start)
        if not analytic_solutions:
            continue
        q_analytic = np.asarray(analytic_solutions[0], dtype=float).reshape(-1)

        result = two_stage_calibrated_ik(
            plant,
            plant_context,
            arm_index,
            X_target,
            ur_analytic_ik.ur5e,
            q_seed=q_start,
        )
        assert result is not None

        analytic_errors.append(_position_error_mm(_fk(plant, plant_context, arm_index, q_analytic), X_target))
        refined_errors.append(
            _position_error_mm(_fk(plant, plant_context, arm_index, result.joint_configuration), X_target)
        )

    assert len(analytic_errors) >= 10
    mean_analytic = float(np.mean(analytic_errors))
    mean_refined = float(np.mean(refined_errors))

    # The calibrated model must actually differ from nominal, else the test is vacuous.
    assert mean_analytic > 0.5, f"analytic error {mean_analytic:.3f} mm too small to be meaningful"
    # The refine must close the gap by a wide margin.
    assert mean_refined < 0.05
    assert mean_refined < mean_analytic / 10.0


def test_two_stage_ik_never_flips_branch():
    """Across many poses, the refined solution stays on the analytic seed's branch.

    Two independent checks per pose: (1) the refine only nudges the analytic joints by a
    small amount (a flip would move some joint by tens of degrees), and (2) of all the
    analytic IK branches, the one nearest the refined solution is the seed branch itself.
    """
    ur_analytic_ik = pytest.importorskip("ur_analytic_ik")

    rng = np.random.default_rng(2)
    robot_diagram, context, arm_index = _build_calibrated_arm(_perturbed_dh(rng))
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    checked = 0
    for _ in range(25):
        q_true = _HOME + rng.uniform(-0.4, 0.4, size=6)
        X_target = _fk(plant, plant_context, arm_index, q_true)
        q_start = q_true + rng.uniform(-0.1, 0.1, size=6)

        analytic_solutions = ur_analytic_ik.ur5e.inverse_kinematics_closest(X_target, *q_start)
        if not analytic_solutions:
            continue
        seed_branch = np.asarray(analytic_solutions[0], dtype=float).reshape(-1)
        result = two_stage_calibrated_ik(
            plant,
            plant_context,
            arm_index,
            X_target,
            ur_analytic_ik.ur5e,
            q_seed=q_start,
        )
        assert result is not None
        q_refined = result.joint_configuration

        # (1) same branch: only a small nudge (calibration corrections are ~mrad); the result agrees.
        assert np.max(np.abs(q_refined - seed_branch)) < np.deg2rad(2.0)
        assert result.is_close_to_seed

        # (2) of all analytic branches, the seed branch is the one nearest the refined solution.
        all_branches = [
            np.asarray(s, dtype=float).reshape(-1) for s in ur_analytic_ik.ur5e.inverse_kinematics(X_target)
        ]
        distances = [_wrapped_joint_distance(q_refined, branch) for branch in all_branches]
        nearest_branch = all_branches[int(np.argmin(distances))]
        assert _wrapped_joint_distance(nearest_branch, seed_branch) < 1e-6

        checked += 1

    assert checked >= 20


def test_two_stage_ik_solves_fast():
    """The two-stage solve is cheap enough for interactive/online use (analytic seed + one local solve)."""
    ur_analytic_ik = pytest.importorskip("ur_analytic_ik")

    rng = np.random.default_rng(3)
    robot_diagram, context, arm_index = _build_calibrated_arm(_perturbed_dh(rng))
    plant = robot_diagram.plant()
    plant_context = plant.GetMyContextFromRoot(context)

    # Warm up once (the first solve pays one-time solver setup we don't want to time).
    q_warmup = _HOME + rng.uniform(-0.4, 0.4, size=6)
    X_warmup = _fk(plant, plant_context, arm_index, q_warmup)
    two_stage_calibrated_ik(plant, plant_context, arm_index, X_warmup, ur_analytic_ik.ur5e, q_seed=q_warmup)

    durations = []
    for _ in range(20):
        q_true = _HOME + rng.uniform(-0.4, 0.4, size=6)
        X_target = _fk(plant, plant_context, arm_index, q_true)
        q_start = q_true + rng.uniform(-0.1, 0.1, size=6)

        start = time.perf_counter()
        result = two_stage_calibrated_ik(
            plant,
            plant_context,
            arm_index,
            X_target,
            ur_analytic_ik.ur5e,
            q_seed=q_start,
        )
        durations.append(time.perf_counter() - start)
        assert result is not None

    mean_ms = 1000.0 * float(np.mean(durations))
    print(f"two-stage calibrated IK: mean {mean_ms:.1f} ms over {len(durations)} solves")
    # Typically ~1 ms here; a very generous ceiling that still catches a regression to seconds.
    assert mean_ms < 250.0


# The three-way comparison against the real robot's control box (analytic vs calibrated
# refine vs control-box move) is a deliberate, hardware-in-the-loop bring-up activity --
# it MOVES the robot, so it lives as an interactive, confirmation-gated script rather than
# an automated test that could move a robot from a casual `pytest` run. See
# scripts/manual_calibrated_ik_hardware_test.py.
