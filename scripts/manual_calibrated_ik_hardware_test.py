"""Manual, confirmation-gated hardware bring-up for the two-stage calibrated IK.

Run this with a UR arm connected (in remote control) to see, on the real robot, that
the calibrated refine closes the ~1-2 mm gap left by nominal analytic IK. It reads the
robot's OWN calibrated DH, so it works with real calibration data out of the box.

Before anything moves, it runs a frame-consistency check: our calibrated tool0 FK must
match the control box's TCP pose at the current joints, otherwise our target poses are
not in the frame the control box expects (a nonzero TCP/payload offset or a base-frame
mismatch) and the control-box move would go somewhere unintended. It aborts on a
mismatch unless --allow-frame-mismatch is given.

Then, for one target pose, it walks through, pausing for you to confirm before every motion:

  1. analytic IK (nominal DH, ur-analytic-ik)      -> q_analytic
  2. VERIFY q_analytic in MeshCat (nominal mesh model) before moving
  3. moveJ to q_analytic; report achieved TCP error
  4. refine against the calibrated model            -> q_refined
  5. VERIFY q_refined in MeshCat, then moveJ to it; report achieved TCP error
  6. control-box move to the target pose (its own calibrated IK); report error + joints

Expected: step 3 ~1-2 mm off, steps 5 and 6 sub-mm and close to each other.
In manual tests, we've experienced that between steps 5 and 6, movement is visually imperceptible;
you just head a short thud as the robot settles into the slightly different (sub-mm delta) configuration.

Usage:
    python scripts/manual_calibrated_ik_hardware_test.py --ip 10.42.0.162 --model ur5e

Notes:
  - Set the controller TCP/payload offset to ZERO so all
    strategies compare flange (tool0) poses in the same frame.
  - MeshCat shows the NOMINAL mesh model driven to the computed joints (the calibrated
    model has no geometry on purpose); it is a safety preview of the arm configuration,
    not a precision frame overlay.
  - Requires airo-robots, ur-analytic-ik, and a UR robot.
"""

import argparse
from typing import Any, Callable

import numpy as np
from airo_typing import HomogeneousMatrixType, JointConfigurationType
from pydrake.geometry import Meshcat
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.planning import RobotDiagram
from pydrake.systems.framework import Context

from airo_drake import (
    CalibratedKinematics,
    KinematicsResult,
    add_manipulator,
    add_meshcat,
    finish_build,
    read_calibrated_dh,
)


def _position_error_mm(X_a: HomogeneousMatrixType, X_b: HomogeneousMatrixType) -> float:
    return 1000.0 * float(np.linalg.norm(np.asarray(X_a)[:3, 3] - np.asarray(X_b)[:3, 3]))


def _orientation_error_deg(X_a: HomogeneousMatrixType, X_b: HomogeneousMatrixType) -> float:
    R = np.asarray(X_a)[:3, :3].T @ np.asarray(X_b)[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0))))


def _confirm(prompt: str) -> None:
    answer = input(f"{prompt} [type 'yes' to proceed, anything else to abort]: ").strip().lower()
    if answer != "yes":
        raise SystemExit("Aborted by user.")


def _check_frame_consistency(
    robot: Any,
    calibrated_fk: Callable[[JointConfigurationType], HomogeneousMatrixType],
    max_position_mm: float,
    max_orientation_deg: float,
    allow_mismatch: bool,
) -> None:
    """Verify our calibrated tool0 FK matches the control box's TCP pose at the current joints.

    Both should be the controller's own calibrated kinematics in the same base frame, so a
    nonneglible offset means our target poses are NOT in the frame the control box expects,
    which makes the control-box move (moveL) go somewhere unintended.
    """
    q = robot.get_joint_configuration()
    X_controlbox = robot.get_tcp_pose()  # controller frame, TCP
    X_ours = calibrated_fk(q)  # our frame, flange (tool0)
    relative = np.linalg.inv(X_ours) @ X_controlbox

    position_mm = _position_error_mm(X_ours, X_controlbox)
    orientation_deg = _orientation_error_deg(X_ours, X_controlbox)
    print("\n=== frame-consistency check (our calibrated FK vs control-box TCP, at current joints) ===")
    print(f"position offset:    {position_mm:.3f} mm")
    print(f"orientation offset: {orientation_deg:.3f} deg")
    print(f"relative transform (our_tool0 -> control_box_tcp):\n{np.round(relative, 4)}")

    if position_mm <= max_position_mm and orientation_deg <= max_orientation_deg:
        print("OK: frames agree; targets are in the control box's frame.")
        return

    print(
        "\nMISMATCH: our target poses are not in the frame the control box uses, so the "
        "control-box move would go to the wrong place (a big, unexpected motion).\n"
        "  - offset is ~pure translation along the tool axis -> nonzero TCP/payload offset on "
        "the pendant (our IK targets the FLANGE); zero it, or fold it into the target.\n"
        "  - offset is a rotation (e.g. ~180 deg) -> base/feature-frame mismatch.\n"
        "Fix this before trusting the comparison."
    )
    if not allow_mismatch:
        raise SystemExit("Aborting before any motion. Re-run with --allow-frame-mismatch to override.")
    print("Continuing anyway (--allow-frame-mismatch).")


def _warn_if_left_branch(refine_result: KinematicsResult) -> None:
    """If the refine left the analytic branch (soft cost lost the seed), warn and confirm."""
    if refine_result.is_close_to_seed:
        return
    print(
        f"WARNING: refine left the analytic branch (max {np.rad2deg(refine_result.max_seed_deviation):.1f} deg "
        "from seed) -- it may be a different configuration than intended. Inspect the MeshCat preview carefully."
    )
    _confirm("Continue with this refined configuration despite the branch deviation?")


def _build_meshcat_scene(model: str) -> tuple[RobotDiagram, Context, ModelInstanceIndex, Meshcat]:
    """Nominal airo_models mesh model + MeshCat, for previewing joint configurations."""
    from pydrake.planning import RobotDiagramBuilder

    builder = RobotDiagramBuilder()
    meshcat = add_meshcat(builder)
    arm_index, _ = add_manipulator(builder, model, "robotiq_2f_85", static_gripper=True)
    robot_diagram, context = finish_build(builder, meshcat)
    return robot_diagram, context, arm_index, meshcat


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", required=True, help="robot IP address (robot must be in remote control)")
    parser.add_argument(
        "--model",
        default="ur5e",
        help="ur3e / ur5e / ... (must match the connected arm)",
    )
    parser.add_argument("--joint-speed", type=float, default=0.3, help="moveJ speed [rad/s]")
    parser.add_argument("--linear-speed", type=float, default=0.1, help="control-box moveL speed [m/s]")
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=8.0,
        help="magnitude of the random joint offset from the current pose used to define the target",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-meshcat", action="store_true", help="skip the MeshCat preview steps")
    parser.add_argument(
        "--allow-frame-mismatch",
        action="store_true",
        help="continue even if the startup frame-consistency check fails (default: abort)",
    )
    parser.add_argument(
        "--max-frame-offset-mm",
        type=float,
        default=1.0,
        help="frame-check position tolerance [mm] (our calibrated FK vs control-box TCP)",
    )
    parser.add_argument(
        "--max-frame-offset-deg",
        type=float,
        default=0.5,
        help="frame-check orientation tolerance [deg]",
    )
    args = parser.parse_args()

    import ur_analytic_ik
    from airo_robots.manipulators.hardware.ur_rtde import URrtde

    analytic_model = getattr(ur_analytic_ik, args.model)
    rng = np.random.default_rng(args.seed)

    print(f"Connecting to {args.model} at {args.ip} ...")
    robot = URrtde(ip_address=args.ip)
    q_current = robot.get_joint_configuration()
    print(f"current joints (deg): {np.round(np.rad2deg(q_current), 2)}")

    print("Reading the robot's calibrated DH parameters (primary interface, port 30001) ...")
    dh = read_calibrated_dh(args.ip)
    print(f"calibration_status = {dh.get('calibration_status')}")
    calibrated_kinematics = CalibratedKinematics(dh, "calibrated", analytic_ik_model=analytic_model)

    # Verify our target frame matches the control box's BEFORE defining a target or moving.
    _check_frame_consistency(
        robot,
        calibrated_kinematics.forward_kinematics,
        args.max_frame_offset_mm,
        args.max_frame_offset_deg,
        args.allow_frame_mismatch,
    )

    # Define a reachable target near the current pose (small, safe motion) via calibrated FK.
    q_target_truth = q_current + np.deg2rad(args.delta_deg) * rng.uniform(-1, 1, size=6)
    X_target = calibrated_kinematics.forward_kinematics(q_target_truth)
    print(f"\ntarget TCP position (m): {np.round(X_target[:3, 3], 4)}")

    meshcat = None
    if not args.no_meshcat:
        viz_diagram, viz_context, viz_arm_index, meshcat = _build_meshcat_scene(args.model)
        viz_plant = viz_diagram.plant()
        viz_plant_context = viz_plant.GetMyContextFromRoot(viz_context)

        def preview(q: JointConfigurationType) -> None:
            viz_plant.SetPositions(viz_plant_context, viz_arm_index, np.asarray(q, dtype=float))
            viz_diagram.ForcedPublish(viz_context)

        print("\nMeshCat is running -- open the printed URL above to preview configurations.")
    else:

        def preview(q: JointConfigurationType) -> None:
            return None

    results = {}

    def move_and_measure(label: str, q_goal: JointConfigurationType) -> None:
        delta_deg = np.rad2deg(np.asarray(q_goal) - robot.get_joint_configuration())
        print(f"\n[{label}] target joints (deg): {np.round(np.rad2deg(q_goal), 2)}")
        print(
            f"[{label}] joint delta from current (deg): {np.round(delta_deg, 2)}  (max {np.abs(delta_deg).max():.1f})"
        )
        preview(q_goal)
        if meshcat is not None:
            _confirm(f"[{label}] Previewed in MeshCat. Move the REAL robot there?")
        else:
            _confirm(f"[{label}] Move the REAL robot there?")
        robot.move_to_joint_configuration(q_goal, joint_speed=args.joint_speed).wait()
        X_achieved = robot.get_tcp_pose()
        pos = _position_error_mm(X_achieved, X_target)
        ori = _orientation_error_deg(X_achieved, X_target)
        print(f"[{label}] achieved TCP error: {pos:.3f} mm, {ori:.3f} deg")
        results[label] = (pos, ori)

    # --- Stage 1: analytic IK (nominal DH) ---
    print("\n=== Stage 1: analytic IK (nominal DH) ===")
    analytic_solutions = analytic_model.inverse_kinematics_closest(X_target, *robot.get_joint_configuration())
    if not analytic_solutions:
        raise SystemExit("Analytic IK found no solution for this target; try a different --seed or --delta-deg.")
    q_analytic = np.asarray(analytic_solutions[0], dtype=float).reshape(-1)
    move_and_measure("analytic", q_analytic)

    # --- Stage 2: calibrated refine (seeded on the analytic branch) ---
    print("\n=== Stage 2: calibrated refine (two-stage IK) ===")
    refine_result = calibrated_kinematics.inverse_kinematics_closest(X_target, q_seed=robot.get_joint_configuration())
    if refine_result is None:
        raise SystemExit("Calibrated refine did not converge for this target.")
    q_refined = refine_result.joint_configuration
    print(f"refine nudged the analytic joints by (deg): {np.round(np.rad2deg(q_refined - q_analytic), 3)}")
    _warn_if_left_branch(refine_result)
    move_and_measure("refined", q_refined)

    # --- Stage 3: control-box move (the control box does its own calibrated IK) ---
    print("\n=== Stage 3: control-box move (moveL to the target pose) ===")
    _confirm("Let the control box move linearly to the target pose (its own calibrated IK)?")
    robot.move_linear_to_tcp_pose(X_target, linear_speed=args.linear_speed).wait()
    X_achieved = robot.get_tcp_pose()
    q_control_box = robot.get_joint_configuration()
    results["control_box"] = (
        _position_error_mm(X_achieved, X_target),
        _orientation_error_deg(X_achieved, X_target),
    )
    print(f"[control_box] achieved TCP error: {results['control_box'][0]:.3f} mm, {results['control_box'][1]:.3f} deg")
    print(f"[control_box] joints chosen (deg): {np.round(np.rad2deg(q_control_box), 2)}")

    # --- Summary ---
    print("\n=== SUMMARY (achieved TCP error vs target) ===")
    print(f"{'strategy':14s} {'pos [mm]':>10s} {'ori [deg]':>10s}")
    for label in ("analytic", "refined", "control_box"):
        pos, ori = results[label]
        print(f"{label:14s} {pos:10.3f} {ori:10.3f}")
    print("\nExpected: analytic ~1-2 mm; refined ~ control_box (sub-mm).")


if __name__ == "__main__":
    main()
