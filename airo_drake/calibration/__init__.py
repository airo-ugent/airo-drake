"""Generate a per-robot calibrated URDF for UR arms, and use it for accurate IK.

1. **Seed**: run analytic IK (nominal DH) to pick the joint-configuration branch
   for the target pose. It's fast and deterministic near singularities/limits,
   which numerical IK alone is not.
2. **Refine**: run Drake's numerical `InverseKinematics` on the *calibrated* URDF
   (see `calibrated_dh_to_urdf`), seeded at the analytic solution with a
   stay-near-seed cost, so it converges to the accurate joints without flipping
   branches.
"""
