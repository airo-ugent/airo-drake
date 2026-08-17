"""Per-robot calibrated URDFs for UR arms, and calibrated kinematics built on them.

Every physical UR arm has its own calibrated DH parameters, differing from the nominal
DH by enough to cause ~1-2 mm TCP error. `read_calibrated_dh` reads them from the control
box, `calibrated_dh_to_urdf` turns them into a URDF, and `CalibratedKinematics` provides
accurate forward/inverse kinematics on that model.
"""
