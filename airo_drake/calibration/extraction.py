"""Read a UR robot's calibrated DH parameters off its primary interface.

The controller broadcasts a KinematicsInfo sub-package (type 5,
fixed 220-byte payload) on TCP port 30001. We connect, accumulate the stream, and
lock onto that package by its signature (big-endian length==225, type==5).

Payload layout (big-endian), per the UR client library:
    checksum     : uint32 x6
    dh_theta     : double x6
    dh_a         : double x6
    dh_d         : double x6
    dh_alpha     : double x6
    calib_status : uint32 x1
"""

import math
import socket
import struct

_KINEMATICS_INFO_TYPE = 5
_PAYLOAD_LEN = 220  # 6*4 + 4*(6*8) + 4
_SUBPACKAGE_LEN = _PAYLOAD_LEN + 5  # + int32 length + uint8 type = 225


def _parse_payload(payload: bytes) -> dict:
    checksum = struct.unpack(">6I", payload[0:24])
    theta = struct.unpack(">6d", payload[24:72])
    a = struct.unpack(">6d", payload[72:120])
    d = struct.unpack(">6d", payload[120:168])
    alpha = struct.unpack(">6d", payload[168:216])
    status = struct.unpack(">I", payload[216:220])[0]
    return {
        "theta": list(theta),
        "a": list(a),
        "d": list(d),
        "alpha": list(alpha),
        "checksum": list(checksum),
        "calibration_status": status,
    }


def _looks_sane(dh: dict) -> bool:
    values = dh["a"] + dh["d"] + dh["alpha"] + dh["theta"]
    return all(math.isfinite(value) and abs(value) < 1e4 for value in values)


def _scan(buffer: bytes) -> dict | None:
    """Find a valid KinematicsInfo sub-package anywhere in buffer."""
    for offset in range(0, len(buffer) - _SUBPACKAGE_LEN + 1):
        length, package_type = struct.unpack(">iB", buffer[offset : offset + 5])
        if length == _SUBPACKAGE_LEN and package_type == _KINEMATICS_INFO_TYPE:
            dh = _parse_payload(buffer[offset + 5 : offset + _SUBPACKAGE_LEN])
            if _looks_sane(dh):
                return dh
    return None


def read_calibrated_dh(
    robot_ip: str, port: int = 30001, timeout: float = 10.0, max_bytes: int = 5_000_000
) -> dict:
    """Connect to a UR robot's primary interface and return its calibrated DH parameters.

    The robot only needs to be powered on.
    Works for any UR e-series arm: the kinematics come from the robot itself, so the
    same function handles ur3e, ur5e, etc.

    Args:
        robot_ip: IP address of the UR control box.
        port: Primary interface port, 30001 by default.
        timeout: Socket timeout in seconds, both for connecting and for waiting on data.
        max_bytes: Safety cap on how much of the stream to buffer while scanning.

    Returns:
        A dict with keys "theta", "a", "d", "alpha" (each a list of 6 floats) and
        "calibration_status".

    Raises:
        RuntimeError: if no KinematicsInfo package arrives within the timeout.
    """
    sock = socket.create_connection((robot_ip, port), timeout=timeout)
    sock.settimeout(timeout)
    buffer = b""
    try:
        while len(buffer) < max_bytes:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            buffer += chunk
            dh = _scan(buffer)
            if dh is not None:
                return dh
    finally:
        sock.close()
    raise RuntimeError(
        f"No KinematicsInfo received from {robot_ip}:{port} within {timeout}s "
        f"({len(buffer)} bytes read). Is the robot powered on and reachable?"
    )
