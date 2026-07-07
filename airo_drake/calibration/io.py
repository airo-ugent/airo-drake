"""Save/load calibrated DH parameters, for versioning and offline reruns.

Handles two json layouts:
(a) flat:   {"theta": [...], "a": [...], "d": [...], "alpha": [...]}
            (as returned by `extraction.read_calibrated_dh`)
(b) nested: {"robots": {<serial>: {"published_dh_parameters": {"joints": [...]},
                                    "calibration": {"delta_a": [...], ...}}}}
            (e.g. a `robot_calibration.json` produced by other UR calibration tooling)

For (b), the corrected DH is `published + delta` (UR convention).
"""

import json

_DH_KEYS = ("theta", "a", "d", "alpha")


def save_calibrated_dh(dh: dict, path: str) -> None:
    """Dump a DH dict (as returned by `extraction.read_calibrated_dh`) to json."""
    with open(path, "w") as file:
        json.dump(dh, file, indent=2)


def _load_nested(obj: dict, want_model: str | None = None, serial: str | None = None) -> tuple[dict, dict]:
    robots = obj["robots"]
    if serial is not None:
        entry = robots[serial]
    elif len(robots) == 1:
        entry = next(iter(robots.values()))
    else:
        matches = [
            robot for robot in robots.values() if want_model and robot.get("model", "").lower() == want_model.lower()
        ]
        if len(matches) != 1:
            raise ValueError(f"{len(robots)} robots in file; pass serial= to choose (have: {list(robots)})")
        entry = matches[0]

    joints = sorted(entry["published_dh_parameters"]["joints"], key=lambda joint: joint["joint"])
    published = {key: [joint[key] for joint in joints] for key in _DH_KEYS}
    calibration = entry.get("calibration", {}) or {}
    deltas = {key: list(calibration.get(f"delta_{key}", [0.0] * 6)) for key in _DH_KEYS}
    dh = {key: [published[key][i] + deltas[key][i] for i in range(6)] for key in _DH_KEYS}
    meta = {
        "model": entry.get("model"),
        "serial": entry.get("serial_number"),
        "published": published,
        "deltas": deltas,
    }
    return dh, meta


def load_calibrated_dh(path: str, want_model: str | None = None, serial: str | None = None) -> tuple[dict, dict]:
    """Load a DH dict from either the flat or nested json layout.

    Args:
        path: path to the json file.
        want_model: for nested files with multiple robots, select by model name
            (e.g. "ur5e") when there's no ambiguity.
        serial: for nested files with multiple robots, select by serial number.

    Returns:
        A tuple `(dh, meta)`. `meta` is `{}` for flat files, and otherwise contains
        "model", "serial", "published" and "deltas" (see `warn_if_uncalibrated`).
    """
    with open(path) as file:
        obj = json.load(file)
    if "robots" in obj:
        return _load_nested(obj, want_model, serial)
    if all(key in obj for key in _DH_KEYS):
        return {key: list(obj[key]) for key in _DH_KEYS}, {}
    raise ValueError("unrecognized DH json format")


def warn_if_uncalibrated(meta: dict) -> bool:
    """Print a warning if a nested file's calibration deltas are all (near) zero.

    Returns True if a warning was printed, False otherwise (including for flat
    files, which have no `meta` to judge).
    """
    deltas = meta.get("deltas")
    if not deltas:
        return False
    all_zero = all(abs(value) < 1e-12 for values in deltas.values() for value in values)
    if all_zero:
        print(
            "WARNING: all calibration deltas are ZERO -- this file carries NO calibration.\n"
            "The 'calibrated' model equals the nominal one; the ~1-2 mm gap will remain.\n"
            "Re-extract the DH from the robot (extraction.read_calibrated_dh)."
        )
    return all_zero
