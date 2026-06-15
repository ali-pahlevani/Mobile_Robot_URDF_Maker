"""Save / load wizard configurations (presets).

A preset is a small YAML file capturing everything needed to reproduce a
configuration: the robot type, the controller type, and every parameter the
user entered. This lets a configuration be reopened and tweaked later instead
of being lost the moment the URDF is generated.
"""

import os
import logging
import yaml

logger = logging.getLogger(__name__)

PRESET_VERSION = 1

VALID_ROBOT_TYPES = {"4_wheeled", "3_wheeled", "2_wheeled_caster"}
VALID_CONTROLLERS = {
    "4_wheeled": {"diff_4w", "mecanum", "ackermann"},
    "3_wheeled": {"tricycle", "triSteer"},
    "2_wheeled_caster": {"diff_2wc"},
}

# Default location for user presets.
DEFAULT_PRESET_DIR = os.path.join(os.path.expanduser("~"), "mobRobURDF_presets")


def default_preset_dir():
    os.makedirs(DEFAULT_PRESET_DIR, exist_ok=True)
    return DEFAULT_PRESET_DIR


def save_preset(path, robot_type, controller_type, params):
    """Write a preset YAML. Raises on I/O error."""
    data = {
        "format": "mobRobURDF_preset",
        "version": PRESET_VERSION,
        "robot_type": robot_type,
        "controller_type": controller_type,
        "parameters": dict(params),
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    logger.debug("Preset saved to %s", path)


def load_preset(path):
    """Read and validate a preset YAML.

    Returns (robot_type, controller_type, parameters).
    Raises ValueError if the file is malformed or inconsistent.
    """
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("Preset file is empty or not a valid YAML mapping.")

    robot_type = data.get("robot_type")
    controller_type = data.get("controller_type")
    params = data.get("parameters", {})

    if robot_type not in VALID_ROBOT_TYPES:
        raise ValueError(f"Unknown robot_type: {robot_type!r}")
    if controller_type not in VALID_CONTROLLERS.get(robot_type, set()):
        raise ValueError(
            f"Controller {controller_type!r} is not valid for robot type {robot_type!r}"
        )
    if not isinstance(params, dict):
        raise ValueError("'parameters' must be a mapping.")

    # Coerce every value to a string (the UI line edits operate on strings).
    params = {str(k): str(v) for k, v in params.items()}
    return robot_type, controller_type, params
