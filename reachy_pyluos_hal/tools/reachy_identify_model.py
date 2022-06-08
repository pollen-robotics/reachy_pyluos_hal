"""Utility tool to facilitate the identification of the Reachy model used by other program.

To determine the model we:
- read the environment variable REACHY_MODEL if it exists (eg. EXPORT REACHY_MODEL="starter_kit_right")
- the presence of a config file located at REACHY_CONFIG_FILE (or ~/.reachy.yaml by default)
- assume you are using a standard Reachy (both arms and head).

While you could easily check the same values in your own program,
this command line tool is mainly aiming at providing backward compatibility.

The config possibilities are:

    - 'full_kit'
    - 'full_kit_left_advanced'
    - 'full_kit_right_advanced'
    - 'full_kit_full_advanced'
    - 'starter_kit_left'
    - 'starter_kit_left_advanced'
    - 'starter_kit_right'
    - 'starter_kit_right_advanced'

    - 'robotic_arm_left'
    - 'robotic_arm_left_advanced'
    - 'robotic_arm_right'
    - 'robotic_arm_right_advanced'

"""

import os
import sys

from reachy_pyluos_hal.config import get_reachy_config


DEFAULT_MODEL = 'full_kit'
DEFAULT_ZUUU_MODEL = 'none'


def print_model_and_leave(model: str):
    """Print the model found on stdout and exit."""
    print(model)
    sys.exit(0)


def main():
    """Run model identification checks."""
    config = get_reachy_config()

    if config is not None:
        model = config['model']
        print_model_and_leave(model)

    else:
        # Kept only for compatibility, the yaml config file should be prefered!
        model = os.getenv('REACHY_MODEL')
        if model is not None:
            print_model_and_leave(model)

        else:
            print_model_and_leave(DEFAULT_MODEL)

def zuuu_config():
    """Run model identification checks on the mobile base"""
    config = get_reachy_config()

    if config is not None and 'zuuu_model' in config:
        model = config['zuuu_model']
        print_model_and_leave(model)
    else:
        print_model_and_leave(DEFAULT_ZUUU_MODEL)


if __name__ == '__main__':
    # main()
    zuuu_config()
