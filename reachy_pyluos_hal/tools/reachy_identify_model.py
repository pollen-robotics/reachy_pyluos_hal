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

import yaml

DEFAULT_MODEL = 'full_kit'


def print_model_and_leave(model: str):
    """Print the model found on stdout and exit."""
    print(model)
    sys.exit(0)


def main():
    """Run model identification checks."""
    model = os.getenv('REACHY_MODEL')
    if model is not None:
        print_model_and_leave(model)

    config_file = os.getenv('REACHY_CONFIG_FILE', default=os.path.expanduser('~/.reachy.yaml'))
    if not os.path.exists(config_file):
        print_model_and_leave(DEFAULT_MODEL)

    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        if 'model' not in config:
            print_model_and_leave(DEFAULT_MODEL)

        model = config['model']
        print_model_and_leave(model)


if __name__ == '__main__':
    main()
