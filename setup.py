#!/usr/bin/env python
"""Setup configuration file."""

from setuptools import find_packages, setup


setup(
    name='reachy_pyluos_hal',
    version='0.3.1',
    packages=find_packages(exclude=['tests']),

    install_requires=[
        'numpy',
        'pyquaternion',
        'pyserial',
        'PyYAML',
        'scipy',
        'sklearn',
    ],

    package_data={'': ['config/*.yaml']},

    entry_points={
        'console_scripts': [
            'dynamixel-config=reachy_pyluos_hal.tools.dynamixel_config:main',
            'reachy-dynamixel-config=reachy_pyluos_hal.tools.reachy_dynamixel_config:main',
            'reachy-identify-model=reachy_pyluos_hal.tools.reachy_identify_model:main',
            'reachy-identify-zuuu-model=reachy_pyluos_hal.tools.reachy_identify_model:zuuu_config',
        ],
    },

    author='Pollen Robotics',
    author_email='contact@pollen-robotics.com',
    url='https://github.com/pollen-robotics/reachy_pyluos_hal',

    description='Reachy hardware library controller',
)
