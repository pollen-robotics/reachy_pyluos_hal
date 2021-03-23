# Python Package for low-level serial communication with Reachy 2021 hardware

This Python package is responsible for communicating with Reachy 2021 hardware. 

More specifically, it communicates with our custom Luos board. It gather and handles all sensors updates (motors, force sensor, etc) and allows for sending commands.

## Installation

The package is a pure Python package and can be easily installed via PyPi or from the source. It requires Python >= 3.6.

## Usage

While the code is documented, it is mainly intended to be used by our ROS2 Foxy packages. It is not necessarly easy to used as a standalone.

*Note: This package will not work with standard Luos Gate but only with our custom version. See our fork here: https://github.com/pollen-robotics/luos-modules*

---

This package is part of the ROS2-based software release of the version 2021 of Reachy.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or visit [our forum](https://forum.pollen-robotics.com) if you have any questions.
