"""Module responsible for loading and parsing config files."""
from pathlib import Path
from typing import Any, Dict, List, Tuple, Union

import yaml

from .device import Device
from .dynamixel import DynamixelMotor, MX106, MX64, MX28, AX18, XL320
from .fan import DxlFan, Fan, OrbitaFan
from .force_sensor import ForceSensor
from .orbita import OrbitaActuator


def load_config(config_name: str) -> List[Dict[str, Device]]:
    """Load and parse part config files corresponding to config and returns all devices in it."""
    configs = {
        'full_kit': ['left_arm', 'right_arm', 'head'],
        'full_kit_left_advanced': ['left_arm_advanced', 'right_arm', 'head'],
        'full_kit_right_advanced': ['left_arm', 'right_arm_advanced', 'head'],
        'full_kit_full_advanced': ['left_arm_advanced', 'right_arm_advanced', 'head'],

        'starter_kit_left': ['left_arm', 'head'],
        'starter_kit_left_advanced': ['left_arm_advanced', 'head'],
        'starter_kit_right': ['right_arm', 'head'],
        'starter_kit_right_advanced': ['right_arm_advanced', 'head'],

        'robotic_arm_left': ['left_arm'],
        'robotic_arm_left_advanced': ['left_arm_advanced'],
        'robotic_arm_right': ['right_arm'],
        'robotic_arm_right_advanced': ['right_arm_advanced'],
    }

    devices = []

    try:
        config = configs[config_name]
    except KeyError:
        raise KeyError(f'{config_name} should be one of {list(configs.keys())}')

    for part_name in config:
        filename = get_part_config_file(part_name)
        with open(filename) as f:
            part_conf = yaml.load(f, Loader=yaml.FullLoader)

        for part, config in part_conf.items():
            joints = joints_from_config(config)
            fans = fans_from_config(config, joints)
            sensors = sensors_from_config(config)

            part_devices: Dict[str, Device] = {}
            part_devices.update(joints)
            part_devices.update(fans)
            part_devices.update(sensors)

            devices.append(part_devices)

    return devices


def get_part_config_file(part_name: str) -> Path:
    """Find the configuration file for the given robot part."""
    import reachy_pyluos_hal
    return Path(reachy_pyluos_hal.__file__).parent / 'config' / f'{part_name}.yaml'


def joints_from_config(config: Dict[str, Dict[str, Dict[str, Any]]]) -> Dict[str, Union[DynamixelMotor, OrbitaActuator]]:
    """Create the joints described by the config."""
    joints: Dict[str, Union[DynamixelMotor, OrbitaActuator]] = {}

    for dev_name, dev_conf in config.items():
        dev_type, dev_conf = next(iter(dev_conf.items()))
        if dev_type == 'dxl_motor':
            joints[dev_name] = dxl_from_config(dev_conf)
        elif dev_type == 'orbita_actuator':
            joints[dev_name] = orbita_from_config(dev_conf)

    return joints


def fans_from_config(config: Dict[str, Dict[str, Dict[str, Any]]],
                     joints: Dict[str, Union[DynamixelMotor, OrbitaActuator]],
                     ) -> Dict[str, Fan]:
    """Create the fans described by the config."""
    def find_associated_joint(id: int) -> Tuple[str, Union[DynamixelMotor, OrbitaActuator]]:
        for name, joint in joints.items():
            if joint.id == id:
                return name, joint
        else:
            raise KeyError

    fans: Dict[str, Fan] = {}

    for dev_name, dev_conf in config.items():
        dev_type, dev_conf = next(iter(dev_conf.items()))
        if dev_type == 'fan':
            if not isinstance(dev_conf['id'], int):
                raise ValueError(f'Id should be an int({config})!')
            joint_name, joint = find_associated_joint(dev_conf['id'])
            if isinstance(joint, DynamixelMotor):
                fans[dev_name] = DxlFan(id=joint.id)
            elif isinstance(joint, OrbitaActuator):
                fans[dev_name] = OrbitaFan(id=joint.id, orbita=joint_name)

    return fans


def sensors_from_config(config: Dict[str, Dict[str, Dict[str, Any]]]) -> Dict[str, ForceSensor]:
    """Create the sensors described by the config."""
    sensors: Dict[str, ForceSensor] = {}
    for dev_name, dev_conf in config.items():
        dev_type, dev_conf = next(iter(dev_conf.items()))
        if dev_type == 'force_sensor':
            if not isinstance(dev_conf['id'], int):
                raise ValueError(f'Id should be an int({config})!')
            sensors[dev_name] = ForceSensor(id=dev_conf['id'])

    return sensors


def dxl_from_config(config: Dict[str, Any]) -> DynamixelMotor:
    """Create the specific DynamixelMotor described by the config."""
    return {
        'AX-18': AX18,
        'MX-28': MX28,
        'MX-64': MX64,
        'MX-106': MX106,
        'XL-320': XL320,
    }[config['type']](
        id=config['id'],
        offset=config.get('offset', 0.0),
        direct=config.get('direct', True),
        cw_angle_limit=config.get('cw_angle_limit', -3.14),
        ccw_angle_limit=config.get('ccw_angle_limit', 3.14),
        reduction=config.get('reduction', 1),
    )


def orbita_from_config(config: Dict[str, Any]) -> OrbitaActuator:
    """Create the specific OrbitaActuator described by the config."""
    return OrbitaActuator(id=config['id'])
