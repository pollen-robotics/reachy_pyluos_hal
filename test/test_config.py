import pytest


from reachy_pyluos_hal.config import load_config


NB_DEVICE_ARM = 12
NB_DEVICE_HEAD = 6


def test_full_kit():
    conf = load_config('full_kit')
    check_full_kit(conf)
    check_arm_normal('left', conf)
    check_arm_normal('right', conf)
    check_head(conf)


def test_full_kit_one_advanced_arm():
    for side in ('left', 'right'):
        other_side = 'right' if side == 'left' else 'left'

        conf = load_config(f'full_kit_{side}_advanced')
        check_full_kit(conf)

        check_arm_normal(other_side, conf)
        check_arm_advanced(side, conf)
        check_head(conf)


def test_full_kit_both_advanced_arms():
    conf = load_config('full_kit_full_advanced')
    check_full_kit(conf)

    check_arm_advanced('left', conf)
    check_arm_advanced('right', conf)
    check_head(conf)


def test_starter_kit():
    for side in ('left', 'right'):
        conf = load_config(f'starter_kit_{side}')
        assert len(conf) == 2
        devices = {}
        for part in conf:
            devices.update(part)
        assert len(devices) == NB_DEVICE_HEAD + NB_DEVICE_ARM
        check_head(conf)
        check_arm_normal(side, conf)    


def test_advanced_starter_kit():
    for side in ('left', 'right'):
        conf = load_config(f'starter_kit_{side}_advanced')
        assert len(conf) == 2
        devices = {}
        for part in conf:
            devices.update(part)
        assert len(devices) == NB_DEVICE_HEAD + NB_DEVICE_ARM
        check_head(conf)
        check_arm_advanced(side, conf)


def test_robotic_arm():
    for side in ('left', 'right'):
        conf = load_config(f'robotic_arm_{side}')
        assert len(conf) == 1
        devices = {}
        for part in conf:
            devices.update(part)
        assert len(devices) == NB_DEVICE_ARM
        check_arm_normal(side, conf)


def test_advanced_robotic_arm():
    for side in ('left', 'right'):
        conf = load_config(f'robotic_arm_{side}_advanced')
        assert len(conf) == 1
        devices = {}
        for part in conf:
            devices.update(part)
        assert len(devices) == NB_DEVICE_ARM
        check_arm_advanced(side, conf)


def test_wrong_config():
    with pytest.raises(KeyError):
        load_config('robotic_arm')


def check_full_kit(conf):
    assert len(conf) == 3
    devices = {}
    for part in conf:
        devices.update(part)
    assert len(devices) == 2 * NB_DEVICE_ARM + NB_DEVICE_HEAD


def check_arm_normal(side, conf):
    devices = {}
    for part in conf:
        devices.update(part)

    side_prefix = 'l' if side == 'left' else 'r'
    assert devices[f'{side_prefix}_shoulder_roll'].motor_type == 'MX64'


def check_arm_advanced(side, conf):
    devices = {}
    for part in conf:
        devices.update(part)

    side_prefix = 'l' if side == 'left' else 'r'
    assert devices[f'{side_prefix}_shoulder_roll'].motor_type == 'MX106'


def check_head(conf):
    devices = {}
    for part in conf:
        devices.update(part)

    assert 'neck' in devices
