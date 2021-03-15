"""Command line utility tool to configure Dynamixel motor by using the Reachy configuration file."""

from subprocess import call

from ..config import load_config
from ..dynamixel import DynamixelMotor, DynamixelError


def main():
    """Run main entry point."""
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('port')
    parser.add_argument('config_file')
    parser.add_argument('joint_name')
    parser.add_argument('--return-delay-time', type=int, default=20)
    parser.add_argument('--temperature-limit', type=int, default=55)
    parser.add_argument('--alarm-shutdown', choices=[err.name for err in DynamixelError], nargs='+', default=[DynamixelError.OverheatingError])
    args = parser.parse_args()

    config = load_config(args.config_file)
    dynamixel_motors = {}
    for part in config:
        dynamixel_motors.update({
            name: device
            for name, device in part.items()
            if isinstance(device, DynamixelMotor)
        })

    if args.joint_name not in dynamixel_motors:
        raise ValueError(f'Error: The joint name should be one of {dynamixel_motors.keys()}!')

    dxl = dynamixel_motors[args.joint_name]

    cmd = [
        'dynamixel-config',
        args.port,
        '--id', str(dxl.id),
        '--return-delay-time', str(args.return_delay_time),
        '--cw-angle-limit', str(dxl.cw_angle_limit),
        '--ccw-angle-limit', str(dxl.ccw_angle_limit),
        '--temperature-limit', str(args.temperature_limit),
        '--alarm-shutdown', ' '.join([error.name for error in args.alarm_shutdown]),
    ]
    print(f'Will now run cmd \"{" ".join(cmd)}\"')

    print(call(cmd))


if __name__ == '__main__':
    main()
