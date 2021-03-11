"""Command line utility tool to configure Dynamixel motor using Reachy specific Luos Module."""

import argparse
import logging
import struct
import sys
import time
from typing import Optional

from serial import Serial
from serial.threaded import ReaderThread


from ..discovery import identify_luos_containers
from ..dynamixel import AX18, DynamixelError, DynamixelMotor, get_motor_from_model
from ..pycore import GateProtocol


class GateHandler(GateProtocol):
    """Custom handler for dynamixel message callback."""

    dxl: Optional[DynamixelMotor] = None

    def handle_assert(self, msg):
        """Assert message reception callback."""
        raise AssertionError(msg)

    def handle_dxl_pub_data(self, register, ids, errors, data):
        """Dynamixel message reception callback."""
        register = self.dxl.find_register_by_addr(register)

        for id, val in zip(ids, data):
            if id == self.dxl.id:
                self.dxl.update_value(register, val)


def get_dxl_register(gate_protocol: GateHandler, reg: str):
    """Send a get dynamixel request and wait for the answer."""
    dxl = gate_protocol.dxl
    assert dxl is not None

    addr, num_bytes = dxl.dxl_config[reg]
    gate_protocol.send_dxl_get(addr, num_bytes, [dxl.id])
    time.sleep(0.25)

    return dxl.get_value_as_usi(dxl.find_register_by_addr(addr))


def set_dxl_register(gate_protocol: GateHandler, reg: str, value):
    """Send a set dynamixel request."""
    dxl = gate_protocol.dxl
    assert dxl is not None

    addr, num_bytes = dxl.dxl_config[reg]
    raw_value = dxl.registers[reg].cvt_as_raw(value)

    gate_protocol.send_dxl_set(addr, num_bytes, {dxl.id: raw_value})


def get_dxl(port, dxl_alias) -> DynamixelMotor:
    """Retrieve the specific motor model given its id."""
    dxl_id = int(dxl_alias.split('_')[1])

    with Serial(port, baudrate=1000000) as s:
        with ReaderThread(s, GateHandler) as p:
            # We first retrieve the model number using any model
            # (all models share the same register for ModelNumber)
            p.dxl = AX18(dxl_id, 0, True)
            model_number = get_dxl_register(p, 'model_number')
            # Now we switch to the real model
            # to make sure all registers are correctly used.
            return get_motor_from_model(model_number)(dxl_id, 0, True)


def read_eeprom(port, dxl):
    """Read and print the EEPROM configuration of a dynamixel motor."""
    with Serial(args.port, baudrate=1000000) as s:
        with ReaderThread(s, GateHandler) as p:
            p.dxl = dxl

            print(f'Reading the EEPROM of motor {dxl.id}...')

            for reg in [
                'model_number',
                'id', 'return_delay_time',
                'cw_angle_limit', 'ccw_angle_limit',
                'temperature_limit', 'alarm_shutdown',
            ]:
                val = get_dxl_register(p, reg)
                if isinstance(val, float):
                    val = round(val, 2)
                print(f'\t{reg.capitalize()}: {val}')


def write_eeprom(port, dxl, args):
    """Write the specified values to a dynamixel motor EEPROM."""
    first_write = True

    with Serial(args.port, baudrate=1000000) as s:
        with ReaderThread(s, GateHandler) as p:
            p.dxl = dxl
            for reg in (
                'return_delay_time',
                'cw_angle_limit', 'ccw_angle_limit',
                'temperature_limit', 'alarm_shutdown',
                'id',
            ):
                val = getattr(args, reg)
                if val is not None:
                    if first_write:
                        print(f'Starting writing on {dxl.id}...')
                        first_write = False

                    print(f'\tWriting {reg} to {val}...')

                    if reg == 'alarm_shutdown':
                        val = [getattr(DynamixelError, name) for name in val]

                    set_dxl_register(p, reg, val)
                    time.sleep(0.5)

                    if reg == 'id':
                        print('\tForcing a re-detection to update the dynamixel ID list...')
                        # We use the previous id as the table as not yet been updated
                        p.send_msg(bytes([p.MSG_TYPE_DXL_DETECT, dxl.id]))
                        time.sleep(0.5)
                        p.send_msg(bytes([p.MSG_DETECTION_RUN]))
                        time.sleep(0.5)
                        # Now the id should have been updated everywhere.
                        dxl.id = args.id

        if not first_write:
            print('Writing done!')


def change_dxl_baudrate(port: str, baud: int):
    """Change the baudrate of a dynamixel motor and reset the connection to still be able to communicate with it."""
    with Serial(port, baudrate=1000000) as s:
        with ReaderThread(s, GateHandler) as p:
            msg = [p.MSG_TYPE_DXL_SET_BAUDRATE, p.DXL_BROADCAST_ID]
            msg += list(struct.pack('I', baud))
            p.send_msg(bytes(msg))
            time.sleep(1.0)
            p.send_msg(bytes([p.MSG_TYPE_DXL_DETECT, p.DXL_BROADCAST_ID]))
            time.sleep(1.0)
            p.send_msg(bytes([p.MSG_DETECTION_RUN]))
            time.sleep(1.0)


def get_dxl_motor_from_containers(containers, logger):
    """Look for a single dynamixel motor on a Luos bus."""
    if len(containers) != 2:
        logger.error('Make sure exactly only the gate, the dynamixel module and one motor are connected!')
        logger.error(f'(modules found: {containers}')
        sys.exit(1)

    gate = containers[0]
    if len(gate) != 1 or gate[0].type != 'Gate':
        logger.error('Make sure exactly only the gate, the dynamixel module and one motor are connected!')
        logger.error(f'(modules found: {containers}')
        sys.exit(1)

    dxl_motors = containers[2]
    if len(dxl_motors) != 1 or dxl_motors[0].type != 'DynamixelMotor':
        logger.error('Make sure exactly only the gate, the dynamixel module and one motor are connected!')
        logger.error(f'(modules found: {containers}')
        sys.exit(1)

    return dxl_motors[0]


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('port')
    parser.add_argument('--id', type=int)
    parser.add_argument('--return-delay-time', type=int)
    parser.add_argument('--cw-angle-limit', type=float)
    parser.add_argument('--ccw-angle-limit', type=float)
    parser.add_argument('--temperature-limit', type=int)
    parser.add_argument('--alarm-shutdown', choices=[err.name for err in DynamixelError], nargs='+')
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger()

    containers = identify_luos_containers(args.port, logger)
    dxl_motors = get_dxl_motor_from_containers(containers, logger)

    GateHandler.logger = logger

    if (dxl_motors.alias == 'void_dxl'):
        print('No motor found on baudrate 1M, trying on 57600...')
        change_dxl_baudrate(args.port, 57600)

        try:
            containers = identify_luos_containers(args.port, logger)
            dxl_motors = get_dxl_motor_from_containers(containers, logger)
            if dxl_motors.alias == 'void_dxl':
                print('No motor found! Check the connection and try again.')
                change_dxl_baudrate(args.port, 1000000)
                sys.exit(1)

            for trials in range(10):
                try:
                    dxl = get_dxl(args.port, dxl_motors.alias)
                    print(f'Found motor {dxl.id} {dxl.motor_type} on baudrate 57600. We will switch it to 1M baudrate now...')
                    with Serial(args.port, baudrate=1000000) as s:
                        with ReaderThread(s, GateHandler) as p:
                            p.dxl = dxl
                            set_dxl_register(p, 'baudrate', 1000000)
                            time.sleep(0.5)
                    break
                except ValueError:
                    if trials == 9:
                        raise

        except Exception as e:
            change_dxl_baudrate(args.port, 1000000)
            raise e

        change_dxl_baudrate(args.port, 1000000)
        print('Trying to find it on 1M to check if everything went well...')
        containers = identify_luos_containers(args.port, logger)
        dxl_motors = get_dxl_motor_from_containers(containers, logger)
        if dxl_motors.alias == 'void_dxl':
            print('No motor found! Check the connection and try again!')
            sys.exit(1)

    dxl = get_dxl(args.port, dxl_motors.alias)
    write_eeprom(args.port, dxl, args)
    read_eeprom(args.port, dxl)
