import time
import numpy as np

from threading import Thread

from reachy_pyluos_hal.joint_hal import JointLuos

running = [True]

def pub_joint(robot_hardware):
    joint_names = robot_hardware.get_all_joint_names()

    while running[0]:
        positions = robot_hardware.get_joint_positions(joint_names)
        velocities = robot_hardware.get_joint_velocities(joint_names)
        efforts = robot_hardware.get_joint_efforts(joint_names)

        # print(positions, velocities, efforts)

        time.sleep(1 / 100)

def pub_temp(robot_hardware):
    joint_names = robot_hardware.get_all_joint_names()

    while running[0]:
        temps = robot_hardware.get_joint_temperatures(joint_names)
        # print(temps)

        time.sleep(1 / 0.1)

def pub_fan(robot_hardware):
    fan_names = robot_hardware.get_all_fan_names()

    while running[0]:
        fans = robot_hardware.get_fans_state(fan_names)
        # print(fans)

        time.sleep(1 / 0.1)

def pub_force(robot_hardware):
    force_sensor_names = robot_hardware.get_all_force_sensor_names()

    while running[0]:
        forces = robot_hardware.get_force(force_sensor_names)
        # print(forces)

        time.sleep(1 / 10)


def get_full_state(robot_hardware):
    print('in')
    joint_names = robot_hardware.get_all_joint_names()

    positions = robot_hardware.get_joint_positions(joint_names)
    velocities = robot_hardware.get_joint_velocities(joint_names)
    efforts = robot_hardware.get_joint_efforts(joint_names)
    temperature = [float(temp) for temp in robot_hardware.get_joint_temperatures(joint_names)]
    compliant = robot_hardware.get_compliant(joint_names)
    goal_position = robot_hardware.get_goal_positions(joint_names)
    speed_limit = robot_hardware.get_goal_velocities(joint_names)
    torque_limit = robot_hardware.get_goal_efforts(joint_names)
    pid_gain = robot_hardware.get_joint_pids(joint_names)

    # print(positions, velocities, efforts, temperature, compliant, goal_position, speed_limit, torque_limit, pid_gain)
    print('out')

def random_call(robot_hardware, f):
    while running[0]:
        f(robot_hardware)
        time.sleep(np.random.rand())


def main(logger):
    running[0] = True

    robot_hardware = JointLuos('custom_kit', logger=logger)
    robot_hardware.__enter__()

    print(robot_hardware.get_all_fan_names())
    print(robot_hardware.get_all_force_sensor_names())
    print(robot_hardware.get_all_joint_names())

    tjoint = Thread(target=lambda: pub_joint(robot_hardware))
    tjoint.daemon = True
    tjoint.start()

    ttemp = Thread(target=lambda: pub_temp(robot_hardware))
    ttemp.daemon = True
    ttemp.start()

    tfan = Thread(target=lambda: pub_fan(robot_hardware))
    tfan.daemon = True
    tfan.start()

    tforce = Thread(target=lambda: pub_force(robot_hardware))
    tforce.daemon = True
    tforce.start()

    tfull = Thread(target=lambda: random_call(robot_hardware, get_full_state))
    tfull.daemon = True
    tfull.start()

    time.sleep(30 + np.random.rand() * 30)

    running[0] = False
    for t in tjoint, ttemp, tfan, tforce:
        t.join()

    robot_hardware.stop()


if __name__ == '__main__':
    import logging

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('STRESS')

    while True:
        print('START')
        main(logger)
        print('STOP')

