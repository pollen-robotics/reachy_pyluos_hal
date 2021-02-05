import logging
import time

from reachy_pyluos_hal.joint_hal import JointLuos

if __name__ == '__main__':
    import sys

    logging.basicConfig(
        filename=sys.argv[1],
        level=logging.DEBUG,
        format='%(asctime)s %(levelname)s %(message)s',
    )

    hal = JointLuos(logger=logging.getLogger())
    names = hal.get_all_joint_names()

    hal.set_compliance({
        'l_antenna': False, 
        'r_antenna': True,
        'neck_disk_top': False,
        'neck_disk_middle': False,
        'neck_disk_botton': True,
    })

    hal.set_goal_positions({
        'l_antenna': 0, 
        'neck_disk_top': 0,
        'neck_disk_middle': 0,
    })

    t = []

    while True:
        p1, p2 = hal.get_joint_positions(['r_antenna', 'neck_disk_bottom'])

        hal.set_goal_positions({
            'l_antenna': p2,
            'neck_disk_top': p2,
            'neck_disk_middle': p1,
        })
        t.append(time.time())
        time.sleep(0.005)

        if t[-1] - t[0] > 60:
            break

    hal.reachy.stop()
