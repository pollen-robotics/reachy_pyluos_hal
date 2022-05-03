import time
from collections import defaultdict
from logging import Logger
from threading import Lock, Thread

from .joint_hal import JointLuos


class PyluosIOController:
    def __init__(self, config_name: str, logger: Logger) -> None:
        self.io = JointLuos(config_name=config_name, logger=logger)
        self.io.__enter__()

        self.cache = defaultdict(dict)

        cache_dict = {
            'fan': (self.get_all_fan_names(), self.io.get_fans_state),
            'force': (self.get_all_force_sensor_names(), self.io.get_force),
            'goal_pos': (self.get_all_joint_names(), self.io.get_goal_positions),
            'compliant': (self.get_all_joint_names(), self.io.get_compliant),
            'speed_limit': (self.get_all_joint_names(), self.io.get_goal_velocities),
            'torque_limit': (self.get_all_joint_names(), self.io.get_goal_efforts),
            'pid': (self.get_all_joint_names(), self.io.get_joint_pids),
        }

        for label, (joints, getter) in cache_dict.items():
            self._update_cache(label, dict(zip(joints, getter(joints))))

        self._polling = True
        self._polling_lock = Lock()

        self._poll_force_t = Thread(target=self._poll_force)
        self._poll_force_t.start()
        self._poll_fan_t = Thread(target=self._poll_fan)
        self._poll_fan_t.start()

    def stop(self):
        self._polling = False
        self._poll_force_t.join()
        self._poll_fan_t.join()

        self.io.stop()

    def get_all_joint_names(self):
        return self.io.get_all_joint_names()

    def get_all_fan_names(self):
        return self.io.get_all_fan_names()

    def get_all_force_sensor_names(self):
        return self.io.get_all_force_sensor_names()

    def get_joint_positions(self, names):
        return self.io.get_joint_positions(names)

    def get_joint_temperatures(self, names):
        return self.io.get_joint_temperatures(names)

    def get_fan_states(self, names):
        return self._get_cache('fan', names)

    def set_fan_states(self, states):
        new_states = self._update_cache('fan', states)
        if new_states:
            return self.io.set_fans_state(states)    
        return True

    def get_force(self, names):
        return self._get_cache('force', names)

    def get_goal_positions(self, names):
        return self._get_cache('goal_pos', names)

    def set_goal_positions(self, goal_position):
        new_goal_pos = self._update_cache('goal_pos', goal_position)
        if new_goal_pos:
            return self.io.set_goal_positions(new_goal_pos)
        return True

    def get_compliant(self, names):
        return self._get_cache('compliant', names)

    def set_compliant(self, compliances):
        new_compliances = self._update_cache('compliant', compliances)
        if new_compliances:
            return self.io.set_compliance(new_compliances)
        return True

    def get_goal_velocities(self, names):
        return self._get_cache('speed_limit', names)

    def set_goal_velocities(self, goal_velocities):
        new_goal_velocities = self._update_cache('speed_limit', goal_velocities)
        if new_goal_velocities:
            return self.io.set_goal_velocities(new_goal_velocities)
        return True

    def get_goal_efforts(self, names):
        return self._get_cache('torque_limit', names)

    def set_goal_efforts(self, goal_efforts):
        new_goal_efforts = self._update_cache('torque_limit', goal_efforts)
        if new_goal_efforts:
            return self.io.set_goal_efforts(new_goal_efforts)
        return True

    def get_joint_pids(self, names):
        return self._get_cache('pid', names)

    def set_goal_pids(self, pids):
        new_pids = self._update_cache('pid', pids)
        if new_pids:
            return self.io.set_goal_pids(new_pids)
        return True

    def _get_cache(self, field, names):
        return [self.cache[field][n] for n in names]

    def _update_cache(self, field, values):
        new_values = {}

        for k, v in values.items():
            if k not in self.cache[field] or self.cache[field][k] != v:
                new_values[k] = v
                self.cache[field][k] = v

        return new_values

    def _poll_force(self):
        while self._polling:
            with self._polling_lock:
                self.cache['force'].update(dict(zip(
                    self.get_all_force_sensor_names(),
                    self.io.get_force(self.get_all_force_sensor_names()),
                )))

            time.sleep(0.1)

    def _poll_fan(self):
        while self._polling:
            with self._polling_lock:
                self.cache['fan'].update(dict(zip(
                    self.get_all_fan_names(),
                    self.io.get_fans_state(self.get_all_fan_names()),
                )))

            time.sleep(1)
