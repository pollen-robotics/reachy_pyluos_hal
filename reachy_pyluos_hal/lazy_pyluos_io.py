from logging import Logger
from typing import Dict, List, Tuple

from .joint_hal import JointLuos


class LazyPyluosIO(JointLuos):
    def __init__(self, config_name: str, logger: Logger) -> None:
        JointLuos.__init__(self, config_name, logger)

    def __enter__(self):
        JointLuos.__enter__(self)

        all_joint_names = self.get_all_joint_names()

        self._compliancies = {
            j: c
            for j, c in zip(
                all_joint_names, 
                JointLuos.get_compliant(self, all_joint_names),
            )
        }
        self._goal_pos = {
            j: c
            for j, c in zip(
                all_joint_names, 
                JointLuos.get_goal_positions(self, all_joint_names),
            )
        }
        self._pids = {
            j: pid
            for j, pid in zip(
                all_joint_names,
                JointLuos.get_joint_pids(self, all_joint_names),
            )
        }

    def get_compliant(self, names: List[str]) -> List[bool]:
        return [self._compliancies[n] for n in names]

    def set_compliance(self, compliancies: Dict[str, bool]) -> bool:
        diff_compliancies = {
            k: c
            for k, c in compliancies.items()
            if self._compliancies[k] != c
        }

        if not diff_compliancies:
            return True

        ok = JointLuos.set_compliance(self, diff_compliancies)
        if ok:
            self._compliancies.update(diff_compliancies)
        return ok

    def get_goal_positions(self):
        return [self._goal_pos[n] for n in names]

    def set_goal_positions(self, goal_positions: Dict[str, float]) -> bool:
        compliancies = self.get_compliant(goal_positions.keys())
        needed_goal_positions = {
            joint: pos
            for i, (joint, pos) in enumerate(goal_positions.items())
            if not compliancies[i]
        }

        needed_goal_positions = {
            k: pos
            for k, pos in needed_goal_positions.items()
            if self._goal_pos[k] != pos
        }

        if not needed_goal_positions:
            return True

        ok = JointLuos.set_goal_positions(self, needed_goal_positions)
        if ok:
            self._goal_pos.update(needed_goal_positions)
        return ok

    def get_joint_pids(self, names: List[str]) -> List[Tuple[float, float, float]]:
        return [self._pids[n] for n in names]

    def get_homogeneous_pids(self, names: List[str]) -> List[Tuple[float, float, float, float]]:
        return [
            tuple(pids) if len(pids) == 4 else tuple(list(pids) + [float('nan')])
            for pids in self.get_joint_pids(names)
        ]

    def set_goal_pids(self, goal_pids: Dict[str, Tuple[float, float, float]]) -> bool:
        diff_pids = {
            k: pid
            for k, pid in goal_pids.items()
            if self._pids[k] != pid
        }
        if not diff_pids:
            return True

        ok = JointLuos.set_goal_pids(self, diff_pids)

        if ok:
            self._pids.update(diff_pids)

        return ok
