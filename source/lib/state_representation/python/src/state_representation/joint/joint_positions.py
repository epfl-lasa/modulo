#!/usr/bin/python
from state_representation.joint.joint_state import JointState
import numpy as np
import math
import copy

class JointPositions(JointState):
    def __init__(self, robot_name, nb_joints=0, joint_names=[], positions=np.array([])):
        super().__init__(robot_name, nb_joints=nb_joints, joint_names=joint_names, positions=positions)

    def __iadd__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.positions = self.positions + other
            return self
        else:
            return super().__iadd__(other)

    def __isub__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.positions = self.positions - other
            return self
        else:
            return super().__iadd__(other)

    def __repr__(self):
        return "JointPositions(%r, joint_names=%r, positions=%r)" % (self.name, self.names, self.positions)

    def __str__(self):
        if self.is_empty():
            res = "Empty " + self.name + " JointPositions"
        else:
            res = self.name + " JointPositions\n"
            res += "names: [" + " ".join(self.names) + "]\n"
            res += "positions: [" + " ".join([str(x) for x in self.positions]) + "]\n"
        return res
