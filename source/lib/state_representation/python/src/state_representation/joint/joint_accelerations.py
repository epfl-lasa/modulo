#!/usr/bin/python
from state_representation.joint.joint_state import JointState
import numpy as np
import math
import copy

class JointAccelerations(JointState):
    def __init__(self, robot_name, nb_joints=0, joint_names=[], accelerations=np.array([])):
        super().__init__(robot_name, nb_joints=nb_joints, joint_names=joint_names, accelerations=accelerations)

    def __iadd__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.accelerations = self.accelerations + other
            return self
        else:
            return super().__iadd__(other)

    def __isub__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.accelerations = self.accelerations - other
            return self
        else:
            return super().__iadd__(other)

    def __repr__(self):
        return "JointAccelerations(%r, joint_names=%r, accelerations=%r)" % (self.name, self.names, self.accelerations)

    def __str__(self):
        if self.is_empty():
            res = "Empty " + self.name + " JointAccelerations"
        else:
            res = self.name + " JointAccelerations\n"
            res += "names: [" + " ".join(self.names) + "]\n"
            res += "accelerations: [" + " ".join([str(x) for x in self.accelerations]) + "]\n"
        return res
