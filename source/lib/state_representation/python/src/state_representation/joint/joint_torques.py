#!/usr/bin/python
from state_representation.joint.joint_state import JointState
import numpy as np
import math
import copy

class JointTorques(JointState):
    def __init__(self, robot_name, nb_joints=0, joint_names=[], torques=np.array([])):
        super().__init__(robot_name, nb_joints=nb_joints, joint_names=joint_names, torques=torques)

    def __iadd__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.torques = self.torques + other
            return self
        else:
            return super().__iadd__(other)

    def __isub__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.torques = self.torques - other
            return self
        else:
            return super().__iadd__(other)

    def __repr__(self):
        return "JointTorques(%r, joint_names=%r, torques=%r)" % (self.name, self.names, self.torques)

    def __str__(self):
        if self.is_empty():
            res = "Empty " + self.name + " JointTorques"
        else:
            res = self.name + " JointTorques\n"
            res += "names: [" + " ".join(self.names) + "]\n"
            res += "torques: [" + " ".join([str(x) for x in self.torques]) + "]\n"
        return res
