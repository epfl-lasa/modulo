#!/usr/bin/python
from state_representation.joint.joint_state import JointState
import numpy as np
import math
import copy

class JointVelocities(JointState):
    def __init__(self, robot_name, nb_joints=0, joint_names=[], velocities=np.array([])):
        super().__init__(robot_name, nb_joints=nb_joints, joint_names=joint_names, velocities=velocities)

    def __iadd__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.velocities = self.velocities + other
            return self
        else:
            return super().__iadd__(other)

    def __isub__(self, other):
        if isinstance(other, np.ndarray) or isinstance(other, float) or isinstance(other, int):
            self.velocities = self.velocities - other
            return self
        else:
            return super().__iadd__(other)

    def __repr__(self):
        return "JointVelocities(%r, joint_names=%r, velocities=%r)" % (self.name, self.names, self.velocities)

    def __str__(self):
        if self.is_empty():
            res = "Empty " + self.name + " JointVelocities"
        else:
            res = self.name + " JointVelocities\n"
            res += "names: [" + " ".join(self.names) + "]\n"
            res += "velocities: [" + " ".join([str(x) for x in self.velocities]) + "]\n"
        return res
