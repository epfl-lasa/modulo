#!/usr/bin/python
from state_representation.state import State
import numpy as np
import math
import copy

class JointState(State):
    def __init__(self, robot_name, nb_joints=0, joint_names=[], positions=np.array([]), velocities=np.array([]), accelerations=np.array([]), torques=np.array([])):
        super().__init__("JointState", robot_name)
        if not joint_names:
            self._names = ["joint" + str(i) for i in range(nb_joints)]
        else:
            self._names = joint_names
        self.initialize()
        self._positions = positions
        self._velocities = velocities
        self._accelerations = accelerations
        self._torques = torques

    def initialize(self):
        super().initialize()
        size = len(self._names)
        self._positions = np.zeros((size,1))
        self._velocities = np.zeros((size,1))
        self._accelerations = np.zeros((size,1))
        self._torques = np.zeros((size,1))

    @property
    def names(self):
        return self._names

    @names.setter
    def names(self, values):
        if type(values) == 'int':
            self._names = ["joint" + str(i) for i in range(values)]
        else:
            self._names = values

    @property
    def positions(self):
        return self._positions

    @positions.setter
    def positions(self, values):
        self.set_filled();
        self._positions = np.array([math.atan2(math.sin(x), math.cos(x)) for x in values])

    @property
    def velocities(self):
        return self._velocities

    @velocities.setter
    def velocities(self, values):
        self.set_filled();
        self._velocities = np.array(values)

    @property
    def accelerations(self):
        return self._accelerations

    @accelerations.setter
    def accelerations(self, values):
        self.set_filled();
        self._accelerations = np.array(values)

    @property
    def torques(self):
        return self._torques

    @torques.setter
    def torques(self, values):
        self.set_filled();
        self._torques = np.array(values)

    def __iadd__(self, other):
        self.positions = self.positions + other.positions
        self.velocities = self.velocities + other.velocities
        self.accelerations = self.accelerations + other.accelerations
        self.torques = self.torques + other.torques
        return self

    def __add__(self, other):
        res = copy.deepcopy(self)
        res += other
        return res

    def __isub__(self, other):
        self.positions = self.positions - other.positions
        self.velocities = self.velocities - other.velocities
        self.accelerations = self.accelerations - other.accelerations
        self.torques = self.torques - other.torques
        return self

    def __sub__(self, other):
        res = copy.deepcopy(self)
        res -= other
        return res

    def __imul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            self.positions = self.positions * other
            self.velocities = self.velocities * other
            self.accelerations = self.accelerations * other
            self.torques = self.torques * other
        elif isinstance(other, np.ndarray):
            self.positions = np.multiply(other, self.positions)
            self.velocities = np.multiply(other, self.velocities)
            self.accelerations = np.multiply(other, self.accelerations)
            self.torques = np.multiply(other, self.torques)
        return self
        #else:
            # throw exception

    def __mul__(self, other):
        res = copy.deepcopy(self)
        res *= other
        return res

    def __rmul__(self, other):
        return self.__mul__(other)

    def __repr__(self):
        return "JointState(%r, joint_names=%r, positions=%r, velocities=%r, accelerations=%r, torques=%r)" % (self.name, self.names, self.positions, self.velocities, self.accelerations, self.torques)

    def __str__(self):
        if self.is_empty():
            res = "Empty " + self.name + " JointState"
        else:
            res = self.name + " JointState\n"
            res += "names: " + self.names + "\n"
            res += "positions: " + self.positions + "\n"
            res += "velocities: " + self.velocities + "\n"
            res += "accelerations: " + self.accelerations + "\n"
            res += "torques: " + self.torques + "\n"
        return res