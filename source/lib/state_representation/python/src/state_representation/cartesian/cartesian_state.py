#!/USSR/bin/python
from state_representation.state import State
import numpy as np
import math
import copy
import warnings
import quaternion


class CartesianState(State):
    def __init__(self, *args, name=None, reference=None, state=None):
        self.dimensions = 3
        super().__init__("CartesianState", robot_name)
        
        self.initialize()

        # Decode input
        if (len(args) and isinstance(args[0], 'CartesianState')):
            state = args[0]
            
        elif (len(args) and isinstance(args[0], str)):
            name = args[0]

        
        if isinstance(state, 'CartesianState')):
            # TODO State?
            super().__init__(state)
            self.position = args[0].position
            self.linear_velocity = args[0]._linear_velocity
            self.angular_velocity = args[0]._angular_velocity
            self.linear_acceleration = args[0]._linear_acceleration
            self.angular_acceleration = args[0]._angular_acceleration
            self.force = args[0].force
            self.torque = args[0].torque
            
        elif isinstance(name, str):
            super().__init__("CartesianState", robot_name, reference)
            
        else:
            super().__init__("CartesianState")

    def initialize(self):
        super().initialize()
        self.position = np.zeros((self.dimensions,1))
        self.orientation = np.zeros((self.dimensions,1))
        self.linear_velocity = np.zeros((self.dimensions,1))
        self.angular_velocity = np.zeros((self.dimensions,1))
        self.linear_acceleration = np.zeros((self.dimensions,1))
        self.angular_acceleration = np.zeros((self.dimensions,1))
        self.force = np.zeros((self.dimensions,1))
        self.torque = np.zeros((self.dimensions, 1))
    
    def __imul__(self, other):
        if self.is_empty():
            warnings.warn("WARNING STATE IS EMPTY")
            return self

        self.positions = self.positions * other
        self.orientation = self.orientation * other # NOT MEANINGFUL
        self.linear_velocity = self.linear_velocity * other
        self.angular_velocity = self.angular_velocity * other
        self.linear_acceleration = self.linear_acceleration * other
        self.angular_acceleration = self.angular_acceleration * other
        self.force = self.force * other
        self.torque = self.torque * other
        
        return self
        #else:
            # throw exception

    def __mul__(self, other):
        res = copy.deepcopy(self)
        res *= other
        return res

    def __rmul__(self, other):
        return self.__mul__(other)

    def __str__(self):
        if self.is_empty():
            res = "Empty CartesianState"
        else:
            res = self.name + " CartesianState\n"
            res += "positions: [" + " ".join([str(x) for x in self.positions]) + "]\n"
            res += "orientation: [" + " ".join([str(x) for x in self.orientation]) + "]\n"
            # res += ("<=> angle // axis: " + joint([str(x) for x in quaternion.as_rotation_vector(self.orientation)]) + "]\n") # TEST
            res += "linear_velocity: [" + " ".join([str(x) for x in self.linear_velocity]) + "]\n"
            res += "angular_velocity: [" + " ".join([str(x) for x in self.angular_velocity]) + "]\n"
            res += "linear_acceleration: [" + " ".join([str(x) for x in self.linear_acceleration]) + "]\n"
            res += "angular_acceleration: [" + " ".join([str(x) for x in self.angular_acceleration]) + "]\n"
            res += "force: [" + " ".join([str(x) for x in self.force]) + "]"
            res += "torque: [" + " ".join([str(x) for x in self.torque]) + "]"
        return res
