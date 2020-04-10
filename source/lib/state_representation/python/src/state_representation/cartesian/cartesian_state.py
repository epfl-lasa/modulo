#!/Ussr/bin/python
from state_representation.state import State

import numpy as np
import math
import copy
import warnings

import quaternion

class CartesianState(State):
    def __init__(self, *args, name=None, reference="world",
                 state=None):
        
        self.dimensions = 3
        
        self.initialize() # TODO move to normal init function
        # self.derivative=0

        # Decode input
        if len(args):
            if isinstance(args[0], CartesianState):
                state = args[0]
                
            elif isinstance(args[0], str):
                name = args[0]

                if len(args)==2:
                    reference = args[1]

        if isinstance(state, CartesianState):
            super().__init__(state)
            self.position = state.position
            self.linear_velocity = state.linear_velocity
            self.angular_velocity = state.angular_velocity
            self.linear_acceleration = state.linear_acceleration
            self.angular_acceleration = state.angular_acceleration
            self.force = state.force
            self.torque = state.torque
            
        else:
            super().__init__("CartesianState", name, reference)
            
    def initialize(self):
        super().initialize()
        self.position = np.zeros((self.dimensions, 1))
        self.orientation = [1, 0, 0, 0]
        self.linear_velocity = np.zeros((self.dimensions, 1))
        self.angular_velocity = np.zeros((self.dimensions, 1))
        self.linear_acceleration = np.zeros((self.dimensions, 1))
        self.angular_acceleration = np.zeros((self.dimensions, 1))
        self.force = np.zeros((self.dimensions, 1))
        self.torque = np.zeros((self.dimensions, 1))

    # MAYBE - This linear/angular property replace the rest
    @property
    def linear(self):
        raise NotImplementedError()
    
    @linear.setter
    def linear(self, value):
        raise NotImplementedError()
        
    @property
    def angular(self):
        raise NotImplementedError()

    @angular.setter
    def angular(self, value):
        raise NotImplementedError()

    @property
    def value_names(self):
       return {"linear":"linear", "angular":"angular"}
    
    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(self.dimensions,)):
            self._position = np.squeeze(value)
            self.set_filled()
        else:
            raise TypeError("Wrong input position argument -- {}".format(value))

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if (isinstance(value, (list, np.ndarray)) and np.squeeze(value).shape==(4,)):
            self._orientation = quaternion.as_quat_array(value)
        elif isinstance(value, (quaternion.quaternion)):
            self._orientation = value
        else:
            raise TypeError("Wrong input argument -- {}".format(value))

    @property
    def linear_velocity(self):
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(self.dimensions,)):
            self._linear_velocity = np.squeeze(value)
            self.set_filled()
        else:
            raise TypeError("Wrong input argument -- {}".format(value))

    @property
    def angular_velocity(self):
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(self.dimensions,)):
            self._angular_velocity = np.squeeze(value)
            self.set_filled()
        else:
            raise TypeError("Wrong input argument -- {}".format(value))

    @property
    def linear_acceleration(self):
        return self._linear_acceleration

    @linear_acceleration.setter
    def linear_acceleration(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(self.dimensions,)):
            self._linear_acceleration = np.squeeze(value)
            self.set_filled()
        else:
            raise TypeError("Wrong input argument -- {}".format(value))

    @property
    def angular_acceleration(self):
        return self._angular_acceleration

    @angular_acceleration.setter
    def angular_acceleration(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(self.dimensions,)):
            self._angular_acceleration = value
            self.set_filled()
        else:
            raise TypeError("Wrong input argument -- {}".format(value))

    @property
    def force(self):
        return self._force

    @force.setter
    def force(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(self.dimensions,)):
            self._force = value
            self.set_filled()
        else:
            raise TypeError("Wrong input argument -- {}".format(value))

    @property
    def torque(self):
        return self._torque

    @torque.setter
    def torque(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(self.dimensions,)):
            self._torque = value
            self.set_filled()
        else:
            raise TypeError("Wrong input argument -- {}".format(value))
                      
    
    def __imul__(self, other):
        if self.is_empty:
            raise RuntimeError("State is empty")

        self.position = self.position * other
        self.orientation = self.orientation * other # NOT MEANINGFUL
        self.linear_velocity = self.linear_velocity * other
        self.angular_velocity = self.angular_velocity * other
        self.linear_acceleration = self.linear_acceleration * other
        self.angular_acceleration = self.angular_acceleration * other
        self.force = self.force * other
        self.torque = self.torque * other
        
        return self
        # else:
            # throw exception

    def __mul__(self, other):
        res = copy.deepcopy(self)
        res *= other
        return res

    def __rmul__(self, other):
        return self.__mul__(other)

    def __str__(self):
        if self.is_empty:
            res = "Empty CartesianState"
        else:
            res = self.name + " CartesianState\n"
            res += "position: [" + " ".join([str(x) for x in self.position]) + "]\n"
            res += "orientation: [" + " ".join([str(x) for x in self.orientation.components]) + "]\n"
            res += ("<=> angle " + str(self.orientation.angle)
                    + "// axis: " + joint([str(x) for x in self.orientation.vec]) + "]\n") # TEST
            res += "linear_velocity: [" + " ".join([str(x) for x in self.linear_velocity]) + "]\n"
            res += "angular_velocity: [" + " ".join([str(x) for x in self.angular_velocity]) + "]\n"
            res += "linear_acceleration: [" + " ".join([str(x) for x in self.linear_acceleration]) + "]\n"
            res += "angular_acceleration: [" + " ".join([str(x) for x in self.angular_acceleration]) + "]\n"
            res += "force: [" + " ".join([str(x) for x in self.force]) + "]"
            res += "torque: [" + " ".join([str(x) for x in self.torque]) + "]"
        return res
