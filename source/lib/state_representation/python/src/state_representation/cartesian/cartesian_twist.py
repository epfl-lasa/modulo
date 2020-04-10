#!/usr/bin/python
from state_representation.cartesian.cartesian_state import CartesianState
from state_representation.cartesian.cartesian_wrench import CartesianWrench

import numpy as np
import math
import copy
import warnings
import time

import quaternion

class CartesianTwist(CartesianState):
    def __init__(self, *args, name=None, reference="world",
                 linear_velocity=None, angular_velocity=None,
                 twist_vector=None, twist=None, state=None):

        if len(args):
            if isinstance(args[0], CartesianState):
                twist = args[0]
            elif  isinstance(args[0], str):
                name = args[0]

                if len(args)==2:
                    reference = args[1]

                elif len(args)==3:
                    if np.array(args[1]).shape[0]==3:
                        linear_velocity = np.array(args[1])
                        angular_velocity = np.array(args[2])
                    elif np.array(args[1]).shape[0]==6:
                        twist_vector = np.array(args[1])
                    else:
                        TypeError("Vector has unexpected dimensions {}".format(np.array(args[1])))
                    
                elif len(args)==4:
                    linear_velocity = np.array(args[1])
                    angular_velocity = np.array(args[2])
                    reference = args[3]
            else:
                TypeError("Unexpected Input format {}".format(type(args[0])))
                
        if isinstance(state, CartesianTwist):
            self = copy.deepcopy(state)
        elif isinstance(state, CartesianState):
            super().__init__(state)
        elif isinstance(name, str):
            super().__init__(name, reference)
        else:
            super().__init__()
            
        if not isinstance(twist_vector, type(None)):
            self.set_twist(self.twist_vector)
        else:
            if not isinstance(linear_velocity, type(None)):
                self.linear_velocity = linear_velocity
            if not isinstance(angular_velocity, type(None)):
                self.angular_velocity = angular_velocity

    def __str__(self):
        if self.is_empty:
            res = "Empty CartesianTwist"
        else:
            res = self.name + " CartesianTwist expressed in " + self.reference_frame + "\n"
            res += "linear_velocity: [" + " ".join([str(x) for x in self.linear_velocity]) + "]\n"
            res += "angular_velocity: [" + " ".join([str(x) for x in self.angular_velocity]) + "]\n"
        return res


    @property
    def linear(self):
        # self._linear = self._linear_velocity
        return self.linear_velocity

    @linear.setter
    def linear(self, value):
        # self._linear = value
        self.linear_velocity = value
        
    @property
    def angular(self):
        # self._angular = self._angular_velocity
        return self.angular_velocity

    @angular.setter
    def angular(self, value):
        # self._angular = value
        self.angular_velocity = value

    @property
    def twist(self):
        return np.hstack((self.linear_velocity, self.angular_velocity))

    @twist.setter        
    def twist(self, other):
        self.linear_velocity = other[:3]
        self.angular_velocity = other[3:]

    def get_array(self):
        return self.twist

    def __iadd__(self, other):
        if self.is_empty:
            RuntimeError("State is empty")

        if isinstance(other, (CartesianTwist, CartesianWrench)):
            if other.is_empty:
                RuntimeError("State is empty")

            if (self.reference_frame != self.reference_frame):
                RuntimeError("State is empty")

            # Allow to adding twist & acceleration (for integration)
            self.linear_velocity = self.linear_velocity + other.linear
            self.angular_velocity = self.angular_velocity + other.angular
            
        elif isinstance(other, (list, np.ndarray)):
            other = np.squeeze(other)
            if other.shape == (3,):
                self.set_twist(self.get_twist + other)
            else:
                TypeError("Wrong array size --- dim = {}".format(np.array(other)))
            
        else:
            TypeError("Unknown input type --- {}".format(type(other)))

        return self

    def __add__(self, other):
        result = copy.deepcopy(self)
        result += other
        return result

    def __isub__(self, other):
        if self.is_empty:
            RuntimeError("State is empty")

        if isinstance(other, (CartesianTwist, CartesianWrench)):
            if other.is_empty:
                RuntimeError("State is empty")

            if (self.reference_frame != self.reference_frame):
                RuntimeError("State is empty")

            self.linear_velocity = self.linear_velocity - other.linear
            self.angular_velocity = self.angular_velocity - other.angular
            
        elif isinstance(other, (list, np.ndarray)):
            other = np.squeeze(other)
            if other.shape == (3,):
                self.set_twist(self.get_twist - other)
            else:
                TypeError("Wrong array size --- dim = {}".format(np.array(other)))
        else:
            TypeError("Unknown input type --- {}".format(type(other)))

        return self


    def __sub__(self, other):
        result = copy.deepcopy(self)
        result -= other
        return result

    def __imul__(self, other):
        if self.is_empty:
            RuntimeError("State is empty")

        if isinstance(other, (int, float)):
            self.linear_velocity = self.linear_velocity*other
            self.angular_velocity = self.angular_velocity*other
        else:
            TypeError("Unknown input type --- {}".format(type(other)))
        return self

    def __mul__(self, other):
        result = copy.deepcopy(self)
        result *= other
        return result
    
    def clamp(self, max_linear=None, max_angular=None, linear_noise_ratio=None, angular_noise_ratio=None):
        if not linear_noise_ratio is None:
            # Substract noise ratio
            linear_velocity_norm = np.linalg.norm(self.linear_velocity)
            if linear_velocity_norm: # nonzero
                self.linear_velocity = (self.linear_velocity - linear_noise_ratio*self.linear_velocity/linear_velocity_norm)

            # Apply deadzone
            if(np.linalg.norm(self.linear_velocity) < linear_noise_ratio):
                self.linear_velocity = np.zeros(self.dimensions)
                
        if not angular_noise_ratio is None: # nonzero
            angular_velocity_norm = np.linalg.norm(self.angular_velocity)
            if angular_velocity_norm: # nonzero
                 self.angular_velocity = (self.angular_velocity - angular_noise_ratio*self.angular_velocity/angular_velocity_norm)
            
            if(np.linalg.norm(self.angular_velocity) < angular_noise_ratio):
                self.angular_velocity = np.zeros(self.dimensions)

        if not max_linear is None:
            linear_velocity_norm = np.linalg.norm(self.linear_velocity)
            if linear_velocity_norm > max_linear:
                self.linear_velocity = self.linear_velocity * max_linear/linear_velocity_norm

        if not max_angular is None:
            angular_velocity_norm = np.linalg.norm(self.angular_velocity)
            if angular_velocity_norm > max_angular:
                self.angular_velocity = self.angular_velocity * max_angular/angular_velocity_norm


    def clamped(self, max_linear=None, max_angular=None, linear_noise_ratio=None, angular_noise_ratio=None):
        result = copy.deepcopy(self)
        result.clamp(max_linear, max_angular, linear_noise_ratio, angular_noise_ratio)
        return result

