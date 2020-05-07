#!/usr/bin/python
from state_representation.cartesian.cartesian_state import CartesianState

import numpy as np
import math
import copy
import warnings
import time

import quaternion

class CartesianWrench(CartesianState):
    ''' Force & Torque '''
    def __init__(self, *args, name=None, reference="world",
                 force=None, torque=None,
                 wrench_vector=None, state=None):

        if len(args):
            if isinstance(args[0], (CartesianState, CartesianPose, CartesianTwist)):
                state = args[0]
            elif  isinstance(args[0], str):
                name = args[0]

                if len(args)==2:
                    reference = args[1]

                elif len(args)==3:
                    if np.array(args[2]).shape[0]==3:
                        force = np.array(args[1])
                    elif np.array(args[2]).shape[0]==6:
                        wrench_vector = np.array(args[1])
                    else:
                        TypeError("Vector has unexpected dimensions {}".format(np.array(args[1])))
                    reference = args[2]
                    
                elif len(args)==4: 
                    force = np.array(args[1])
                    torque = np.array(args[2])
                    reference = args[3]
            else:
                TypeError("Unexpected Input format {}".format(type(args[0])))

        if isinstance(state, CartesianWrench):
            self = copy.deepcopy(state)
                          
        if isinstance(state, CartesianPose):
            super().__init__(state) # TODO FIX
            
        elif isinstance(state, CartesianTwist):
            super().__init__(state)

        elif isinstance(state, CartesianState):
            super().__init__(state)

        elif isinstance(name, str):
            super().__init__(name, reference)
            if not isinstance(wrench_vector, type(None)):
                self.set_wrench(self.wrench_vector)
            else:
                if not isinstance(force, type(None)):
                    self.force = force
                if not isinstance(torque, type(None)):
                    self.torque = torque
        else:
            super().__init__()

    @property
    def linear(self):
        return self.force

    @linear.setter
    def linear(self, value):
        self.force = value
        
    @property
    def angular(self):
        return self.torque

    @angular.setter
    def angular(self, value):
        self.torque = value
    
    def set_wrench(self, other):
        self.force = other[:3]
        self.torque = other[3:]

    def get_wrench(self):
        return np.hstack((self.force, self.torque))

    # def get_wrench(self):
        # return self.get_wrench()

    def __iadd__(self, other):
        if self.is_empty:
            RuntimeError("State is empty")

        if isinstance(other, CartesianWrench):
            if other.is_empty:
                RuntimeError("State is empty")

            if (self.reference_frame != self.reference_frame):
                RuntimeError("State is empty")

            self.force = self.force + other.force
            self.torque = self.torque + other.torque
            
        elif isinstance(other, (list, np.ndarray)):
            other = np.squeeze(other)
            if other.shape == (3,):
                self.set_wrench(self.get_wrench + other)
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

        if isinstance(other, CartesianWrench):
            if other.is_empty:
                RuntimeError("State is empty")

            if (self.reference_frame != self.reference_frame):
                RuntimeError("State is empty")

            self.force = self.force - other.force
            self.torque = self.torque - other.torque
            
        elif isinstance(other, (list, np.ndarray)):
            other = np.squeeze(other)
            if other.shape == (3,):
                self.set_wrench(self.get_wrench - other)
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
            self.force = self.force*other
            self.torque = self.torque*other

        # elif isinstance(other, (int, float)):
        else:
            TypeError("Unknown input type --- {}".format(type(other)))

    def __mul__(self, other):
        result = copy.deepcopy(self)
        result *= other
        return result

    def clamp(self, max_linear, max_angular, linear_noise_ratio=0, angular_noise_ratio=0):
        if linear_noise_ratio or angular_noise_ratio: # nonzero
            # Substract noise ratio
            force_norm = np.linalg.norm(self.force)
            if force_norm: # nonzero
                self.force = (self.force - self.linear_noise_ratio*self.force/force_norm)

            torque_norm = np.linalg.norm(self.torque)
            if torque_norm: # nonzero
                self.torque = (self.torque - self.angular_noise_ratio*self.torque/torque_norm)

            # Apply deadzone
            if(np.linalg.norm(self.force) < linear_noise_ratio):
                self.force = np.array(self.dimensions)

            if(np.linalg.norm(self.torque) < angular_noise_ratio):
                self.torque = np.array(self.dimensions)

        force_norm = np.linalg.norm(self.force)
        if force_norm > max_linear:
            self.force = self.force * max_linear/force_norm

        torque_norm = np.linalg.norm(self.torque)
        if torque_norm > max_angular:
            self.torque = self.torque * max_angular/torque_norm


    def clamped(self, max_linear, max_angular, linear_noise_ratio=0, angular_noise_ratio=0):
        result = copy.deepcopy(self)
        result.clamp(max_linear, max_angular, linear_noise_ratio, angular_noise_ratio)
        return result

    def __str__(self):
        if self.is_empty:
            res = "Empty CartesianWrench"
        else:
            res = self.name + " CartesianWrench expressed in " + self.reference_frame + "\n"
            res += "force: [" + " ".join([str(x) for x in self.force]) + "]\n"
            res += "torque: [" + " ".join([str(x) for x in self.torque]) + "]\n"
        return res
