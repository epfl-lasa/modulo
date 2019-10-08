#!/usr/bin/python
from state_representation.cartesian.cartesian_state import CartesianState
from state_representation.cartesian.cartesian_twist import CartesianTwist
from state_representation.cartesian.cartesian_wrench import CartesianWrench

import numpy as np
import math
import copy
import warnings
import time

import quaternion

class CartesianPose(CartesianState):
    def __init__(self, *args, name=None,
                 position=[0, 0, 0], orientation=[1, 0, 0, 0], reference="world",
                 state=None,
                 x=None, y=None, z=None):

        if len(args): # Assign arguments
            if isinstance(args[0],
                          (CartesianState, CartesianPose, CartesianTwist, CartesianWrench)):
                state = args[0]

            elif isinstance(args[0], str):
                name = args[0]
                ii = 1
                while(ii < len(args)):
                    if isinstance(args[ii], (list, np.ndarray)):
                        if np.unique(args[ii]).shape == (3,):
                            position = args[ii]
                        elif np.unique(args[ii]).shape == (4,):
                            orientation = quaternion.as_quat_array(args[ii])
                    elif isinstance(args[ii], (quaternion.quaternion)):
                        orientation = args[ii]
                    elif isinstance(args[ii], (str)):
                        reference = args[ii]
                    elif isinstance(args[ii], (float, int)):
                        position = args[ii:ii+3]
                        ii+=2
                    ii += 1
            else:
                raise TypeError()
                    
        if isinstance(x, (float, int)):
            position[0] = x
        if isinstance(y, (float, int)):
            position[1] = y
        if isinstance(x, (float, int)):
            position[2] = z
            
        if isinstance(state, CartesianPose):
            self = copy.deepcopy(state)
        elif isinstance(state, CartesianTwist):
            super().__init__(state)
        elif isinstance(state, CartesianWrench):
            super().__init__(state)
        elif isinstance(state, CartesianState):
            super().__init__(state)

        else:
            if isinstance(name, str):
                super().__init__(name)
            else:
                super().__init__()

            self.position = position
            self.orientation = orientation
            self.reference_frame = reference

    @property
    def linear(self):
        self._linear = self.position
        return self._linear

    @linear.setter
    def linear(self, value):
        self._linear = value
        self._position = value
        
    @property
    def angular(self):
        self.angular = self.orientation
        return self._angular

    @angular.setter
    def angular(self, value):
        self._angular = value
        self._orientation = value

    @property
    def value_names(self):
        return {"linear":"position", "angular":"orientation"}
    
        
    def __imul__(self, other):
        if self.is_empty:
            raise RuntimeError("{} state is empty".format(self.name))

        if isinstance(other, CartesianPose):
            if other.is_empty:
                raise RuntimeError("{} state is empty".format(other.name))

            if (self.name != other.reference_frame):
                raise ValueError("Expected {}, got {}".format(self.name, other.reference_frame))

            # self.position = self.position + self.orientation*other.position
            self.position = self.position + quaternion.rotate_vectors(self.orientation, other.position)
            self.orientation = self.orientation * other.orientation

        elif isinstance(other, CartesianState):
            self.orientation = self.orientation * other.orientation
            
            self.linear_velocity = self.linear_velocity * other.linear_velocity
            self.angular_velocity = self.angular_velocity * other.angular_velocity
            
            self.linear_acceleration = self.linear_acceleration * other.linear_acceleration
            self.angular_acceleration = self.angular_acceleration * other.angular_acceleration
            
            self.force = self * other.force
            self.torque = self * other.torque
            
        elif isinstance(other, (float, int)):
            self.position *= other
            self.orientation *=  other
            
        return self

    def __mul__(self, other):
        if (isinstance(other, (np.ndarray, list))
            and np.arryay(other).shape[0]==self.dimensions):
            return self.orientation*other + self.position

        result = copy.deepcopy(self)
        result *= other
        return result


    def __idiv__(self, other):
        if other.is_empty:
            raise RuntimeError("{} state is empty".format(other.name))

        twist = CartesianTwist(other.name, other.reference_frame)
        period = time.time()
        twist.linear_velocity(other.position/period)

    def __div__(self, other):
        result = copy.deepcopy(self)
        result /= other
        return result

    def __iadd__(self, other):
        if self.is_empty:
            raise RuntimeError("{} state is empty".format(self.name))
        if isinstance(other, CartesianPose):
            if other.is_empty:
                raise RuntimeError("{} state is empty".format(other.name()))
                  
            if self.reference_frame != other.reference_frame:
                raise ValueError("Expected {}, got {}".format(self.reference_frame,
                                                                other.reference_frame))
            self.position += other.position
            self.orientation *= self.orientation
        else:
            raise TypeError()

        return self
    
    def __add__(self, other):
        result = copy.deepcopy(self)
        result += other
        return result
    
    def __isub__(self, other):
        if isinstance(other, CartesianPose):
            if self.is_empty:
                raise RuntimeError("{} state is empty".format(self.name()))
                  
            if other.is_empty:
                raise RuntimeError("{} state is empty".format(other.name()))
                  
            if self.reference_frame != other.reference_frame:
                raise ValueError("Expected {}, got {}".format(self.reference_frame,
                                                           other.reference_frame))

            self.orientation *= other.orientation.conjugate
        else:
            TypeError()
                  
        return self
    
    def __sub__(self, other):
        result = copy.deepcopy(self)
        result -= other
        return result

    def __pow__(self, other):
        if other==(-1):
            return self.inverse()
        elif (other > 1) and int(other)==other:
            self *= self**(int(other)-1)
        elif (other == 2):
            self *= self
        else:
            raise ValueError("Power not defined for pow={}".format(other))
        
        return self
    
    def __repr__(self):
        return "CartesianPositions(%r, position=%r)" % (self.name, self.position)

    def __str__(self):
        if self.is_empty:
            res = "Empty " + self.name + " CartesianPosition"
        else:
            res = self.name + " CartesianPosition\n"
            res += "name: [" + " ".join(self.name) + "]\n"
            res += "position: [" + " ".join([str(x) for x in self.position]) + "]\n"
            res += "orientation: [" + " ".join([str(x) for x in self.orientation.components]) + "]\n"
        return res

    def inverse(self):
        result = copy.deepcopy(self)

        result.name = self.reference_frame
        result.reference_frame = self.name

        result.orientation = self.orientation.conjugate()
        result.position = quaternion.rotate_vectors(result.orientation, -self.position)
        
        return result
    
    def dist(self, pose1, pose2=None):
        if isinstance(pose2, type(None)):
            pose2 = self

        if pose1.is_empty:
            raise RuntimeError("{} state is empty".format(pose1.name()))
                  
        if pose2.is_empty:
            raise RuntimeError("{} state is empty".format(pose2.name()))

        result_dist = pose1.position - pose2.position
        result_orient = pose1.orientation.dot(pose2.orientation)
        result_orient = math.arccos(2 * result_orient * result_orient - 1)
        return result_dist, result_orient

                            
    def copy(self):
        return copy.deepcopy(self)
