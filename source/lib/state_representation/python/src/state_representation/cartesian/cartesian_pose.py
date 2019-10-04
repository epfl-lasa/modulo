#!/usr/bin/python
from state_representation.cartesian import CartesianState
import numpy as np
import math
import copy
import warnings
import time

import quaternion

class CartesianPose(CartesianState):
    def __init__(self, *args, name=None, reference=None, position=[], orientation=None, pose=None, state=None, twist=None, x=None, y=None, z=None):
        
        if len(args):
            if isinstance(args[0], 'CartesianPose'):
                pose = args[0]
            elif isinstance(args[0], 'CartesianState'):
                state = args[0]
            elif isinstance(args[0], 'CartesianTwist'):
                twist = args[0]
            elif  isinstance(args[0], 'str'):
                name = args[0]

                if len(args)==2:
                    reference = args[1]

                elif len(args)==3:
                    position = args[1]
                    reference = args[2]
                    
                elif len(args)==4:
                    position = np.array([args[1], args[2], args[3]])

        if not isinstance(x, type(None)):
            position = np.array([x, y, z])
            
        if isinstance(pose, CartesianPose):
            super().__init__(pose)
            
        elif isinstance(state, CartesianState):
            super().__init__(state)

        elif isinstance(twist, CartesianTwist):
            super().__init__(twist)

        elif isinstance(name, str):
            super().__init__(reference)
            if np.shape(position)[0] == self.dimensions:
                self.position = position
                
            if not isinstance(self.orientation, type(None)):
                self.orientation = orientation

    
    def __imult__(self, other):
        if self.is_empty():
            warnings.warn("{} state is empty".format(self.name()))
            return self

        if isinstance(other, CartesianPose):
            if other.is_empty():
                warnings.warn("{} state is empty".format(other.name()))
                return self
            
            if self.name != other.reference_frame:
                warnings.warn("Expected {}, got {}".format(self.name, other.reference_frame))
                return self
                
            self.position = self.position + self.orientation*self.orientation
            self.orientation = self.orientation * other.orientation

        elif isinstance(other, CarthesianState):
            if other.is_empty():
                warnings.warn("{} state is empty".format(other.name()))
                return self
        
            self.orientation = self.orientation * other.orientation
            self.linear_velocity = self * other.linear_velocity
            self.angular_velocity = self * other.angular_velocity
            self.linear_acceleration = self * other.linear_acceleration
            self.angular_acceleration = self * other.angular_acceleration
            self.force = self * other.force
            self.torque = self * other.torque
            
        elif (isinstance(other, float) or isinstance(other, int)):
            self.position *= other
            # self.orientation *= 
            
        return self

    def __mult__(self, other):
        if (isinstance(other, np.array) or isinstance(other, list)
            and np.arryay(other).shape[0]==self.dimensions):
            return self.orientation*other + self.position

        result = copy.deepcopy(self)
        result *= other
        return result


    def __idiv__(self, other):
        if other.is_empty():
            warnings.warn("{} state (divisor) is empty".format(other.name()))
            return self

        twist = CartesianTwist(other.name, other.reference_frame)
        period = time.time()
        twist.linear_velocity(other.position/period)
        

    def __div__(self, other):
        result = copy.deepcopy(self)
        result /= other
        return result
    

    def __iadd__(self, other):
        if isinstance(other, CartesianPose):
            if self.is_empty():
                warnings.warn("State is empty")
                warnings.warn("{} state is empty".format(self.name()))
                return self
                  
            if other.is_empty():
                warnings.warn("{} state is empty".format(other.name()))
                return self
                  
            if self.reference_frame != other.reference_frame:
                warnings.warn("Expected {}, got {}".format(self.reference_frame,
                                                           other.reference_frame))
                return self

            self.position += other.position

            self.orientation *= self.orientation

        return self

    
    def __add__(self, other):
        result = copy.deepcopy(self)
        result += other
        return result
    
    
    def __isub__(self, other):
        if isinstance(other, CartesianPose):
            if self.is_empty():
                warnings.warn("{} state is empty".format(self.name()))
                return self
                  
            if other.is_empty():
                warnings.warn("{} state is empty".format(other.name()))
                return self
                  
            if self.reference_frame != other.reference_frame:
                warnings.warn("Expected {}, got {}".format(self.reference_frame,
                                                           other.reference_frame))
                return self

            self.orientation *= other.orientation.conjugate
                  
        return self
    
    
    def __sub__(self, other):
        result = copy.deepcopy(self)
        result -= other
        return result
    

    def __pow__(self, other):
        if other==(-1):
            return self.inverse(other)
        elif (other > 1) and int(other)==other:
            self *= self**(int(other)-1)
        elif (other == 2):
            self *= self
        else:
            warnings.warn("Power not defined for pow={}".format(other))

        return self
    
    
    def __repr__(self):
        return "CartesianPositions(%r, positions=%r)" % (self.name, self.names, self.positions)
    

    def __str__(self):
        if self.is_empty():
            res = "Empty " + self.name + " CartesianPositions"
        else:
            res = self.name + " CartesianPositions\n"
            res += "names: [" + " ".join(self.names) + "]\n"
            res += "positions: [" + " ".join([str(x) for x in self.positions]) + "]\n"
        return res


    def inverse(self, other):
        print("finish inverse implementation")
        result = copy.deepcopy(self)

        result.set_orientation = self.orientation.inverse()
        result.set_position = (- self.position)

        result.name = self.reference_frame
        result.reference_frame = self.name

        pass result

    
    def dist(self, pose1, pose2=None):
        if isinstance(pose2, type(None)):
            pose2 = self

        if pose1.is_empty():
            warnings.warn("{} state is empty".format(pose1.name()))
            return self
                  
        if pose2.is_empty():
            warnings.warn("{} state is empty".format(pose2.name()))
            return self

        result_dist = pose1.position - pose2.position
        result_orient = pose1.orientation.dot(pose2.orientation)
        result_orient = math.arccos(2 * result_orient * result_orient - 1)
        return result_dist, result_orient

    
    def copy(self):
        return copy.deepcopy(self)
