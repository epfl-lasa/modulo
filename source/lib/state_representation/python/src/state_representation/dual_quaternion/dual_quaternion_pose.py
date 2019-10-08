#!/USSR/bin/python
from state_representation.state import State

import numpy as np
import math
import copy

import quaternion

class DualQuaternionPose(DualQuaternionState):
    def __init__(self, *args, name=None, reference="world",
                 position=None, orientation=None
                 state=None):
        
        if len(args):
            if isinstance(args[0], (DualQuaternionState, DualQuaternionPose)):
                dual_quaternion_pose = args[0]
            elif isinstance(args[0], str):
                for ii in range(len(arg)):
                    if (isinstance(args[ii], (list, np.ndarray))
                        and np.squeeze(args[ii]).shape=(3,)):
                        position = args[ii]
                    elif isinstance(args[ii], (list, np.ndarray, quaternion.quaternion)):
                        orientation = args[ii]
                    elif isintance(args[ii], str):
                        reference=args[ii]
                    else:
                        raise TypeError()
            else:
                raise TypeError()

        # Choose type
        if isinstance(state, DualQuaternionPose):
            super().__init__(state)
            self.position = state.position
            
        elif isinstance(state, DualQuaternionState):
            super().__init__(state)
        
        elif isinstance(robot_name, str):
            super().__init__("DualQuaternionState", name, reference)
            self.initialize()
            if not isinstance(positio, type(None)):
                self.position = position
                
            if not isinstance(orientation, type(None)):
                self.orientation = orientation
        else:
            raise TypeError()

    @property
    def position(self):
        # Position is evaluated here, because
        return 2*(self.dual*self.primary.conjugate()).vec

    @position.setter
    def position(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(3,)):
            value = np.squeeze(value)
            self.dual = quaternion.as_quat_array(0.5*np.hstack((0, position))*self.primary.components)
        elif (isinstance(value, (list, np.ndarray)) and
               np.squeeze(value).shape==(4,)):
            self.dual = quaternion.as_quat_array(np.squeeze(value))
            
        elif isinstance(value, quaternion.quaternion):
            self.dual = value
        else:
            TypeError("Wrong input argument -- {}".format(value))

    @property
    def orientation(self):
        return self.primary

    @orientation.setter
    def orientation(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(4,)):
            value = quaternion.as_quat_array(np.squeeze(value))
        elif not isinstance(value, (quaternion.quaternion)):
            TypeError("Wrong input argument -- {}".format(value))

        # TODO why normalize
        # if(orientation.norm() - 1 < 1e-4) temp.normalize();

        position_const = self.position
        
        if value.w < 0:
            self.primary = -value
        else:
            self.primary = value
            
        self.dual = quaternion.as_quat_array(0.5*np.hstack((0, position_const))*self.primary.components)

    def __imul__(self, other):
        if isintance(other, (DualQuaternionPose, DualQuaternionState)):
            if other.is_empty:
                raise RuntimeError()
            primary = self.primary * other.primary
            dual = self.primary*other.dual + self.dual*other.primary.components
        
            self.primary = primary
            self.position = dual # Updates Dual, too
            self.name = other.name

            return self
            
        elif isinstance(other, (float, int)):
            result = copy.deepcopy(self)
            result.primary = other*self.primary
            result.dual = other*self.dual
            return result
        
        else:
            TypeError("Wrong input argument -- {}".format(other))
            
    def conjugate(self):
        return super.conjugate()

    def inverse(self):
        result = copy.deepcopy(self)
        result.name = self.reference_frame
        result.reference_frame = self.name
        return result.conjugate()

    def log(self):
        result = DualQuaternionState(self.get_name, self.reference_frame)
        axis_angle = self.primary
        rotation = (axis_angle.angle * axis_angle.vec)/2
        position = state.position/2
        result.primary = quaternion.as_quat_array(np.hstack((0,position)))
        result.dual = quaternion.as_quat_array(np.hstack((0,position)))
        return result

    def initialize(self):
        super.initialize()
        self.position = np.array([0,0,0])

    def copy(self):
        return copy.deepcopy(self)

    def __str__(self):
        if self.is_empty:
            res = "Empty DualQuaternionPose"
        else:
            res = self.name + " DualQuaternionPose expressed in " + self.reference_frame + "\n"
            res += "primary/orientation: " + str(self.primary) + "\n"
            axis_angle = self.primary
            res += "<=> angle: " + str(axis_angle.angle) + "\n"
            res += "vector: " + str(axis_angle.vector) + "\n"
            res += "dual: " + str(self.dual) + "\n"
            res += "position: " + str(self.position) + "\n"
        return res

