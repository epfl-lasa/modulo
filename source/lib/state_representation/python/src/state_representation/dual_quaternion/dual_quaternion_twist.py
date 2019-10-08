#!/USSR/bin/python
from state_representation.state import State

import numpy as np
import math
import copy

import quaternion

class DualQuaternionTwist(DualQuaternionState):
    def __init__(self, *args, name=None, reference="world",
                 linear_velocity=None, angular_velocity=None
                 state=None):
        
        if len(args):
             if isinstance(args[0], (DualQuaternionState, DualQuaternionTwist)):
                dual_quaternion_pose = args[0]
            elif isinstance(args[0], str):
                if len(args)==2:
                    reference = args[1]
                elif len(args)==5:
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
        if isinstance(state, DualQuaternionTwist):
            super().__init__(state)
            self.initialize()
            self.linear_velocity = state.linear_velocity
            self.angular_velocity = state.angular_velocity
            self.position = state.position
            
        # elif isinstance(state, DualQuaternionState):
            # super().__init__(state)
            # self.primary = state.primary
            # self.dual = state.dual
            # self.linear_velocity = state.dual.vec - np.cross(self.position, self.get_angular_velocity)
            
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
        return _position

    @position.setter
    def position(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(3,)):
            self._position = np.squeeze(value)
            self.dual = self.get_absolute_velocity
        else:
            TypeError("Wrong input argument -- {}".format(value))

    @property
    def linear_velocity(self):
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(3,)):
            self._linear_velocity = np.squeeze(value)
            self.dual = np.hstack((0, self.get_absolute_velocity))
        else:
            TypeError("Wrong input argument -- {}".format(value))
            
    @property
    def angular_velocity(self):
        return self.primary.vec

    @angular_velocity.setter
    def angular_velocity(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(3,)):
            self.primary = quaternion.quaternion((np.vstack((0, value))))
            self.dual = self.get_dual_from_absolute_velocity()
        else:
            TypeError("Wrong input argument -- {}".format(value))

    def get_absolute_velocity_dual(self):
        # TODO - why NOT angular_vel x positionp
        absolute_velocity = self.linear_velocity + np.cross(self.position, self.angular_velocity)
        return quaternion.quaternion(np.hstack((0, absolute_velocity)))

    def initialize(self):
        super.initialize()
        self.position = np.array([0,0,0])
        self.linear_velocity = np.array([0,0,0])

    def copy(self):
        return copy.deepcopy(self)

    def __str__(self):
        if self.is_empty:
            res = "Empty DualQuaternionPose"
        else:
            res = self.name + " DualQuaternionPose expressed in " + self.reference_frame + "\n"
            res += "primary: " + str(self.primary) + "\n"
            res += "<=> angular_velocity: " + str(self.primary.axis) + "\n"
            res += "dual: " + str(self.dual) + "\n"
            res += "linear velocity: " + str(self.linear_velocity) + "\n"
            res += "angular velocity: " + str(self.angular_velocity) + "\n"
        return res

