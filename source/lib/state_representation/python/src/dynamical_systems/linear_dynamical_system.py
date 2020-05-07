#!/usr/bin/python
from state_representation.state import State
from state_representation.cartesian.cartesian_pose import CartesianPose
from state_representation.cartesian.cartesian_twist import CartesianTwist
from dynamical_systems.dynamical_system import DynamicalSystem

import numpy as np
import quaternion

class LinearDynamicalSystem(DynamicalSystem):
    ''' Linera DS '''
    
    # TODO create for cartesian space
    def __init__(self, *args, attractor=None, state=None, dim=3, **kwargs):

        if isinstance(state, State):
            if isinstance(dynamical_system, LinearDynamicalSystem):
                self = copy.deepcopy(state)
            else:
                super().__init__(state=state, *args, **kwargs)
        else:
            super().__init__(*args, **kwargs)
            
            if not attractor is None:
                self.attractor = attractor
            
        # elif isinstance(state, State):

    @property
    def attractor(self):
        return self._attracractor

    @attractor.setter
    def attractor(self, value):
        # TODO: check attractor type? cartesian/joint?
        # self._attracractor = CartesianTwist(value)
        self._attracractor = value
    
    def evaluate(self, state):
        if self.attractor is None:
            raise ValueError("No attractor defined.")

        if isinstance(state, CartesianPose):
            pose_diff = (self.attractor-state)
            
            ds_twist = CartesianTwist(state=pose_diff, linear_velocity=pose_diff.position, angular_velocity=quaternion.as_rotation_vector(pose_diff.orientation))

            return self.gain*ds_twist
        
        else:
            raise NotImplementedError("Implement for other state types")

