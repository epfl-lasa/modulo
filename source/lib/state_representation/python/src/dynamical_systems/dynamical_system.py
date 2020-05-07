#!/usr/bin/python
from state_representation.state import State

import copy

class DynamicalSystem(State):
    """Virtual Dynamical System"""
    def __init__(self, gain=1, state=None, max_velocity=None):
        
        if isinstance(state, DynamicalSystem):
            self = copy.deepcopy(state)
        elif isinstance(state, State):
            super().__init__(state)
        else:
            super().__init__()
            self.gain = gain
            self._max_velocity = max_velocity

    @property
    def max_velocity(self):
        return self._max_velocity

    @max_velocity.setter
    def max_velocity(self, value):
        self._max_velocity = value
    
    @property
    def gain(self):
        return self._gain
    
    @gain.setter
    def gain(self, value):
        self._gain = value

    def evaluate(self):
        raise NotImplementedError("Implement this function in each derived-class")

    # def limit_velocity(self, velocity):
    #     # Set to constant or maximum velocity
    #     if not (self.max_velocity is None):
    #         norm_vel = np.linalg.norm(velocity)
    #         if norm_vel>self.max_velocity:
    #             velocity = velocity/norm_velocity*self.max_velocity
    #     return velocity


