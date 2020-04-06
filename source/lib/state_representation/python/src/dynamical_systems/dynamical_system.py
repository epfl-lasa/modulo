#!/usr/bin/python
from state_representation.state import State

class DynamicalSystem():
    """Virtual Dynamical System"""
    def __init__(self, gain=1, state=None, dimensions=3, max_velocity=None):
        if state is None:
            self.gain = gain
        
        # elif isinstance(state, State):
        self.dimensions = dimensions

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

    def limit_velocity(self, velocity):
        # Set to constant or maximum velocity
        if not (self.max_velocity is None):
            norm_vel = np.linalg.norm(velocity)
            if norm_vel>self.max_velocity:
                velocity = velocity/norm_velocity*self.max_velocity
        return velocity

    def evaluate(self):
        raise NotImplementedError()
    
    
