#!/usr/bin/python
from state_representation.state import State

import numpy as np

class LinearDynamicalSystem(State):
    def __init__(self, attractor=None, state=None, dim=3, *args, **kwargs):

        if isinstance(dynamical_system, state):
            if isinstance(dynamical_system, LinearDynamicalSystem):
                self = copy.deepcopy(state)
            else:
                super().__init__(state=state, *args, **kwargs)
        super().__init__(*args, **kwargs)
            
        # elif isinstance(state, State):

    @property
    def attracractor(self):
        return self._attracractor

    @attractor.setter
    def attracractor(self, value):
        self._attracractor = value
    
    def evaluate(self, position):
        if self.attractor is None:
            raise ValueError("No attractor defined.")

        velocity = position-self.attractor
        
        return self.limit_velocity(velocity)

