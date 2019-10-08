#!/USSR/bin/python
from state_representation.state import State

import numpy as np
import math
import copy

import quaternion

class DualQuaternionState(State):
    def __init__(self, *args, name=None, reference="world",
                 primary=None, dual=None
                 dual_quaternion_state=None):
        if len(args):
            if isinstance(args[0], DualQuaternionState):
                dual_quaternion_state = args[0]
            elif isinstance(args[0], str):
                for ii in range(len(arg)):
                    if isinstance(args[ii], (list, np.ndarray, quaternion.quaternion)):
                        if not isintance(primary, type(None))
                            dual = args[ii]
                        else:
                            primary = args[ii]
                    elif isintance(args[ii], str):
                        reference="world"
                    else:
                        raise TypeError()
            else:
                raise TypeError()

        # Choose type
        if isinstance(dual_quaternion_state, DualQuaternionState):
            super().__init__(DualQuaternionState)
            
            self.primary = state.primary
            self.dual = state.dual
            
        elif isinstance(robot_name, str):
            super().__init__("DualQuaternionState", name, reference)

            if not isinstance(primary, type(None)):
                self.primary = primary
                
            if not isinstance(dual, type(None)):
                self.dual = dual
        else:
            super.__init__("DualQuaternionState")


    @property
    def primary(self):
        return self._primary

    @primary.setter
    def primary(self):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(4,)):
            self._primary = quaternion.as_quat_array(value)
            self.set_filled()        
        elif isinstance(value, (quaternion.quaternion)):
            self._primary = value
            self.set_filled()
        else:
            TypeError("Wrong input argument -- {}".format(value))

    @property
    def dual(self):
        return self._dual

    @dual.setter
    def dual(self, value):
        if (isinstance(value, (list, np.ndarray)) and
                      np.squeeze(value).shape==(4,)):
            self._dual = quaternion.as_quat_array(value)
            self.set_filled()
        elif isinstance(value, (quaternion.quaternion)):
            self._dual = value
            self.set_filled()
        else:
            TypeError("Wrong input argument -- {}".format(value))
        
    def __imul__(self, other):
        if isintance(other, DualQuaternionState):
            if other.is_empty:
                raise RuntimeError()
            primary = self.primary * other.primary
            dual = self.primary*other.dual + self.dual*other.primary.components
        
            self.primary = primary
            self.dual = dual
            self.name = other.name

            return self
        elif isinstance(other, (float, int)):
            result = copy.deepcopy(self)
            result.primary = other*self.primary
            result.dual = other*self.dual
            return result
        
        else:
            TypeError("Wrong input argument -- {}".format(other))

    def __mul__(self, other):
        result = copy.deepcopy(self)
        result *= other
        return result

    def __rmul__(self, other):
        return self * other

    def exp(self, other):
        if isintance(other, DualQuaternionState):
            if other.is_empty:
                raise RuntimeError()
            result = DualQuaternionState(other.name, other.reference_frame)
            pexps = quternion.quaternion(1, 0, 0, 0)
            norm_primary = state.primary.norm()
            if(norm_primary > 1e-5):
                coeffs = math.sin(norm_primary)/norm_primary*state.primary.components
                pexps = quaternion.as_quat_array(coeffs)
            else:
                pexps = quaternion.quaternion(1, 0, 0, 0)
                
            result.primary = pexps
            result.dual = state.dual * pexp
            return result
        else:
            raise TypeError()

    def conjugate(self):
        result = copy.deepcopy(self)
        result.primary = primary.conjugate()
        result.dual = dual.conjugate()
        return result

    def initialize(self):
        super.initialize()
        self.primary = quaternion.quaternion(1,0,0,0)
        self.dual = quaternion.quaternion(0,0,0,0)

    def copy(self):
        return copy.deepcopy(self)

    def __str__(self):
        if self.is_empty:
            res = "Empty DualQuaternionState"
        else:
            res = self.name + " DualQuaternionState expressed in " + self.reference_frame + "\n"
            res += "joint names: [" + " ".join([str(x) for x in self.joint_names]) + "]\n"
            res += "primary: " + self.primary
            res += "dual: " + self.dual
        return res

