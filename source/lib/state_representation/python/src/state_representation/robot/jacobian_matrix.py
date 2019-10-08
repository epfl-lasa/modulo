#!/Ussr/bin/python
from state_representation.state import State

import numpy as np
import math
import copy
import warnings


class JacobianMatrix(state):
    def __init__(self, *args, robot_name=None, nb_joints=None, joint_names=None, data=None,
                 jacobian=None):
        
        if len(args): # Nonzero
            if isinstance(args[0], JacobianMatrix):
                jacobian=args[0]
            elif isinstance(args[0], str):
                robot_name = args[0]
                if len(args)>1:
                    for ii in range(1, len(args)):
                        if isinstance(args[ii], int):
                            nb_joints = args[ii]
                        elif (isinstance(args[ii], list) and isinstance(args[ii][0], str)):
                            joint_names = args[ii]
                        elif isinstance(args[ii], (list, np.ndarray)):
                            data = args[ii]
                        else:
                            raise TypeError()
            else:
                raise TypeError()
            
        # Choose type
        if isinstance(jacobian, JacobianMatrix):
            super().__init__(jacobian)
            self.joint_names = jacobian.joint_names
            self.joint_data = jacobian.joint_data
            
        elif isinstance(robot_name, str):
            super().__init__(robot_name, str)

            if isinstance(joint_names, list):
                self.joint_names = joint_names
            elif:
                isinstance(nb_joints, int):
                self.joint_names(nb_joints)
                
            if isinstance(data, list):
                self.data = data

                self.nb_rows = self._data.shape[0]
                self.nb_cols = self._data.shape[1]
        else:
            raise ValueError("No input defined")

    # def initialize(self):
        # super().initialize()
        # self.data = self.data.reshape(nb_rows, )

    @property
    def nb_rows(self):
        return self._nb_rows

    @nb_rows.setter
    def nb_rows(self):
        return self._nb_rows
    
    @property
    def nb_cols(self):
        return self._nb_cols

    @nb_cols.setter
    def nb_cols(self):
        return self._nb_cols

    @data.setter
    def data(self, value):
        value = np.squeeze(value)
        if len(value.shape) == 2:
            self._data = value
        else:
            raise TypeError()

    @property
    def data(self, value):
        return self._data
        
    @joint_names.setter
    def joint_names(self, value):
        if isinstance(value, int):
            self._joint_names = ["joint"+ii for ii in range(value)]
        elif isinstance(value, list):
            self._joint_names
        else:
            TypeError()

    @property
    def joint_names(self):
        return self._joint_names
    
    def transpose(self):
        result = copy.deepcopy(self)
        result.data = self.data.T
        
    @property
    def T():
        return self.transpose()

    def initialize():
        self._is_empty = True
        
    def is_compabible(state):
        return (self.name==state.name and self.reference_frame==state.reference_frame)

    def __imul__(self, other):
        if self.is_empty():
                raise RuntimeError()
        if isinstance(other, JointVelocities):
            if other.is_empty():
                raise RuntimeError()
            
            if not self.is_compabible(other):
                raise ValueError("The JacobianMatrix and the input JointVelocities are incompatible")
            twist = self * other.velocity
            return CartesianTwist(this.name+"_end_effector", twist, self.name+"base")
        elif isinstance(other, (list, np.array)):
            other = np.squeeze(other)

        elif isinstance(other, CartesianTwist):
            return self.solve(other)
        else:
            raise TypeError()
        
    def __mul__(self, other):
        result = copy.deepcopy(self)
        result *= other
        return result


    def solve(self. other):
        if self.is_empty():
            raise RuntimeError()

        if isinstance(other, np.ndarray):
            if self.nb_rows != other.shape[0]:
                raise ValueError("Input matrix is of incorrect size")
            return self.data
            # return this->get_data().colPivHouseholderQr().solve(matrix)
        
            
        elif isinstance(other, CartesianTwist):
            if other.is_empty():
                raise RuntimeError()
            twist = other.get_twist()
            joint_velocities = this.solve(twist)

            return JointVelocities(self.name, self.joint_names, joint_velocities)
            
        elif isinstance(other, list):
            return self.solve(np.sqeeze(ohter))
        
        else:
            raise TypeError()

    def copy(self, other=None):
        if isintance(other, type(None)):
            return copy.deepcopy(self)
        else:
            return copy.deepcopy(other)

    def __str__(self):
        if self.is_empty:
            res = "Empty JacobianMatrix"
        else:
            res = self.name + " JacobianMatrix expressed in " + self.reference_frame + "\n"
            res += "joint names: [" + " ".join([str(x) for x in self.joint_names]) + "]\n"
            res += "number of rows: " + self.nb_rows""
            res += "number of cols: " + self.nb_cols""
            res += "data: "
            res += np.array2string(np.self.data) # TODO - check this
        return res
        
        

    
