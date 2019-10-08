#!/usr/bin/python
import time

import copy

class State(object):
    def __init__(self, *args, typename=None, name="default", reference_frame="world", is_empty=True, state=None):

        # Decode input
        if len(args): # Nonzero
            if isinstance(args[0], State):
                state = args[0]
            elif isinstance(args[0], str):
                typename = args[0]
                if len(args)>1:
                    name=args[1]
                    if len(args)>2:
                        reference_frame=args[2]
                        if len(args)>3:
                            is_empty=args[3]
            else:
                Exception("Input has wrong type -- {}.".format(type(args[0])))

        if isinstance(typename, str):
            self.typename = typename
            self.name = name
            self.reference_frame = reference_frame
            self._is_empty = is_empty
            self.timestamp = time.time()
            
        elif isinstance(state, State):
            self.typename = state.typename
            self.name = state.name
            self.reference_frame = state.reference_frame
            self._is_empty = state.is_empty
            self.timestamp = time.time()
            
        else:
            Exception("Wrong input type.")
                

    @property
    def typename(self):
        return self._typename

    @typename.setter
    def typename(self, value):
        self._typename = value

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value

    @property
    def reference_frame(self):
        return self._reference_frame

    @reference_frame.setter
    def reference_frame(self, value):
        self._reference_frame = value

    @property
    def reference_frame(self):
        return self._reference_frame

    @reference_frame.setter
    def reference_frame(self, value):
        self._reference_frame = value

    @property
    def is_empty(self):
        return self._is_empty
    
    # @is_empty.setter
    # def is_empty(self, value):
        # self._is_empty = bool(value)

    # Keep for compatibility with c++
    def set_empty(self):
        self._is_empty = True

    def set_filled(self):
        self.reset_timestamp()
        self._is_empty = False
        
    def reset_timestamp(self):
        self._timestamp = time.time()

    def is_deprecated(self, time_delay):
        return (time.time() - self._timestamp) > time_delay

    def initialize(self):
        self.set_empty()

    def __add__(self, other):
        result = copy.deepcopy(self)
        result += other
        return result

    def __radd__(self, other):
        return self + other

    def __sub__(self, other):
        result = copy.deepcopy(self)
        result += other
        return result

    def __rsub__(self, other):
        return self + other

    def __mul__(self, other):
        result = copy.deepcopy(self)
        result *= other
        return result

    def __div__(self, other):
        result = copy.deepcopy(self)
        result /= other
        return result

    # Multiplication and division not commutative
    
    def __repr__(self):
        return "State(%r, %r, %r, %r)" % (self._typename, self._name, self._reference_frame, self._empty)

    def __str__(self):
        if self._empty:
            res = "Empty "
        res += "State: " + self._name + " expressed in " + self._reference_frame + " frame"
        return res
