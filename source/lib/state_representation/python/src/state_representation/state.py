#!/usr/bin/python
import time

class State(object):
    def __init__(self, typename, name, reference_frame="world", empty=True):
        self._typename = typename
        self._name = name
        self._reference_frame = reference_frame
        self._empty = empty
        self._timestamp = time.time()

    @property
    def typename(self):
        return self._typename

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

    def is_empty(self):
        return self._empty

    def set_empty(self):
        self._empty = True

    def set_filled(self):
        self._empty = False
        self.reset_timestamp()

    def reset_timestamp(self):
        self._timestamp = time.time()

    def is_deprecated(self, time_delay):
        return (time.time() - self._timestamp) > time_delay

    def initialize(self):
        self.set_empty()

    def __repr__(self):
        return "State(%r, %r, %r, %r)" % (self._typename, self._name, self._reference_frame, self._empty)

    def __str__(self):
        if self._empty:
            res = "Empty "
        res += "State: " + self._name + " expressed in " + self._reference_frame + " frame"
        return res
    
