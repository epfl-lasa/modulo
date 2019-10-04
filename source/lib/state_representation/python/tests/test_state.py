#!/usr/bin/python
from state_representation.state import State

def test_state_creation():
    s = State("state", "test")
    assert s.name == "test"
    assert s.reference_frame == "world"
    assert s.is_empty()