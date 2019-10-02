#!/usr/bin/python
from state_representation.joint.joint_state import JointState
import numpy as np
import copy

def test_state_creation():
	s = JointState("test", 6)
	assert s.name == "test"
	assert s.reference_frame == "world"
	assert s.is_empty()
	assert len(s.names) == 6
	assert s.names[0] == "joint0"

def test_set_positions_with_modulo():
	positions = [1, 2, 3, 4]
	js = JointState("test_robot", 4)
	js.positions = positions
	assert len(js.positions) == 4
	for q in js.positions:
		assert -np.pi < q and q < np.pi

def test_add_two_state():
	pos1 = np.random.rand(4,1)
	pos2 = np.random.rand(4,1)

	js1 = JointState("test_robot", 4, positions=pos1)
	js2 = JointState("test_robot", 4, positions=pos2)

	jsum = js1 + js2
	for i, q in enumerate(jsum.positions):
		assert (q - (js1.positions[i] + js2.positions[i]) < 1e-4)

def test_multiply_with_scalar():
	pos1 = np.random.rand(4,1)
	js1 = JointState("test_robot", 4, positions=pos1)
	
	k = 2
	jres = js1 * k
	for i, q in enumerate(jres.positions):
		assert (q - (k * js1.positions[i]) < 1e-4)

	k = 1.5
	jres = js1 * k
	for i, q in enumerate(jres.positions):
		assert (q - (k * js1.positions[i]) < 1e-4)

	k = 2
	js2 = copy.deepcopy(js1)
	js2 *= k
	for i, q in enumerate(js2.positions):
		assert (q - (k * js1.positions[i]) < 1e-4)

def test_multiply_with_array():
	pos1 = np.random.rand(4,1)
	js1 = JointState("test_robot", 4, positions=pos1)

	k = np.random.rand(4,1)
	jres = js1 * k
	for i, q in enumerate(jres.positions):
		assert (q - (k[i] * js1.positions[i]) < 1e-4)

	k = np.random.rand(4,1)
	js2 = copy.deepcopy(js1)
	js2 *= k
	for i, q in enumerate(js2.positions):
		assert (q - (k[i] * js1.positions[i]) < 1e-4)