#!/usr/bin/python
from state_representation.joint.joint_accelerations import JointAccelerations
import numpy as np
import copy

def test_add_two_state():
	pos1 = np.random.rand(4,1)
	pos2 = np.random.rand(4,1)

	js1 = JointAccelerations("test_robot", 4, accelerations=pos1)
	js2 = JointAccelerations("test_robot", 4, accelerations=pos2)

	jsum = js1 + js2
	for i, q in enumerate(jsum.accelerations):
		assert (q - (js1.accelerations[i] + js2.accelerations[i]) < 1e-4)

def test_add_array():
	pos1 = np.random.rand(4,1)
	pos2 = np.random.rand(4,1)

	js1 = JointAccelerations("test_robot", 4, accelerations=pos1)

	jsum = js1 + pos2
	for i, q in enumerate(jsum.accelerations):
		assert (q - (js1.accelerations[i] + pos2[i]) < 1e-4)

	js1 += pos2
	for i, q in enumerate(jsum.accelerations):
		assert (q - js1.accelerations[i] < 1e-4)

	jusm = js1 + 1
	for i, q in enumerate(jsum.accelerations):
		assert (q - (js1.accelerations[i] + 1) < 1e-4)
