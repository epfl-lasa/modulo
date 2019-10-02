#!/usr/bin/python
from state_representation.joint.joint_torques import JointTorques
import numpy as np
import copy

def test_add_two_state():
	pos1 = np.random.rand(4,1)
	pos2 = np.random.rand(4,1)

	js1 = JointTorques("test_robot", 4, torques=pos1)
	js2 = JointTorques("test_robot", 4, torques=pos2)

	jsum = js1 + js2
	for i, q in enumerate(jsum.torques):
		assert (q - (js1.torques[i] + js2.torques[i]) < 1e-4)

def test_add_array():
	pos1 = np.random.rand(4,1)
	pos2 = np.random.rand(4,1)

	js1 = JointTorques("test_robot", 4, torques=pos1)

	jsum = js1 + pos2
	for i, q in enumerate(jsum.torques):
		assert (q - (js1.torques[i] + pos2[i]) < 1e-4)

	js1 += pos2
	for i, q in enumerate(jsum.torques):
		assert (q - js1.torques[i] < 1e-4)

	jusm = js1 + 1
	for i, q in enumerate(jsum.torques):
		assert (q - (js1.torques[i] + 1) < 1e-4)