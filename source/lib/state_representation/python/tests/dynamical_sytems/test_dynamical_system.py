#!/usr/bin/python
from dynamical_systems.linear_dynamical_system import LinearDynamicalSystem

from state_representation.cartesian.cartesian_pose import CartesianPose

import numpy as np

dim = 3

def test_ds_position():
    target_pose = CartesianPose("attr", 10*np.random.rand(3))
    # target_pose = CartesianPose("attr", position=[0,0,0])
    linear_ds = LinearDynamicalSystem(gain=1, attractor=target_pose)
    
    current_pose = CartesianPose("robot", 10*np.random.rand(3))
    
    ds_linear = LinearDynamicalSystem()
    
    nb_steps = 100
    dt = 0.1

    max_vel = 0.2

    dist_target  = current_pose - target_pose
    for ii in range(nb_steps):
        vel_twist = linear_ds.evaluate(current_pose)
        vel_twist.clamp(max_vel)
        
        current_pose += dt*vel_twist
        dist_target_new = current_pose - target_pose
        
        assert all (np.abs(dist_target.position) > np.abs(dist_target_new.position))
        dist_target = dist_target_new

    print("Current Pose: \n{}".format(current_pose))
    print("Goal Pose: \n{}".format(target_pose))

                                     
def test_ds_orientation():
    # Not needed/implemented yet
    pass
        
                            
if __name__=="__main__":
    test_ds_position()
