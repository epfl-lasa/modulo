#!/usr/bin/python
from state_representation.cartesian.cartesian_state import CartesianState
from state_representation.cartesian.cartesian_pose import CartesianPose
# from state_representation.cartesian.cartesian_twist import CartesianTwist
# from state_representation.cartesian.cartesian_Wrench import CartesianWrench

import numpy as np
import copy

import quaternion

def quat_move_w_to_end(q):
    return np.hstack((q[1:], q[0]))

def quat_move_w_to_front(q):
    return np.hstack((q[3], q[:3]))

def quat_mul_vec_w(a, b, change_vector_angle_order=True):
    if change_vector_angle_order:
        a = quat_move_w_to_end(a)
        b = quat_move_w_to_end(b)
    
    res = np.zeros(4)
    res[0] =  a[1]*b[2] - a[2]*b[1] + a[0]*b[3] + a[3]*b[0]
    res[1] =  a[2]*b[0] - a[0]*b[2] + a[1]*b[3] + a[3]*b[1]
    res[2] =  a[0]*b[1] - a[1]*b[0] + a[2]*b[3] + a[3]*b[2]
    res[3] = -a[0]*b[0] - a[1]*b[1] - a[2]*b[2] + a[3]*b[3]
    if (res[3]<0):
        res *= -1

    if change_vector_angle_order:
        res = quat_move_w_to_front(res)

    return res
    
def quat_mul(a, b):
    res = np.zeros(4)
    # res[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    # res[1] = a[0]*b[1] + a[1]*b[0] - a[2]*b[3] + a[3]*b[2];
    # res[2] = a[0]*b[2] + a[1]*b[3] + a[2]*b[0] - a[3]*b[1];
    # res[3] = a[0]*b[3] - a[1]*b[2] + a[2]*b[1] + a[3]*b[0];
    # print('r', a.shape)
    # print('q', b.shape)
    res[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    res[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    res[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    res[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    if (res[0]<0):
        res *= -1;
    return res;


def test_negate_quaternion():
    q = quaternion.as_quat_array(np.random.rand(4))
    q2 = -q

    assert (q.w == -q2.w)
    for ii in range(3):
        assert (q.vec[ii]==-q2.vec[ii])
        
def test_quaternion_mulitply():
    q1 = quaternion.as_quat_array(np.random.rand(4))
    q2 = quaternion.as_quat_array(np.random.rand(4))

    qres = q1*q2
    if(qres.components[0] < 0):
        qres = -qres
    print("qres:", qres.components)
    # res_init = quat_mul(q1.components, q2.components)
    res = quat_mul(q1.components, q2.components)
    print("res", res)
    
    for ii in range(4):
        assert (qres.components[ii]==res[ii])

    q3 = -q2
    qres = q1*q3
    if(qres.components[0] < 0):
        qres = -qres
    print("qres:", qres.components)
    res = quat_mul(q1.components, q3.components)
    print("res", res)
    for ii in range(4):
        assert (qres.components[ii]==res[ii])

def test_multiply_transforms_both_operators():
    pos1 = np.array([1,2,3])
    rot1 = quaternion.as_quat_array([1,0,0,0])
    tf1 = CartesianPose("t1", pos1, rot1)

    pos2 = np.array([4,5,6])
    rot2 = quaternion.as_quat_array([1,0,0,0])
    tf2 = CartesianPose("t2", pos2, rot2, "t1")

    tf3 = tf1 * tf2
    tf1 *= tf2

    print("name t3", tf3.name)
    
    for ii in range(tf1.position.shape[0]):
        assert (np.abs(tf1.position[ii]-tf3.position[ii]) < 0.00001)

def test_multiply_transoforms_same_orientation():
    pos1 = np.array([1,2,3])
    rot1 = quaternion.quaternion(1,0,0,0)
    tf1 = CartesianPose("t1", pos1, rot1)

    pos2 = np.array([4,5,6])
    rot2 = quaternion.as_quat_array([1,0,0,0])
    tf2 = CartesianPose("t2", pos2, rot2, "t1")

    tf1 *= tf2
    
    print("position", tf1.position)
    print("orientation", tf1.orientation)

    pos_truth = np.array([5, 7, 9])
    
    for ii in range(tf1.position.shape[0]):
        assert (np.abs(tf1.position[ii]-pos_truth[ii]) < 0.00001)

def test_multiply_transofrms_difference_orientation():
    pos1 = np.array([1,2,3])
    rot1 = quaternion.quaternion(0.70710678, 0.70710678, 0., 0.)
    tf1 = CartesianPose("t1", pos1, rot1)

    pos2 = np.array([4,5,6])
    rot2 = quaternion.quaternion(0., 0., 0.70710678, 0.70710678)
    tf2 = CartesianPose("t2", pos2, rot2, "t1")

    tf1 *= tf2

    pos_truth = np.array([5, -4, 8])
    rot_truth = quaternion.quaternion(0., 0., 0., 1.)

    print("position", tf1.position)
    print("orientation", tf1.orientation)
    
    for ii in range(tf1.position.shape[0]):
        assert (np.abs(tf1.position[ii]-pos_truth[ii]) < 0.00001)

    for ii in range(rot_truth.components.shape[0]):
        assert ((tf1.orientation.components[ii]-rot_truth.components[ii]) < 1e-5)

    # assert (np.abs(np.norm(tf1.orientation[ii]*rot_truth)-1) < 1e-4)
    
def test_inverse_null_orientation():
    pos1 = np.array([1,2,3])
    rot1 = quaternion.quaternion(1, 0, 0, 0)
    tf1 = CartesianPose("t1", pos1, rot1)

    tf1 = tf1.inverse()

    pos_truth = np.array([-1, -2, -3])
    rot_truth = quaternion.quaternion(1., 0., 0., 0.)
    
    print("position", tf1.position)
    print("orientation", tf1.orientation)
    
    assert(tf1.name == "world")
    assert(tf1.reference_frame == "t1")

    for ii in range(pos_truth.shape[0]):
        assert ((tf1.position[ii]-pos_truth[ii]) < 1e-5)

    for ii in range(rot_truth.components.shape[0]):
        assert ((tf1.orientation.components[ii]-rot_truth.components[ii]) < 1e-5)


def test_inverse_non_null_orientation():
    pos1 = np.array([1,2,3])
    rot1 = quaternion.quaternion(0.70710678, 0.70710678, 0., 0.)
    tf1 = CartesianPose("t1", pos1, rot1)

    tf1 = tf1.inverse()

    pos_truth = np.array([-1,-3,2])
    rot_truth = quaternion.quaternion(0.70710678, -0.70710678, 0., 0.)

    print("position", tf1.position)
    print("position_true", pos_truth)
    print("orientation", tf1.orientation)
    print("orientation", rot_truth)

    for ii in range(pos_truth.shape[0]):
        assert (np.abs(tf1.position[ii]-pos_truth[ii])<1e-5)

    for ii in range(rot_truth.components.shape[0]):
        assert ((tf1.orientation.components[ii]-rot_truth.components[ii]) < 1e-5)


def test_multiply_inverse_non_null_orientation():
    pos1 = np.array([1,2,3])
    rot1 = quaternion.quaternion(0.70710678, 0.70710678, 0., 0.)
    tf1 = CartesianPose("t1", pos1, rot1)

    tf1 *= tf1.inverse()

    pos_truth = np.array([0, 0, 0])
    rot_truth = quaternion.quaternion(1., 0, 0, 0)

    print("position", tf1.position)
    print("position_true", pos_truth)
    print("orientation", tf1.orientation)
    print("orientation", rot_truth)

    for ii in range(pos_truth.shape[0]):
        assert (np.abs(tf1.position[ii]-pos_truth[ii])<1e-5)

    for ii in range(rot_truth.components.shape[0]):
        assert ((tf1.orientation.components[ii]-rot_truth.components[ii]) < 1e-5)

def test_add_two_poses():
    pos1 = np.zeros(3)
    rot1 = quaternion.quaternion(0.70710678, 0.70710678, 0., 0.)
    tf1 = CartesianPose("t1", pos1, rot1)

    tf1 *= tf1.inverse()

    pos_truth = np.array([0, 0, 0])
    rot_truth = quaternion.quaternion(1., 0, 0, 0)

    print("position", tf1.position)
    print("position_true", pos_truth)
    print("orientation", tf1.orientation)
    print("orientation", rot_truth)

    for ii in range(pos_truth.shape[0]):
        assert (np.abs(tf1.position[ii]-pos_truth[ii])<1e-5)

    for ii in range(rot_truth.components.shape[0]):
        assert ((tf1.orientation.components[ii]-rot_truth.components[ii]) < 1e-5)
        

def test_add_displacement():
    # TODO -- how control these
    pos1 = np.zeros(3)
    rot1 = quaternion.quaternion(1, 0, 0., 0.)
    tf1 = CartesianPose("t1", pos1, rot1)

    vel = CartesianTwist("t1")
    vel.linear_velocity = np.array([0.1, 0.1, 0.1])
    vel.angular_velocity = np.array([0.1, 0.1, 0])

    dt = 10.0
    print(tf1 + tf2*dt)

    dt = 1000.0
    print(tf1 + dt*tf2)

    dt = 1.0
    print(tf1 + dt*tf2)
    

def test_pose_to_velocity():
    # TODO -- how control these
    pos1 = np.zeros(3)
    rot1 = quaternion.quaternion(1, 0, 0., 0.)
    tf1 = CartesianPose("t1", pos1, rot1)

    pos2 = np.array([1, 0, 0])
    rot2 = quaternion.quaternion(0, 1, 0., 0.)
    tf2 = CartesianPose("t2", pos2, rot2)
    
    dt = 1.0
    print((tf1-tf2)/dt)

    dt = 10.0
    print((tf1-tf2)/dt)

    dt = 100.0
    print((tf1-tf2)/dt)

    
def test_implicit_conversion():
    # TODO -- how control these
    pos1 = np.zeros(3)
    rot1 = quaternion.quaternion(1, 0, 0., 0.)
    tf1 = CartesianPose("t1", pos1, rot1)

    vel = CartesianTwist("t1")
    vel.linear_velocity = np.array([0.1, 0.1, 0.1])
    vel.angular_velocity = np.array([0.1, 0.1, 0])

    tf1 += vel

    print(tf1)

def test_velocity_clamping():
    vel = CartesianTwist("test", np.array([1, -2, 3]), np.array([1, 2, -3]))
    vel.clapm(1, 0.5)

    print(vel)

    assert(np.linalg.norm(vel.linear_velocity) <= 1)
    assert(np.linalg.norm(vel.linear_velocity) <= 0.5)

    print(vel.clamped(1, 0.5, 0.1, 0.1))

    for ii in range(3):
        assert (vel.clamped(1, 0.5, 0.1, 0.1).linear_velocity[ii] == 0)
        assert (vel.clamped(1, 0.5, 0.1, 0.1).angular_velocity[ii] == 0)
    
def test_pose_distance():
    # TODO
    # what is <<dist>>
    p1 = CartesianPose("test", np.zeros(3))
    p2 = CartesianPose("test", np.array([1, 0, 0]))
    p3 = CartesianPose("test", np.array([1, 0, 0]), quaternion.quaternion(0, 1, 0, 0) )

    # d1 = dist(p1, p2)
    d2 = p1.dist(p2)

    assert(d1-d2 < 1e-5)
    assert(d1[0] == 1)
    assert(d1[1] == 0)

    # d3 = dist(p1, p3)
    # assert( abs(d3[1]-3.14159) < 1e-3)

def test_twist_operators():
    # TODO
    # vec = np.random.random(6,)
    # twist = CartesianTwist("test")
    
    # twist = vec
    # assert(vec-np.linalg.norm(twist.twist) < 1e-4 )
    pass


def test_wrench_operators_with_eigen():
    # TODO
    pass


def test_filter():
    tf1 = CartesianPose("t1", np.random.rand(3,), quaternion.as_quat_array(np.random.rand(4,)))
    tf2 = CartesianPose("t1", np.random.rand(3,), quaternion.as_quat_array(np.random.rand(4,)))

    for ii in range(1000):
        temp = copy.deepcopy(tf1)
    
        alpha = 0.1

        tf1 = (1-alpha)*tf1 + alpha*tf2

        diff = tf1.position - (1-alpha)*temp.position + alpha*tf2.position
        assert (np.linalg.norm(diff) < 1e-4)

    assert (dist(tf1, tf2)[0] < 1e-4)
