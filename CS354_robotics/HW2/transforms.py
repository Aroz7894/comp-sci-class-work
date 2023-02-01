""" Functions for handling homogenous coordinate transforms in Python.

This code is based on 
'Introduction to Homogeneous Transformations & Robot Kinematics' 
by Jennifer Kay 2005.
http://elvis.rowan.edu/~kay/papers/kinematics.pdf

Author: Nathan Sprague
Version: 1/22/2013
"""

import numpy as np 

def trans(x, y, z):
    """ Create a translation matrix. """ 
    matrix = np.eye(4)
    matrix[:,3] = [x, y, z, 1.0]
    return matrix


def rot_x(theta):
    """ Create a rotation matrix around the x-axis. Theta is in radians. """
    matrix = np.eye(4)
    matrix[1,1] = np.cos(theta)
    matrix[1,2] = -np.sin(theta)
    matrix[2,1] = np.sin(theta)
    matrix[2,2] = np.cos(theta)
    return matrix
    

def rot_y(theta):
    """ Create a rotation matrix around the y-axis. Theta is in radians. """
    matrix = np.eye(4)
    matrix[0,0] = np.cos(theta)
    matrix[0,2] = np.sin(theta)
    matrix[2,0] = -np.sin(theta)
    matrix[2,2] = np.cos(theta)
    return matrix

def rot_z(theta):
    """ Create a rotation matrix around the z-axis. Theta is in radians. """
    matrix = np.eye(4)
    matrix[0,0] = np.cos(theta)
    matrix[0,1] = -np.sin(theta)
    matrix[1,0] = np.sin(theta)
    matrix[1,1] = np.cos(theta)
    return matrix

def section7_1():
    """ Calculations for the two-link arm example from section 7.1
    using the 'moving axes' method.  This code converts from gripper
    to world coordinates."""

    L1 = 1.2     # Link lengths
    L2 = 1.0
    theta = 0# Joint angle
    T1 = trans(L1, 0, 0) # Moves the world frame to the joint.
    R2 = rot_z(theta) # Rotates the world frame to line up with the joint.
    T2 = trans(L2, 0, 0) # Moves the world frame to the gripper.
    T = np.dot(T1, np.dot(R2, T2)) # Multiply to get the final transform

    print "Transform matrix for joint at 0 degrees: "
    print T
    print "Gripper (0,0,0) in world coordinates: "
    print np.dot(T, np.array([0, 0, 0, 1]))


    # Redo the calcuations for a different joint angle...
    theta = np.pi/2 
    R2 = rot_z(theta) 
    T = np.dot(T1, np.dot(R2, T2)) 

    print "\nTransform matrix for joint at 90 degrees: "
    print T
    print "Gripper (0,0,0) in world coordinates: "
    print np.dot(T, np.array([0, 0, 0, 1]))


if __name__ == "__main__":
    section7_1()
