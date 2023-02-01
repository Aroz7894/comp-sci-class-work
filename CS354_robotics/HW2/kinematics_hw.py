""" CS480 HW2 Python Exercises

Author: Andrew Rozniakowski
Version: 2/6/2015
"""

#import numpy as np
from transforms import trans, rot_x, rot_y, rot_z


def gripper_to_base(point, theta_s, theta_e):
    """Convert the indicated point from gripper coordinates to base
    coordinates given the indicated joint angles (in radians).

    Arguments:
          point - length-three numpy array containing the coordinates
                  of a point in the gripper coordinate frame.
          theta_s - rotation (in radians) of the shoulder joint.
          theta_e - rotation (in radians) of the elbow joint.

    Returns:
          A length-three numpy array representing the position of the
          point in the base coordinate frame.

    """

    L1 = .8
    L2 = .15
    L3 = .18

    np.append(point, 1)
    
    point_ans = trans(0, 0, L3) * rot_z(theta_e) * trans(0, 0, L2) * rot_z(theta_s) * trans(0, 0 , L1) * point 
   
    return point_ans

def base_to_gripper(point, theta_s, theta_e):
    """
    Convert the indicated point from base coordinates to gripper
    coordinates given the indicated joint angles (in radians).

    Arguments:
          point - length-three numpy array containing the coordinates
                  of a point in the base coordinate frame.
          theta_s - rotation (in radians) of the shoulder joint.
          theta_e - rotaion (in radians) of the elbow joint.

    Returns:
          A length-three numpy array representing the position of the
          in the gripper coordinate frame.
    """

    L1 = .8
    L2 = .15
    L3 = .18

    np.append(point, 1)
    
    point_ans = trans(0, 0, L1) * rot_z(theta_s) * trans(0, 0, L2) * rot_z(theta_e) * trans(0, 0 , L3) * point   

    return point_ans 

def tests():
    """ Recalculate the results from the homework. """

    print "theta_s = 0, theta_e = 0"
    print "(0, 0, 0)_g in base coordinates: "
    print gripper_to_base(np.array([0, 0, 0]), 0, 0)
    print "(0, 0, 0)_b in gripper coordinates: "
    print base_to_gripper(np.array([0, 0, 0]), 0, 0)

    print "\ntheta_s = 90 degrees, theta_e = 0"
    print "(0, 0, 0)_g in base coordinates: "
    print gripper_to_base(np.array([0, 0, 0]), np.pi/2.0, 0)
    print "(0, 0, 0)_b in gripper coordinates: "
    print base_to_gripper(np.array([0, 0, 0]), np.pi/2.0, 0)

    print "\ntheta_s = 90 degrees, theta_e = 90 degrees"
    print "(0, 0, 0)_g in base coordinates: "
    print gripper_to_base(np.array([0, 0, 0]), np.pi/2.0, np.pi/2.0)
    print "(0, 0, 0)_b in gripper coordinates: "
    print base_to_gripper(np.array([0, 0, 0]), np.pi/2.0, np.pi/2.0)
    print "(.4, 0, 0)_g in base coordinates: "
    print gripper_to_base(np.array([.4, 0, 0]), np.pi/2.0, np.pi/2.0)
    print "(.4, 0, 0)_b in gripper coordinates: "
    print base_to_gripper(np.array([.4, 0, 0]), np.pi/2.0, np.pi/2.0)

if __name__ == "__main__":
    tests()
