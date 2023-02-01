"""
Code to move the turtle from his position to a 'goal position' in the turtlesim environment.

Author: Andrew Rozniakowski
Version: 2/2/15
"""

import numpy as np
import rospy
import turtlesim
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Twist is the message type for sending movement commands.
from geometry_msgs.msg import Twist

goal_pos = (10, 17);

def trans(x, y, z):
    """ Create a translation matrix. """
    matrix = np.eye(4)
    matrix[:, 3] = [x, y, z, 1.0]
    return matrix

def rot_z(theta):
    """ Create a rotation matrix around the z-axis. Theta is in radians. """
    matrix = np.eye(4)
    matrix[0, 0] = np.cos(theta)
    matrix[0, 1] = -np.sin(theta)
    matrix[1, 0] = np.sin(theta)
    matrix[1, 1] = np.cos(theta)
    return matrix

def world_to_turtle(point_x, point_y, turtle_x, turtle_y, turtle_theta):
    """ Transform a point from world to turtle coordinates.

    The provided point will be transformed into the turtle's
    coordinate frame.  In that coordinate frame, (0, 0) corresponds to
    the location of the turtle, (1.0, 0) is the spot 1 meter ahead of
    the turtle and (0, 1.0) is the spot 1 meter to the left of the
    turtle.

    Parameters:
       point_x, point_y   - the point to transfrom (in world coordinates).
       turtle_x, turtle_y - the location of the turtle.
       turtle_theta       - the rotation of the turtle (in radians)

    Returns:
       (x, y) - coordinates of the provided point in
                the turtle's coordinate frame.

    Example:
    >>> x, y = world_to_turtle(0, 0, 0, 1, 0)
    >>> print x, y
    0.0 -1.0

    """
    point = [point_x, point_y, 0.0, 1.0]
    R1 = rot_z(-turtle_theta)
    T1 = trans(-turtle_x, -turtle_y, 0)
    T = np.dot(R1, T1)
    result = np.dot(T, point)
    return result[0], result[1]


#def demo():
    """ Short demo of using world_to_turtle. """

    # The turtle is at (0, 0)
    #turtle_x = 0.0
    #turtle_y = 0.0

    # Pointing North
    #turtle_theta = np.pi / 2.0

    # The point we are interested in is at (1.0, 1.0)
   # point_x = 1.0
  #  point_y = 1.0

    # The point of should be at (1, -1) from the point of view
    # of the turtle: one meter in front, and one meter to the right.
 #   print world_to_turtle(point_x, point_y, turtle_x, turtle_y, turtle_theta)


def start():
    
    # Turn this into an official ROS node named approach
    rospy.init_node('turtle_nav')

    # Subscribe to the /scan topic.  From now on
    # scan_callback will be called every time a new scan message is
    # published.
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Create a publisher object for sending Twist messages to the
    # turtlebot_node's velocity topic. 
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist) 

    # Create a twist object. 
    # twist.linear.x represents linear velocity in meters/s.
    # twist.angular.z represents rotational velocity in radians/s.
    twist = Twist()

    # Wait until the first scan is available.
    while SCAN is None and not rospy.is_shutdown():
        rospy.sleep(.1)

    # Try to maintain this target distance to the wall.
    target = 1

    # Rate object used to make the main loop execute at 10hz.
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():

        # Back up if the scan is bad, or if we are too close. 
        if  SCAN.ranges[320] < target:
            twist.linear.x = -0.5  # forward velocity in meters/second
            twist.angular.z = 0    # rotation in radians/second
            rospy.sleep(.1)
            twist.linear.x =  0 
            twist.angular.z = 1.5 
        elif math.isnan(SCAN.ranges[320]):
            twist.linear.x = .3
            twist.angular.z = .1  
        else:
            twist.linear.x = 0.5
            twist.angular.z = 0  

        print "Range: {:.3f}".format(SCAN.ranges[320])

        vel_pub.publish(twist) # These velocities will be applied for .6 seconds
                               # unless another command is sent before that. 
        rate.sleep()           # Pause long enough to maintain correct rate.
        


if __name__ == "__main__":
    start()
