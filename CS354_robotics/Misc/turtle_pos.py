"""
Code for handling coordinate transforms in the turtlesim environment.

Author: Nathan Sprague
Version: 1/25/15
"""

import numpy as np

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


def demo():
    """ Short demo of using world_to_turtle. """

    # The turtle is at (0, 0)
    turtle_x = 0.0
    turtle_y = 0.0

    # Pointing North
    turtle_theta = np.pi / 2.0

    # The point we are interested in is at (1.0, 1.0)
    point_x = 1.0
    point_y = 1.0

    # The point of should be at (1, -1) from the point of view
    # of the turtle: one meter in front, and one meter to the right.
    print world_to_turtle(point_x, point_y, turtle_x, turtle_y, turtle_theta)


if __name__ == "__main__":
    demo()
