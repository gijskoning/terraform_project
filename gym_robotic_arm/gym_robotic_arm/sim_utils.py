import math

import numpy as np
from math import cos, sin


def length(p1, p2=None):
    if p2 is None:
        p2 = [0, 0]
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def state_rotate_pygame(x, y, yaw):
    # Yaw in radians required
    return np.array([x * cos(yaw) + y * sin(yaw), -x * sin(yaw) + y * cos(yaw)])


def state_rotate(x, y, yaw):
    # Yaw in radians required
    return np.array([x * cos(yaw) + y * sin(yaw), x * sin(yaw) + y * cos(yaw)])


def config_to_polygon(x, y, yaw, length, width):
    start = np.array([x, y])
    polygon = [
        start + state_rotate(x=0.0, y=width / 2, yaw=yaw),
        start + state_rotate(x=0.0, y=-width / 2, yaw=yaw),
        start + state_rotate(x=length, y=-width / 2, yaw=yaw),
        start + state_rotate(x=length, y=width / 2, yaw=yaw)]
    return polygon


def arm_to_polygon(x, y, yaw, length, width):
    additional_length_arm = 0.05  # In meters

    x, y = np.array([x, y]) + state_rotate_pygame(x=-additional_length_arm / 2, y=0, yaw=-yaw)
    return config_to_polygon_pygame(x, y, yaw, length + additional_length_arm, width)


def config_to_polygon_pygame(x, y, yaw, length, width):
    start = np.array([x, y])
    yaw *= -1
    polygon = [
        start + state_rotate_pygame(x=0.0, y=width / 2, yaw=yaw),
        start + state_rotate_pygame(x=0.0, y=-width / 2, yaw=yaw),
        start + state_rotate_pygame(x=length, y=-width / 2, yaw=yaw),
        start + state_rotate_pygame(x=length, y=width / 2, yaw=yaw)]
    return polygon


# def config_to_polygon(x, y, yaw, length, width):
#     x, y = [x, y] + state_rotate(length / 2, 0, -yaw)
#     return config_to_polygon_from_center(x, y, yaw, length, width)

def check_collision(arm_list):  # called for every new possible node
    for i in range(len(arm_list)):  # for each (x,y,yaw) config step in the proposed path to the new node
        # car = config_to_polygon(config)
        arm1 = arm_list[i]
        for j in range(i + 2, len(arm_list)):
            if collide(arm1, arm_list[j]):  # check if the two passed polygons collide/overlap
                # if there is collision for one config in path
                return False  # path is invalid
    return True  # if not a single collision along path, path is safe


def collide(p1, p2):
    '''
    Return True if the shapes collide. Otherwise, return False

    p1 and p2 are lists of ordered pairs, the vertices of the polygons in the
    counterclockwise direction.
    '''

    edges = edges_of(p1)
    edges += edges_of(p2)
    for e in edges:
        o = orthogonal(e)
        separates = is_separating_axis(o, p1, p2)
        if separates:
            # they do not collide
            return False
    return True  # they do collide


def edges_of(vertices):
    """
    Return the vectors for the edges of the polygon p.

    p is a polygon.
    """
    N = len(vertices)

    edges = [vertices[(i + 1) % N] - vertices[i] for i in range(N)]

    return edges


def orthogonal(v):
    """
    Return a 90 degree clockwise rotation of the vector v.
    """
    return np.array([-v[1], v[0]])


def is_separating_axis(o, p1, p2):
    """
    Return True if o is a separating axis of p1 and p2.
    Otherwise, return False
    """
    min1, max1 = float('+inf'), float('-inf')
    min2, max2 = float('+inf'), float('-inf')

    for v in p1:
        projection = v @ o

        min1 = min(min1, projection)
        max1 = max(max1, projection)

    for v in p2:
        projection = v @ o

        min2 = min(min2, projection)
        max2 = max(max2, projection)

    if max1 >= min2 and max2 >= min1:
        return False
    else:
        return True
