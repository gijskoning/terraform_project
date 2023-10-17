from math import cos, sin

import numpy as np

from sim_utils import length


def create_inner_waypoints(waypoints):
    inner_waypoints = [waypoints[0]]
    d_length = 0.05

    for i in range(len(waypoints)-1):
        p1 = waypoints[i]
        p2 = waypoints[i+1]
        length_waypoint = length(p1,p2)
        angle = np.arctan2(p2[1]-p1[1], p2[0]-p1[0])
        n = int(length_waypoint/d_length)+1
        for j in range(n):
            inner_waypoints.append(p1 + np.array([cos(angle), sin(angle)])*j*d_length)

    return inner_waypoints