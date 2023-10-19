import math
from math import pi

import numpy as np
from numpy import cos, sin

from constants import CONTROL_DT


def angle_diff(x, y):
    diff = np.arctan2(np.sin(x - y), np.cos(x - y))
    return diff


class RobotArm3dof:

    def __init__(self, l, velocity_constraint, reset_q=None, dt=CONTROL_DT):
        self.l = l  # link length
        if reset_q is not None:
            self.reset_q = reset_q.copy()
        else:
            self.reset_q = np.array([0.0, 0.0, 0.0])
        self.q = self.reset_q.copy()  # joint position
        self.lambda_coeff = 0.0001  # coefficient for robustness of singularity positions

        self.dt = dt
        self.end_p = np.zeros(2)  # end effector position (x,z)
        self.reset()
        # For visualization
        self.arm_regions = []
        self.velocity_constraint = velocity_constraint

    # forward kinematics (until the second elbow, i.e., secondary endpoint)
    def FK2(self):
        p = np.zeros([2])  # endpoint position
        '''*********** Student should fill in ***********'''
        l = self.l
        q = self.q
        p[0] = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1])
        p[1] = l[0] * sin(q[0]) + l[1] * sin(q[0] + q[1])
        '''*********** Student should fill in ***********'''
        return p

    # Jacobian matrix (until the second elbow, i.e., secondary endpoint)
    def Jacobian2(self):
        l = self.l
        q = self.q
        J = np.array(
            [[-l[0] * sin(q[0]) - l[1] * sin(q[0] + q[1]), -l[1] * sin(q[0] + q[1])],
             [l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]), l[1] * cos(q[0] + q[1])]])
        return J

    # forward kinematics (until the end of the chain, i.e., primary endpoint)
    def FK_all_points(self, other_q=None):
        l = self.l
        q = self.q
        if other_q is not None:
            q = other_q
        p1 = np.array([l[0] * cos(q[0]), l[0] * sin(q[0])])
        p2 = p1 + l[1] * np.array([cos(q[0] + q[1]), sin(q[0] + q[1])])
        if len(l) == 3:
            p3 = p2 + l[2] * np.array([cos(q[0] + q[1] + q[2]), sin(q[0] + q[1] + q[2])])
            return np.array([p1, p2, p3])
        return np.array([p1, p2])

    def FK_end_p(self):
        return self.FK_all_points()[-1]

    def Jacobian3(self):
        l = self.l
        q = self.q
        dxq1 = -l[1] * sin(q[0] + q[1])
        dyq1 = l[1] * cos(q[0] + q[1])
        dxq2 = -l[2] * sin(q[0] + q[1] + q[2])
        dyq2 = l[2] * cos(q[0] + q[1] + q[2])
        J = np.array(
            [[-l[0] * sin(q[0]) + dxq1 + dxq2, dxq1 + dxq2, dxq2],
             [l[0] * cos(q[0]) + dyq1 + dyq2, dyq1 + dyq2, dyq2]])
        return J

    # inverse kinematics (until joint 2)
    def IK2(self, p, q0):
        sol_q = np.zeros([2, 2])
        r = np.sqrt(p[0] ** 2 + p[1] ** 2)
        for i, _sign in enumerate([-1, 1]):
            sol_q[i, 1] = _sign * (pi - math.acos((self.l[0] ** 2 + self.l[1] ** 2 - r ** 2) / (2 * self.l[0] * self.l[1])))
            sol_q[i, 0] = math.atan2(p[1], p[0]) - _sign * math.acos((self.l[0] ** 2 - self.l[1] ** 2 + r ** 2) / (2 * self.l[0] * r))
        # Chooses the solution which is the closest to q0 (Thanks for the Tip Sebas :). I noticed the error after you mentioned it.)
        if abs(angle_diff(sol_q[0, 0], q0)) < abs(angle_diff(sol_q[1, 0], q0)):
            return sol_q[0]
        return sol_q[1]

    def get_dq(self, F_2, F_end):
        J2 = self.Jacobian2()
        J_end = self.Jacobian3()

        def J_robust(_J):
            _Jt = _J.transpose()
            damp_identity = self.lambda_coeff * np.identity(len(_J))
            return _Jt @ np.linalg.inv(_J @ _Jt + damp_identity)

        J_2_robust = J_robust(J2)
        J_end_robust = J_robust(J_end)
        null_space_velocity = np.concatenate((J_2_robust @ F_2, np.zeros(1)))
        null_space_control = (np.identity(len(J_end[0])) - J_end_robust @ J_end) @ null_space_velocity

        dq = J_end_robust @ F_end + null_space_control

        return dq

    def request_force_xz(self, F_2, F_end):
        """"
        F: float[2] the endpoint movement (x,z)
        """
        dq = self.get_dq(F_2, F_end)

        return self.move_joints(dq)

    def move_joints(self, dq):
        # constraint velocity
        for i, v in enumerate(self.velocity_constraint):
            if abs(dq[i]) > v:
                dq /= abs(dq[i]) / v
        self.q += dq * self.dt
        # joints back to -pi, pi
        self.q[self.q > pi] -= 2 * pi
        self.q[self.q < -pi] += 2 * pi
        self.end_p = self.FK_end_p()

        return self.end_p, self.q, dq

    def reset(self, joint_angles=None):
        if joint_angles is None:
            self.q = self.reset_q.copy()
        else:
            self.q = joint_angles.copy()

        self.end_p = self.FK_end_p()


def angle_to_pos(angle, length=1):
    return length * np.array([cos(angle), sin(angle)])


def get_angle(p1, p2):
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])


class PIDController:

    def __init__(self, kp=1, ki=0, kd=0, dims=2):
        self.i = 0  # loop counter
        self.se = 0.0  # integrated error
        self.state = []  # state vector
        self.last_value = np.zeros(dims)
        self.se = 0
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def control_step(self, current, goal, dt):
        error = goal - current
        d_p = current - self.last_value

        F_end = self.Kp * error + self.Ki * self.se * dt - self.Kd * d_p / dt

        self.se += error
        self.last_value = current

        return F_end