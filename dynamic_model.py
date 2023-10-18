import numpy as np
import math

from numpy import sin, cos
from math import pi
from sim_utils import arm_to_polygon, check_collision
from constants import ARMS_LENGTHS, ARM_WIDTH, CONTROL_DT


class RobotArm3dof:

    def __init__(self, l, velocity_constraint, reset_q=None, dt=CONTROL_DT):
        self.l = l  # link length
        if reset_q is not None:
            self.reset_q = reset_q.copy()
        else:
            self.reset_q = np.array([0.0, 0.0, 0.0])
        self.q = self.reset_q.copy()  # joint position
        # self.dq = np.array([0.0, 0.0, 0.0])  # joint velocity
        self.tau = np.array([0.0, 0.0, 0.0])  # joint torque
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
    def IK2(self, p):
        q = np.zeros([2])
        r = np.sqrt(p[0] ** 2 + p[1] ** 2)
        q[1] = pi - math.acos((self.l[0] ** 2 + self.l[1] ** 2 - r ** 2) / (2 * self.l[0] * self.l[1]))
        q[0] = math.atan2(p[1], p[0]) - math.acos((self.l[0] ** 2 - self.l[1] ** 2 + r ** 2) / (2 * self.l[0] * r))

        return q

    def get_dq(self, F_2, F_end):
        """"
        F: float[2] the endpoint movement
        """
        # KINEMATIC CONTROL
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

        for i, v in enumerate(self.velocity_constraint):
            if abs(dq[i]) > v:
                dq /= abs(dq[i]) / v
        return dq

    def request_force_xz(self, F_2, F_end):
        """"
        F: float[2] the endpoint movement (x,z)
        """
        dq = self.get_dq(F_2, F_end)

        return self.move_joints(dq)

    def move_joints(self, dq):
        """"
        F: float[2] the endpoint movement (x,z)
        """
        # dq = self.constraint(dq)
        self.q += dq * self.dt
        self.end_p = self.FK_end_p()

        return self.end_p, self.q, dq

    def reset(self, joint_angles=None):
        if joint_angles is None:
            self.q = self.reset_q.copy()
        else:
            self.q = joint_angles.copy()

        self.end_p = self.FK_end_p()

    def constraint(self, dq):
        dq = dq.copy()
        for i in range(len(dq)):
            c = self.velocity_constraint[i]
            dq[i] = np.clip(dq[i], -c, c)
        return dq

    def old_constraint(self, dq):
        global_pos_constraint_lb = [-10, -0.1]  # lower bound global constraint
        p = np.zeros(2)
        self.arm_regions = []

        def create_obstacles(joint_pos_new, q):
            obstacles = []
            for i in range(len(dq)):
                l = ARMS_LENGTHS[i]
                if i > 0:
                    p = joint_pos_new[i - 1]
                else:
                    p = np.zeros(2)
                obstacles.append(arm_to_polygon(*p, np.sum(q[:i + 1]), l, ARM_WIDTH))
            return obstacles

        new_q = self.q + dq * self.dt
        # Check for base arm to not hit the base
        if new_q[0] < 0.1 * np.pi:
            dq[0] = 0
        # Other checks on all arms
        for i in range(len(dq)):
            new_q = self.q + dq * self.dt
            joint_positions_new = self.FK_all_points(new_q)

            # Global constraint check
            if np.any(joint_positions_new < global_pos_constraint_lb):
                dq[i] = 0
            # Check collision with itself
            if i > 0:
                p = joint_positions_new[i - 1].copy()
            if not check_collision(create_obstacles(joint_positions_new, new_q)):
                dq[i] = 0

            # for visualization
            l = ARMS_LENGTHS[i]
            pol = arm_to_polygon(*p, np.sum(self.q[:i + 1]), l, ARM_WIDTH)
            self.arm_regions.append(pol)

        return dq

def angle_to_pos(angle, length=1):
    return length*np.array([cos(angle), sin(angle)])
def get_angle(p1, p2):
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])


class PIDController:

    def __init__(self, kp=1, ki=0, kd=0, dims=2):
        self.i = 0  # loop counter
        self.se = 0.0  # integrated error
        self.state = []  # state vector
        self.last_p_end = np.zeros(dims)
        self.se = 0
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def control_step(self, p_end, goal, dt):
        pos_goal = goal[:2]
        # KINEMATIC CONTROL
        error = pos_goal - p_end
        d_p = p_end - self.last_p_end

        F_end = self.Kp * error + self.Ki * self.se * dt - self.Kd * d_p / dt

        self.se += error
        self.last_p_end = p_end

        return F_end

    # def control_step_angle(self, angle, goal_angle, dt):
    #     diff_angle = goal_angle - angle
    #     if diff_angle > pi:
    #         diff_angle -= 2 * pi
    #     if diff_angle < -pi:
    #         diff_angle += 2 * pi
    #     error = diff_angle
    #     d_p = angle - self.last_p_end
    #
    #     F_end = self.Kp * error + self.Ki * self.se * dt - self.Kd * d_p / dt
    #
    #     self.se += error
    #     self.last_p_end = angle
    #
    #     return F_end


if __name__ == '__main__':
    model = RobotArm3dof(l=ARMS_LENGTHS)
    controller = PIDController()