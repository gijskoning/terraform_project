# SIMULATION PARAMETERS
from math import sin, cos

import matplotlib.pyplot as plt
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_SPACE, K_UP, K_DOWN, K_a, K_w, K_z, K_s, K_x, K_c, K_d, K_r

from constants import ARMS_LENGTHS, Kp, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, CONTROL_DT, goal_reached_length, velocity_constraint, Ki, Kd
from dynamic_model import PIDController, RobotArm3dof
import shelve

from main import Planner
from plot import plot_dq_constraints
from sim_utils import length
from user_input import gripperControl, keyboard_control
from visualize_robot_arm import Display, display_to_coordinate

dt = CONTROL_DT
# ROBOT     PARAMETERS
x0 = 0.0  # base x position
y0 = 0.0  # base y position

global_db = shelve.open("cache_angles")


# rrt* steps
# sample new node within a distance from one of the nodes
# node_i = randint
# dx,dy = rand(),rand()
# new_node = node_i + [dx,dy]
# this would require to calculate the inverse kinematics.
# And not sure if we can nicely put all the constraints like angle constraints in here.


# or with angles
# dtheta = rand.choice([-1,0,1])
# new_node = node_i + dtheta
# and then you want this to be in velocities
# for one next step, you have 3 angle samples, 27 possibilities
# can prefer the same movement, or increase prob, for similar velocities.
#  which is actually the same as sampling acceleration per angle


class PlannerRRTAngle:

    def __init__(self, waypoints=None):
        if waypoints is None:
            waypoints = []

        self.waypoints = waypoints
        self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints)[:,:2])
        self.goal_i = 0
        self.finished = False

    def step(self, current_pos, end_angle):
        if len(self.inner_waypoints) <= 1:
            return [*current_pos, end_angle]
        pos_goal = self.inner_waypoints[self.goal_i]

        dist_to_goal = length(current_pos, pos_goal)
        if dist_to_goal < goal_reached_length:
            if self.goal_i < len(self.inner_waypoints)-1:
                self.goal_i += 1
            else:
                self.finished = True
                print("Goal reached")
        return [*pos_goal, 0.] # todo

    def add_waypoint(self, goal):
        self.waypoints.append(goal)
        # self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints))
        self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints)[:,:2])

    def reset_waypoints(self):
        self.waypoints = []
        self.inner_waypoints = []

    def create_inner_waypoints(self, waypoints):
        if len(waypoints) == 0:
            return []
        inner_waypoints = [waypoints[0]]
        d_length = 0.05

        for i in range(len(waypoints) - 1):
            p1 = waypoints[i,:2]
            p2 = waypoints[i + 1,:2]
            length_waypoint = length(p1, p2)
            angle = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            n = int(length_waypoint / d_length) + 1
            for j in range(n):
                inner_waypoints.append(p1 + np.array([cos(angle), sin(angle)]) * j * d_length)

        return inner_waypoints # todo maybe need inner waypoints?

if __name__ == '__main__':
    waypoints = []
    if 'waypoints' in global_db:
        waypoints = global_db['waypoints']
    robot_base = np.array([0, ZERO_POS_BASE])

    robot_arm = RobotArm3dof(ARMS_LENGTHS, velocity_constraint, reset_q=INITIAL_CONFIG_Q)
    local_endp_start = robot_arm.end_p
    q = robot_arm.q
    controller = PIDController(kp=Kp, ki=Ki, kd=Kd)
    t = 0.0  # time

    state = []  # state vector
    end_pos = robot_base + local_endp_start
    end_angle = sum(q)

    planner = Planner([np.array([*end_pos,end_angle])]+waypoints)

    # goal = robot_base + local_endp_start
    gripper = [100, 100]

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)
    step = 0
    was_click = False
    mouse_released = False
    enable_robot = False
    should_run = True
    should_save = False
    new_click_pos = None
    while not planner.finished and should_run:
        goal = planner.step(end_pos, end_angle)
        goal_pos = goal[:2]

        display.render(q, goal, planner.waypoints, planner.inner_waypoints)  # RENDER
        mouse_x_display, mouse_y_display = pygame.mouse.get_pos()
        mouse_x, mouse_y = display_to_coordinate(mouse_x_display, mouse_y_display)
        on_left_click = pygame.mouse.get_pressed()[0]
        if not on_left_click and was_click:
            mouse_released = True
        if on_left_click and not was_click:
            # new click
            new_click_pos = [mouse_x, mouse_y]
            new_waypoint = np.array(new_click_pos+[0.])
            planner.add_waypoint(new_waypoint)
        if on_left_click:
            second_click_pos = [mouse_x, mouse_y]
            angle = np.arctan2(second_click_pos[1] - new_click_pos[1], second_click_pos[0] - new_click_pos[0])
            new_waypoint = np.array(second_click_pos+[angle])
            planner.waypoints[-1][2] = angle
        # USER CONTROL STUFF
        gripper = gripperControl(gripper)

        new_keyboard_goal, dq_keyboard = keyboard_control(dt, goal_pos)

        keys = pygame.key.get_pressed()
        if keys[K_SPACE]:
            enable_robot = not enable_robot
            print("enable_robot", enable_robot)

        # Control
        local_goal_pos = goal_pos - robot_base
        local_goal = local_goal_pos + [goal[2]]
        # F_end can be replaced with RL action. array[2]
        if enable_robot:
            # gets the end effector goal
            F_end = controller.control_step(robot_arm.FK_end_p(), local_goal, dt)
            # F_end[1] = 0
            if dq_keyboard is None:
                end_pos, q, dq = robot_arm.request_endpoint_force_xz(F_end)  # this requests a endpoint force and returns pos, angle,angle_speed
            else:
                # not used for goals
                end_pos, q, dq = robot_arm.move_joints(dq_keyboard)
                # Set goal exactly to current endpoint
                goal = end_pos + robot_base
            # save state
            if len(q) == 3:
                state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], end_pos[0], end_pos[1]])
            else:
                state.append([t, q[0], q[1], dq[0], dq[1], end_pos[0], end_pos[1]])
        t += dt

        # try to keep it real time with the desired step time
        display.tick()
        pygame.display.flip()  # update display
        step += 1
        was_click = on_left_click
        mouse_released = False

    # plot_dq_constraints(state)
    # plt.show()