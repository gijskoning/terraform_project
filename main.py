# SIMULATION PARAMETERS
from math import sin, cos

import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_SPACE, K_UP, K_DOWN, K_a, K_w, K_z, K_s, K_x, K_c, K_d, K_r

from constants import ARMS_LENGTHS, Kp, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, CONTROL_DT, goal_reached_length, velocity_constraint, Ki, Kd
from dynamic_model import PIDController, RobotArm3dof
import shelve

from sim_utils import length
from user_input import gripperControl, keyboard_control
from visualize_robot_arm import Display, display_to_coordinate

dt = CONTROL_DT
# ROBOT     PARAMETERS
x0 = 0.0  # base x position
y0 = 0.0  # base y position

global_db = shelve.open("cache")

def cap_goal(goal):
    local_goal = goal - robot_base
    l = length(local_goal)

    if l > TOTAL_ARM_LENGTH:
        shorter_local_goal = local_goal / l * TOTAL_ARM_LENGTH
        return shorter_local_goal + robot_base
    return goal


class Planner:

    def __init__(self, waypoints=None):
        if waypoints is None:
            waypoints = []
        self.waypoints = waypoints
        self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints))
        self.goal_i = 0
        self.finished = False

    def step(self, current_pos):
        if len(self.inner_waypoints) <= 1:
            return current_pos
        goal = self.inner_waypoints[self.goal_i]

        dist_to_goal = length(current_pos, goal)
        if dist_to_goal < goal_reached_length:
            if self.goal_i < len(self.inner_waypoints)-1:
                self.goal_i += 1
            else:
                self.finished = True
                print("Goal reached")
        return goal

    def add_waypoint(self, goal):
        self.waypoints.append(goal)
        self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints))

    def reset_waypoints(self):
        self.waypoints = []
        self.inner_waypoints = []

    def create_inner_waypoints(self, waypoints):
        if len(waypoints) == 0:
            return []
        inner_waypoints = [waypoints[0]]
        d_length = 0.05

        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            length_waypoint = length(p1, p2)
            angle = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            n = int(length_waypoint / d_length) + 1
            for j in range(n):
                inner_waypoints.append(p1 + np.array([cos(angle), sin(angle)]) * j * d_length)

        return inner_waypoints


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
    planner = Planner([end_pos]+waypoints)

    # goal = robot_base + local_endp_start
    gripper = [100, 100]

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)
    step = 0
    left_click = False
    mouse_released = False
    enable_robot = True
    should_run = True
    while not planner.finished and should_run:
        goal = planner.step(end_pos)
        display.render(q, goal, planner.waypoints, planner.inner_waypoints)  # RENDER
        mouse_x_display, mouse_y_display = pygame.mouse.get_pos()
        mouse_x, mouse_y = display_to_coordinate(mouse_x_display, mouse_y_display)

        new_left_click = pygame.mouse.get_pressed()[0]
        if not new_left_click and left_click:
            mouse_released = True

        if mouse_released:
            new_waypoint = np.array([mouse_x, mouse_y])
            planner.add_waypoint(new_waypoint)
        # USER CONTROL STUFF
        gripper = gripperControl(gripper)

        new_keyboard_goal, dq_keyboard = keyboard_control(dt, goal)

        keys = pygame.key.get_pressed()
        if keys[K_SPACE]:
            enable_robot = not enable_robot
            print("enable_robot", enable_robot)
        if keys[K_w]:
            if 'waypoints' not in global_db or not (global_db['waypoints'] == planner.waypoints).all():
                global_db['waypoints'] = planner.waypoints
            # if 'waypoints' not in global_db:
                print('save waypoints', waypoints)
            #     global_db['waypoints'] = planner.waypoints
        if keys[K_r]:
            if 'waypoints' in global_db:
                print('reset waypoints', waypoints)
                del global_db['waypoints']
            planner.reset_waypoints()

        # Control
        local_goal = goal - robot_base

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
        left_click = new_left_click
        mouse_released = False
    # if len(state) > 0:
    #     state = np.array(state)
    #     global_db['state'] = state