# SIMULATION PARAMETERS
from math import pi, sin, cos

import matplotlib.pyplot as plt
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_SPACE, K_UP, K_DOWN, K_a, K_w, K_z, K_s, K_x, K_c, K_d, K_r

from constants import ARMS_LENGTHS, Kp, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, CONTROL_DT, inner_waypoint_step_size, goal_reached_angle, \
    goal_reached_length, \
    velocity_constraint, Ki, Kd
from dynamic_model import PIDController, RobotArm3dof, angle_to_pos, get_angle
import shelve
from plot import plot_dq_constraints
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

    def step(self, current_pos, current_end_angle):
        if len(self.inner_waypoints) <= 1:
            return [*current_pos, current_end_angle]
        goal = self.inner_waypoints[self.goal_i]

        dist_to_goal = length(current_pos, goal[:2])
        angle_error = abs(current_end_angle - goal[2])
        if angle_error > pi:
            angle_error -= pi
        if dist_to_goal < goal_reached_length and angle_error < goal_reached_angle:
            if self.goal_i < len(self.inner_waypoints) - 1:
                self.goal_i += 1
            else:
                self.finished = True
                print("Goal reached")
        return goal

    def add_waypoint(self, goal):
        self.waypoints.append(goal)
        # self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints))
        self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints))

    def reset_waypoints(self):
        self.goal_i = 0
        self.waypoints = []
        self.inner_waypoints = []

    def create_inner_waypoints(self, waypoints):
        if len(waypoints) == 0:
            return []
        assert len(waypoints[0]) == 3, 'needs x,y,angle'
        inner_waypoints = [waypoints[0]]

        for i in range(len(waypoints) - 1):
            p1 = waypoints[i, :2]
            angle1 = waypoints[i, 2]
            p2 = waypoints[i + 1, :2]
            angle2 = waypoints[i + 1, 2]
            diff_angle = angle2 - angle1
            if diff_angle > pi:
                diff_angle -= 2 * pi
            if diff_angle < -pi:
                diff_angle += 2 * pi

            length_waypoint = length(p1, p2)
            dir_to_next_point = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            n = int(length_waypoint / inner_waypoint_step_size) + 1
            for j in range(n):
                _pos = p1 + np.array([cos(dir_to_next_point), sin(dir_to_next_point)]) * j * inner_waypoint_step_size
                inner_waypoints.append([*_pos, angle1 + diff_angle / n * j])

        return np.array(inner_waypoints)


if __name__ == '__main__':
    waypoints = []
    if 'waypoints' in global_db:
        waypoints = global_db['waypoints']
    robot_base = np.array([0, ZERO_POS_BASE])

    robot_arm = RobotArm3dof(ARMS_LENGTHS, velocity_constraint, reset_q=INITIAL_CONFIG_Q)
    local_endp_start = robot_arm.end_p
    q = robot_arm.q
    controller = PIDController(kp=Kp, ki=Ki, kd=0.1)
    controller_angle = PIDController(kp=Kp, ki=Ki, kd=0.1)
    t = 0.0  # time

    state = []  # state vector
    end_pos = robot_base + local_endp_start
    end_angle = sum(q)

    planner = Planner([np.array([*end_pos, end_angle])] + waypoints)

    gripper = [100, 100]

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)
    step = 0
    was_click = False
    mouse_released = False
    enable_robot = True
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
            new_waypoint = np.array(new_click_pos + [0.])
            planner.add_waypoint(new_waypoint)
        if on_left_click:
            second_click_pos = [mouse_x, mouse_y]
            angle = np.arctan2(second_click_pos[1] - new_click_pos[1], second_click_pos[0] - new_click_pos[0])
            new_waypoint = np.array(second_click_pos + [angle])
            planner.waypoints[-1][2] = angle
            if step % 2 == 0:
                planner.inner_waypoints = planner.create_inner_waypoints(np.array(planner.waypoints))
        # USER CONTROL STUFF
        gripper = gripperControl(gripper)

        new_keyboard_goal, dq_keyboard = keyboard_control(dt, goal_pos)

        keys = pygame.key.get_pressed()
        if keys[K_SPACE]:
            enable_robot = not enable_robot
            print("enable_robot", enable_robot)
        if keys[K_w]:
            global_db['waypoints'] = planner.waypoints
            print('save waypoints', waypoints)
        if keys[K_r]:
            if 'waypoints' in global_db:
                print('reset waypoints', waypoints)
                del global_db['waypoints']
            planner.reset_waypoints()

        # Control
        local_goal_pos = goal_pos - robot_base
        local_goal = np.array([*local_goal_pos, goal[2]])
        use_pid = False
        if enable_robot:
            # gets the end effector goal
            goal_p2 = local_goal_pos - angle_to_pos(goal[2], ARMS_LENGTHS[-1])

            ik_qs = robot_arm.IK2(goal_p2)
            if use_pid:
                p_2, p_end = robot_arm.FK_all_points()[-2:]

                F_end = controller.control_step(p_end, local_goal_pos, dt)

                F_2 = controller_angle.control_step(p_2, goal_p2, dt)
                end_pos, q, dq = robot_arm.request_force_xz(F_2, F_end)  # this requests a endpoint force and returns pos, angle,angle_speed
            else:
                q2s = robot_arm.IK2(goal_p2)
                new_q3 = goal[2] - sum(q2s)
                new_q = np.array([q2s[0], q2s[1], new_q3])
                dq = new_q - q
                dq *= 100
                end_pos, q, dq = robot_arm.move_joints(dq)
            end_angle = sum(q)
            # else:
            #     # not used for goals
            #     end_pos, q, dq = robot_arm.move_joints(dq_keyboard)
            #     # Set goal exactly to current endpoint
            #     goal = end_pos + robot_base
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
    if len(state) > 0 and should_save:
        state = np.array(state)
        global_db['state'] = state
    elif 'state' in global_db:
        state = global_db['state']

    # plot_dq_constraints(state)
    # plt.show()