# SIMULATION PARAMETERS
import shelve
from math import cos, pi, sin

import matplotlib.pyplot as plt
import numpy as np
import pygame
from pygame import K_SPACE, K_r, K_w

from constants import ARMS_LENGTHS, CONTROL_DT, INITIAL_CONFIG_Q, Kd, Ki, Kp, TOTAL_ARM_LENGTH, goal_reached_angle, goal_reached_length, \
    inner_waypoint_step_size, velocity_constraint
from dynamic_model import PIDController, RobotArm3dof, angle_diff, angle_to_pos
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

    def __init__(self, robot:RobotArm3dof, waypoints=None):
        if waypoints is None:
            waypoints = []

        self.waypoints = waypoints
        self.goal_i = 0
        self.finished = False
        self.robot = robot
        self.initial_q = robot.q
        self.inner_waypoints, self.inner_q = self.create_inner_waypoints(np.array(self.waypoints))


    def step(self, current_pos, current_end_angle):
        if len(self.inner_waypoints) <= 1:
            return [*current_pos, current_end_angle]
        goal = self.inner_waypoints[self.goal_i]

        dist_to_goal = length(current_pos, goal[:2])
        angle_error = abs(angle_diff(goal[2], current_end_angle))
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
        self.inner_waypoints, self.inner_q = self.create_inner_waypoints(np.array(self.waypoints))

    def reset_waypoints(self):
        self.goal_i = 0
        self.waypoints = []
        self.inner_waypoints = []

    def refresh_waypoints(self):
        self.inner_waypoints, self.inner_q = self.create_inner_waypoints(np.array(self.waypoints))

    def create_inner_waypoints(self, waypoints):
        if len(waypoints) == 0:
            return []
        assert len(waypoints[0]) == 3, 'needs x,y,angle'
        inner_waypoints = []
        # inner_waypoints_vel = [np.zeros(3)]
        inner_q = []
        current_q0 = self.initial_q[0]
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
                _end_pos = p1 + np.array([cos(dir_to_next_point), sin(dir_to_next_point)]) * j * inner_waypoint_step_size
                goal_angle = angle1 + diff_angle / n * j
                new_waypoint_pos_angle = [*_end_pos, goal_angle]
                inner_waypoints.append(new_waypoint_pos_angle)

                local_end_pos = _end_pos - robot_base
                pos2 = local_end_pos - angle_to_pos(goal_angle, ARMS_LENGTHS[-1])
                q0,q1 = self.robot.IK2(pos2, current_q0)
                q3 = angle_diff(goal_angle, q0+q1)

                inner_q.append([q0,q1,q3])
        return np.array(inner_waypoints), np.array(inner_q)


if __name__ == '__main__':
    print('INSTRUCTIONS')
    print('Press SPACE to enable/disable robot')
    print('Press W to save waypoints')
    print('Press R to reset waypoints')
    print('Click and drag to add waypoints with the required end effector direction')
    should_run = True # False: go to plots directly
    should_save = False # True: save state to cache
    use_pid = False # PID works with null-space control, however not very good.

    waypoints = []
    if 'waypoints' in global_db:
        waypoints = global_db['waypoints']
    robot_base = np.array([0., 0.])

    robot_arm = RobotArm3dof(ARMS_LENGTHS, velocity_constraint, reset_q=INITIAL_CONFIG_Q)
    local_endp_start = robot_arm.end_p
    q = robot_arm.q
    controller = PIDController(kp=Kp, ki=Ki, kd=Kd)
    controller_angle = PIDController(kp=Kp*10, ki=Ki, kd=Kd)
    t = 0.0  # time

    state = []  # state vector
    end_pos = robot_base + local_endp_start
    end_angle = sum(q)

    planner = Planner(robot_arm, [np.array([*end_pos, end_angle])] + waypoints)

    gripper = [100, 100]

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)
    step = 0
    was_click = False
    mouse_released = False
    enable_robot = True

    new_click_pos = None

    while not planner.finished and should_run:
        goal = planner.step(end_pos, end_angle)
        goal_pos = goal[:2]

        display.render(q, goal, planner.waypoints, planner.inner_waypoints, planner.inner_q, robot_arm)  # RENDER
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
                planner.refresh_waypoints()
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
        if enable_robot:
            # gets the end effector goal
            end_effector_goal_angle = goal[2]
            goal_p2 = local_goal_pos - angle_to_pos(end_effector_goal_angle, ARMS_LENGTHS[-1])

            if use_pid:
                p_2, p_end = robot_arm.FK_all_points()[-2:]

                F_end = controller.control_step(p_end, local_goal_pos, dt)

                F_2 = controller_angle.control_step(p_2, goal_p2, dt)
                end_pos, q, dq = robot_arm.request_force_xz(F_2, F_end)  # this requests a endpoint force and returns pos, angle,angle_speed
            else:
                q2s = robot_arm.IK2(goal_p2, q[0])
                new_q3 = angle_diff(goal[2], sum(q2s))
                new_q = np.array([q2s[0], q2s[1], new_q3])
                dq = angle_diff(new_q, q)*1000*dt
                end_pos, q, dq = robot_arm.move_joints(dq)
            end_angle = sum(q)
            state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], end_pos[0], end_pos[1]])
        t += dt

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

    plot_dq_constraints(state)
    plt.show()