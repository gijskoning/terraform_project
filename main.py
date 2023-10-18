# SIMULATION PARAMETERS
from math import pi, sin, cos

import matplotlib.pyplot as plt
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_SPACE, K_UP, K_DOWN, K_a, K_w, K_z, K_s, K_x, K_c, K_d, K_r

from constants import ARMS_LENGTHS, Kp, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, CONTROL_DT, inner_waypoint_step_size, goal_reached_angle, \
    goal_reached_length, \
    velocity_constraint, Ki, Kd
from dynamic_model import PIDController, RobotArm3dof, angle_diff, angle_to_pos, get_angle
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

    def __init__(self, robot: RobotArm3dof, waypoints=None):
        if waypoints is None:
            waypoints = []

        self.waypoints = waypoints
        self.goal_i = 0
        self.finished = False
        self.robot = robot
        self.initial_q = robot.q
        self.inner_waypoints, self.inner_q, self.current_vel_q, self.endpoint_accel = self.create_inner_waypoints(np.array(self.waypoints))

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
        return goal, self.inner_q[self.goal_i], self.current_vel_q[self.goal_i]

    def add_waypoint(self, goal):
        self.waypoints.append(goal)
        # self.inner_waypoints = self.create_inner_waypoints(np.array(self.waypoints))
        self.inner_waypoints, self.inner_q, self.current_vel_q, self.endpoint_accel  = self.create_inner_waypoints(np.array(self.waypoints))

    def reset_waypoints(self):
        self.goal_i = 0
        self.waypoints = []
        self.inner_waypoints = []

    def refresh_waypoints(self):
        self.inner_waypoints, self.inner_q, self.endpoint_vel, self.endpoint_accel = self.create_inner_waypoints(np.array(self.waypoints))

    def get_new_point(self, current_end_pos, current_goal_angle, current_q, goal_diff_angle, dir_to_next_point, current_step_size, plan_timestep):
        # new_end_goal = current_end_pos + np.array([cos(dir_to_next_point), sin(dir_to_next_point)]) * current_step_size
        new_end_vel = np.array([cos(dir_to_next_point), sin(dir_to_next_point)]) * current_step_size
        new_end_pos = current_end_pos + new_end_vel*plan_timestep#+ np.array([cos(dir_to_next_point), sin(dir_to_next_point)]) * current_step_size


        new_goal_angle = current_goal_angle + goal_diff_angle * current_step_size

        q0, q1 = self.robot.IK2(new_end_pos, current_q[0])
        q2 = angle_diff(new_goal_angle,
                        q0 + q1)  # get the final q by new_goal_angle - sum(q0,q1). The goal is in global orientation. While q1,3 are local joint angles.

        new_q = np.array([q0, q1, q2])
        vel_q = angle_diff(new_q, current_q)
        return new_end_pos, new_end_vel, new_goal_angle, new_q, vel_q

    def create_inner_waypoints(self, waypoints):
        if len(waypoints) == 0:
            return []
        assert len(waypoints[0]) == 3, 'needs x,y,angle'
        p1 = waypoints[0, :2]
        first_goal_angle = waypoints[0, 2]
        q0, q1 = self.robot.IK2(p1, self.initial_q[0])
        q2 = angle_diff(first_goal_angle, q0 + q1)

        p1 = waypoints[0, :2]
        angle1 = waypoints[0, 2]
        p2 = waypoints[0 + 1, :2]
        angle2 = waypoints[0 + 1, 2]
        diff_goal_angle = angle2 - angle1
        if diff_goal_angle > pi:
            diff_goal_angle -= 2 * pi
        if diff_goal_angle < -pi:
            diff_goal_angle += 2 * pi
        dir_to_next_point = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        accel_val = 0.3 # per timestep

        current_q = [np.array([q0, q1, q2])]
        current_step_size = [0.1]
        current_end_pos = [waypoints[0, :2]]
        current_end_vel = [0.]
        current_goal_angle = [waypoints[0, 2]]
        current_diff_goal_angle = [diff_goal_angle]
        current_dir_to_next_point = [dir_to_next_point]
        current_vel_q = [np.zeros(3)]
        current_accel = [accel_val]
        current_waypoint_index = [0]
        inner_waypoint_index = 0
        while current_waypoint_index[inner_waypoint_index] < len(waypoints) - 1:
            i = current_waypoint_index[inner_waypoint_index]
        # for i in range(len(waypoints) - 1):
            p1 = waypoints[i, :2]
            angle1 = waypoints[i, 2]
            p2 = waypoints[i + 1, :2]
            angle2 = waypoints[i + 1, 2]
            diff_goal_angle = angle2 - angle1
            if diff_goal_angle > pi:
                diff_goal_angle -= 2 * pi
            if diff_goal_angle < -pi:
                diff_goal_angle += 2 * pi

            dir_to_next_point = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            plan_timestep = 0.05
            new_step_size = current_step_size[inner_waypoint_index] + current_accel[inner_waypoint_index] * plan_timestep

            new_end_pos, new_end_vel, new_goal_angle, new_q, vel_q = self.get_new_point(current_end_pos[inner_waypoint_index], current_goal_angle[inner_waypoint_index], current_q[inner_waypoint_index], current_diff_goal_angle[inner_waypoint_index], current_dir_to_next_point[inner_waypoint_index], new_step_size, plan_timestep)
            if not self.robot.within_joint_vel_constraints(vel_q):
                if current_accel[inner_waypoint_index] == -accel_val:
                    inner_waypoint_index -= 1
                    current_accel[inner_waypoint_index] -= accel_val
                    continue
                else:
                    current_accel[inner_waypoint_index] -= accel_val
                    continue
                # while current_accel[inner_waypoint_index] <= accel_val: # go back until some acceleration is found
                #     inner_waypoint_index -= 1
                # current_accel[inner_waypoint_index] -= accel_val
                # continue
            inner_waypoint_index += 1

            if inner_waypoint_index == len(current_step_size):
                current_q.append(None)
                current_step_size.append(None)
                current_end_vel.append(None)
                current_end_pos.append(None)
                current_goal_angle.append(None)
                current_diff_goal_angle.append(None)
                current_dir_to_next_point.append(None)
                current_accel.append(accel_val)
                current_waypoint_index.append(None)
                current_vel_q.append(None)

            current_q[inner_waypoint_index] = new_q
            current_step_size[inner_waypoint_index] = new_step_size
            current_end_vel[inner_waypoint_index] = new_end_vel
            current_end_pos[inner_waypoint_index] = new_end_pos
            current_goal_angle[inner_waypoint_index] = new_goal_angle
            current_diff_goal_angle[inner_waypoint_index] = diff_goal_angle
            current_dir_to_next_point[inner_waypoint_index] = dir_to_next_point
            current_vel_q[inner_waypoint_index] = vel_q
            if length(current_end_pos[inner_waypoint_index], p2) < 0.1: # reached waypoint
                current_waypoint_index[inner_waypoint_index] = current_waypoint_index[inner_waypoint_index-1]+1
            else:
                current_waypoint_index[inner_waypoint_index] = current_waypoint_index[inner_waypoint_index-1]
            # new_waypoint_pos_angle = [*current_end_pos, current_goal_angle]
            # inner_waypoints.append(new_waypoint_pos_angle)
            # if i == 2:
            #     exit()
            # local_end_pos = current_end_pos - robot_base
            # pos2 = local_end_pos - angle_to_pos(goal_angle, ARMS_LENGTHS[-1])
            # q0, q1 = self.robot.IK2(pos2, current_q[0])
            # q2 = angle_diff(goal_angle, q0 + q1)

            # inner_q.append(new_q)# todo
        inner_waypoints = np.array([*np.array(current_end_pos).T, np.array(current_goal_angle)]).T # shape (n,3)
        return inner_waypoints, np.array(current_q), current_vel_q, current_accel


if __name__ == '__main__':
    waypoints = []
    if 'waypoints' in global_db:
        waypoints = global_db['waypoints']
    robot_base = np.array([0, ZERO_POS_BASE])

    robot_arm = RobotArm3dof(ARMS_LENGTHS, velocity_constraint, reset_q=INITIAL_CONFIG_Q)
    local_endp_start = robot_arm.end_p
    q = robot_arm.q
    controller = PIDController(kp=Kp, ki=Ki, kd=0.1)
    controller_velq = PIDController(kp=Kp, ki=Ki, kd=0.1, dims=3)
    controller_angle = PIDController(kp=Kp, ki=Ki, kd=0.1)
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
    should_run = True
    should_save = True
    new_click_pos = None
    k_space_pressed = False
    while not planner.finished and should_run:
        goal, q_goal, goal_vel_q = planner.step(end_pos, end_angle)
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
        if keys[K_SPACE] and not k_space_pressed:
            enable_robot = not enable_robot
            print("enable_robot", enable_robot)
            k_space_pressed = True
        if not keys[K_SPACE]:
            k_space_pressed = False
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
        use_pid = True
        if enable_robot:
            # gets the end effector goal
            goal_p2 = local_goal_pos - angle_to_pos(goal[2], ARMS_LENGTHS[-1])

            if use_pid:
                # p_2, p_end = robot_arm.FK_all_points()[-2:]

                F_qvel = controller_velq.control_step(robot_arm.dq, goal_vel_q, dt)
                print('F_qvel', F_qvel, 'goal_vel_q', goal_vel_q)
                # F_2 = controller_angle.control_step(p_2, goal_p2, dt)
                # end_pos, q, dq = robot_arm.request_force_xz(F_end)  # this requests a endpoint force and returns pos, angle,angle_speed
                end_pos, q, dq = robot_arm.move_joints(F_qvel)

            else:
                # q2s = robot_arm.IK2(goal_p2, q[0])
                # new_q3 = angle_diff(goal[2], sum(q2s))
                # new_q = np.array([q2s[0], q2s[1], new_q3])
                # todo we want to calculate the error in velocity, and error in position
                aq = angle_diff(q_goal, q)
                # F_end = controller.control_step(p_end, local_goal_pos, dt)
                # end_pos, q, dq = robot_arm.request_force_xz(F_end)  # this requests a endpoint force and returns pos, angle,angle_speed

                # print('dq', dq, new_q, q, goal[2])
                # dq *= 100
                end_pos, q, dq = robot_arm.move_joints(aq)
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