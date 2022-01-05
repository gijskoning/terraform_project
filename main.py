# SIMULATION PARAMETERS
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_UP, K_DOWN

from arduino_communication import sent_action
from constants import ARMS_LENGTHS, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, ARM_WIDTH, INITIAL_CONFIG_SERVO
from dynamic_model import robot_arm_3dof, Controller
from sim_utils import length, config_to_polygon_pygame, check_collision, config_to_polygon, arm_to_polygon
from visualization_util import draw_rectangle_from_config, DISPLAY
from visualize_robot_arm import Display

dt = 0.01  # integration step time

# ROBOT     PARAMETERS
x0 = 0.0  # base x position
y0 = 0.0  # base y position

# PID CONTROLLER PARAMETERS
Kp = 15  # proportional gain
Ki = 0.3  # integral gain
Kd = 0.1  # derivative gain


def keyboard_control(dt, goal):
    step_size = dt * 0.1
    goal = goal.copy()
    pygame.event.get()  # refresh keys
    keys = pygame.key.get_pressed()
    if keys[K_LEFT]:
        goal[0] -= step_size
    if keys[K_RIGHT]:
        goal[0] += step_size

    if keys[K_UP]:
        goal[1] += step_size
    if keys[K_DOWN]:
        goal[1] -= step_size
    return goal


def constraint(model: robot_arm_3dof, dq, dt):
    global_pos_constraint_lb = [0.01, -0.1]
    p = np.zeros(2)
    vis_polygons = []

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

    for i in range(len(dq)):
        new_q = model.q + dq * dt
        joint_pos_new = model.FK4_all(new_q)

        # Global constraint check
        if np.any(joint_pos_new < global_pos_constraint_lb):
            dq[i] = 0
        # Check collision with itself
        if i > 0:
            p = joint_pos_new[i - 1].copy()
        if not check_collision(create_obstacles(joint_pos_new, new_q)):
            dq[i] = 0

        l = ARMS_LENGTHS[i]
        pol = arm_to_polygon(*p, np.sum(model.q[:i + 1]), l, ARM_WIDTH)
        vis_polygons.append(pol)
        # if np.any(joint_pos_new < global_pos_constraint_lb):
        #     dq[i] = 0

    return dq, vis_polygons


def cap_goal(goal):
    local_goal = goal - robot_base
    l = length(local_goal)

    if l > TOTAL_ARM_LENGTH:
        shorter_local_goal = local_goal / l * TOTAL_ARM_LENGTH
        return shorter_local_goal + robot_base
    return goal


if __name__ == '__main__':
    l = ARMS_LENGTHS

    robot_base = np.array([0, ZERO_POS_BASE])
    local_start = np.array([0.3, 0])
    model = robot_arm_3dof(l)
    controller = Controller(kp=15, ki=0.1, kd=0.1)

    t = 0.0  # time
    q = INITIAL_CONFIG_Q.copy()  # initial configuration
    dq = np.array([0., 0., 0.])  # joint velocity

    state = []  # state vector
    p = robot_base + local_start
    goal = robot_base + local_start

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)
    step = 0
    sent = 2
    while True:
        display.render(q, goal)

        model.state(q, dq)

        goal = keyboard_control(dt, goal)
        goal = cap_goal(goal)

        # Control
        local_goal = goal - robot_base

        # F_end can be replaced with RL action. array[2]
        F_end = controller.pid_control(model, local_goal, dt)

        p, dq = controller.control(model, F_end)
        dq, vis_polygons = constraint(model, dq, dt)

        # Move angles
        q += dq * dt
        if step % 100 == 0:
            print("q-INITIAL_CONFIG_Q", q - INITIAL_CONFIG_SERVO)
            print("INITIAL_CONFIG_Q", INITIAL_CONFIG_SERVO)
            q_temp = ((q-INITIAL_CONFIG_SERVO) * 100).astype(int)
            # q_temp = (q * 100).astype(int)
            # sent_action(f"0:{q_temp[0]}")
            # sent_action(f"0:{q_temp[0]},1:{-q_temp[1]},2:{q_temp[2]}")
            sent_action(q)
            # sent_action(f"0:{q[0]},1:{q[1]},2:{q[2]},3:{sent}")
        t += dt

        # Render
        # pygame.draw.polygon(DISPLAY, (255,0,0), config, width=line_width)
        for pol in vis_polygons:
            pol = [xy + robot_base for xy in pol]
            draw_rectangle_from_config(pol)

        # save state
        state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], p[0], p[1]])

        # try to keep it real time with the desired step time
        display.tick()
        pygame.display.flip()  # update display
        step += 1
