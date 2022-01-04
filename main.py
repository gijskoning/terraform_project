# SIMULATION PARAMETERS
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_UP, K_DOWN

from constants import ARMS_LENGTHS
from dynamic_model import robot_arm_3dof, Controller
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
    global_pos_constraint_lb = [0.01,-0.1]

    joint_pos = model.FK4_all()
    joint_pos_new = model.FK4_all(model.q + dq * dt)
    # todo
    # for i in range(len(dq)):
    #     joint_pos_new = model.FK4_all(model.q + dq * dt)
    #     if[joint_pos_new[i]]

    # Set dq to zero if constraint is met
    dq[np.any(joint_pos_new < global_pos_constraint_lb)] = 0
    # dq[np.any(joint_pos_new < global_pos_constraint_lb)] = 0
    # print(np.any(joint_pos_new < global_pos_constraint_lb, axis=1))
    return dq


if __name__ == '__main__':
    l = ARMS_LENGTHS

    initial_angles = np.array([0, np.pi * 0.8, -np.pi * 0.8])
    robot_base = np.array([0, 0.1])
    local_start = np.array([0.3, 0])
    model = robot_arm_3dof(l)
    controller = Controller(kp=15, ki=0.1, kd=0.1)

    t = 0.0  # time
    q = initial_angles  # initial configuration
    dq = np.array([0., 0., 0.])  # joint velocity

    state = []  # state vector
    p = robot_base + local_start
    goal = robot_base + local_start

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)

    while True:
        model.state(q, dq)

        goal = keyboard_control(dt, goal)
        # Render
        display.render(q, goal)
        # Control
        local_goal = goal - robot_base

        # F_end can be replaced with RL action. array[2]
        F_end = controller.pid_control(model, local_goal, dt)

        p, dq = controller.control(model, local_goal, F_end)
        dq = constraint(model, q, dq, dt)
        # Move angles
        q += dq * dt
        t += dt

        # save state
        state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], p[0], p[1]])

        # try to keep it real time with the desired step time
        display.tick()
