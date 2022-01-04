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


if __name__ == '__main__':
    l = ARMS_LENGTHS

    start = np.array([0, 0.2])
    model = robot_arm_3dof(l)
    controller = Controller(kp=15,ki=0.1,kd=0.1)

    t = 0.0  # time
    # pr = np.array((x / 10 + 0.0, y / 10 + 0.1))  # reference endpoint trajectory
    q0 = model.IK2([0, 0.2])  # initial configuration
    q = np.array([np.pi, -np.pi, q0[0]])  # initial configuration
    dq = np.array([0., 0., 0.])  # joint velocity

    state = []  # state vector
    goal = start

    display = Display(dt, ARMS_LENGTHS, start_pos=start)

    while True:
        model.state(q, dq)

        goal = keyboard_control(dt, goal)
        # Render
        display.render(q, goal)
        # Control
        p, dq = controller.control(model, goal - start, dt)

        q += dq * dt
        t += dt

        # save state
        state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], p[0], p[1]])

        # try to keep it real time with the desired step time
        display.tick()
