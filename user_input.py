# SIMULATION PARAMETERS

import numpy as np
import pygame
from pygame import K_DOWN, K_LEFT, K_RIGHT, K_UP, K_a, K_c, K_d, K_s, K_x, K_z

from constants import ARMS_LENGTHS


def keyboard_control(dt, goal):
    step_size = dt * 0.1
    joint_step_size = step_size * 300
    goal = goal.copy()
    joints = len(ARMS_LENGTHS)
    dq_keyboard = np.zeros(joints)
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

    for i, key_set in enumerate([[K_a, K_z], [K_s, K_x], [K_d, K_c]]):
        up, down = key_set
        if keys[up]:
            dq_keyboard[i] += joint_step_size
        if keys[down]:
            dq_keyboard[i] -= joint_step_size
    dq_keyboard = dq_keyboard if any(dq_keyboard) != 0 else None
    return goal, dq_keyboard


def gripperControl(goal):
    pygame.event.get()  # refresh keys
    keys = pygame.key.get_pressed()
    if keys[pygame.K_q]:
        goal[0] -= 1
    if keys[pygame.K_e]:
        goal[0] += 1
    if keys[pygame.K_a]:
        goal[1] -= 1
    if keys[pygame.K_d]:
        goal[1] += 1
    return goal