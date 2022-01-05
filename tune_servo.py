import time

import numpy as np
import pygame
from pygame import K_LEFT, K_UP, K_RIGHT, K_DOWN

from arduino_communication import sent_action

step_size = 0.01
q = np.array([0., 0.,0.])

pygame.init()  # start pygame

def keyboard():
    pygame.event.get()  # refresh keys
    keys = pygame.key.get_pressed()
    if keys[K_LEFT]:
        q[0] -= step_size
    if keys[K_RIGHT]:
        q[0] += step_size

    if keys[K_UP]:
        q[1] += step_size
    if keys[K_DOWN]:
        q[1] -= step_size

DISPLAY = pygame.display.set_mode((800, 600))  # create a window (size in pixels)
step = 0
while True:
    step += 1
    keyboard()
    if step % 10 == 0:
        sent_action(q*10, debug=True)
    time.sleep(0.01)
