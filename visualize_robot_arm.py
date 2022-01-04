"""
Robot Dynamics and Control Assignment 2e: kinematic task-priority control
-------------------------------------------------------------------------------
DESCRIPTION:
4-DOF planar robot arm model with shoulder and elbow joints. The code includes
simulation environment and visualisation of the robot.

The robot is a classic position-controlled robot:
- Measured joint angle vector q is provided each sample time.
- Calculate the joint velocity dq in each sample time to be sent to the robot.

Important variables:
q[0] -> shoulder joint configuration
q[1] -> first elbow joint configuration
q[2] -> second elbow joint configuration
q[3] -> third elbow joint configuration
p[0] -> endpoint x position
p[1] -> endpoint y position

TASK:
Make the robot track a given endpoint reference trajectory with the primary
endpoint (end of the kinematic chain) by using a kinematic control (i.e., PID
controller). This robot structure has two redundant degrees of freedom. Use
null-space control to make the second elbow (secondary endpoint) track position
[0,0] as a secondary task to the primary endpoint task.
-------------------------------------------------------------------------------


INSTURCTOR: Luka Peternel
e-mail: l.peternel@tudelft.nl

"""

import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
from numpy import sin, cos

from constants import ARMS_LENGTHS
from dynamic_model import Controller, robot_arm_2dof

'''SIMULATION'''

# SIMULATION PARAMETERS
dt = 0.01  # integration step time
dts = dt * 1  # desired simulation step time (NOTE: it may not be achieved)
T = 3  # total simulation time

# ROBOT PARAMETERS
x0 = 0.0  # base x position
y0 = 0.0  # base y position
l1,l2,l3 = ARMS_LENGTHS
# l4 = 0.3  # link 4 length
l = [l1, l2, l3]  # link length

# REFERENCE TRAJETORY
ts = T / dt  # trajectory size
xt = np.linspace(-2, 2, int(ts))
yt1 = np.sqrt(1 - (abs(xt) - 1) ** 2)
yt2 = -3 * np.sqrt(1 - (abs(xt) / 2) ** 0.5)

x = np.concatenate((xt, np.flip(xt, 0)), axis=0)
y = np.concatenate((yt1, np.flip(yt2, 0)), axis=0)

pr = np.array((x / 10 + 0.0, y / 10 + 0.1))  # reference endpoint trajectory

'''*********** Student should fill in ***********'''
# PID CONTROLLER PARAMETERS
Kp = 15  # proportional gain
Ki = 0.3  # integral gain
Kd = 0.1  # derivative gain
'''*********** Student should fill in ***********'''
se = 0.0  # integrated error
pe = 0.0  # previous error

se2 = 0.0  # integrated error
pe2 = 0.0  # previous error
# SIMULATOR
# initialise robot model class
model = robot_arm_2dof(l)
controller = Controller(Kp, Ki,Kd)

# initialise real-time plot with pygame
pygame.init()  # start pygame
window = pygame.display.set_mode((800, 600))  # create a window (size in pixels)
window.fill((255, 255, 255))  # white background
xc, yc = window.get_rect().center  # window center
pygame.display.set_caption('robot arm')

font = pygame.font.Font('freesansbold.ttf', 12)  # printing text font and font size
text = font.render('robot arm', True, (0, 0, 0), (255, 255, 255))  # printing text object
textRect = text.get_rect()
textRect.topleft = (10, 10)  # printing text position with respect to the top-left corner of the window

clock = pygame.time.Clock()  # initialise clock
FPS = int(1 / dts)  # refresh rate

# initial conditions
t = 0.0  # time
q0 = model.IK2(pr[:, 0])  # initial configuration
q = np.array([np.pi, -np.pi, q0[0]])  # initial configuration
dq = np.array([0., 0., 0.])  # joint velocity
model.state(q, dq)  # update initial state
i = 0  # loop counter
state = []  # state vector
last_p_end = np.array([0, 0])
last_p_2 = np.array([0, 0])

# scaling
window_scale = 400  # conversion from meters to pixles

# wait until the start button is pressed
run = True
while run:
    for event in pygame.event.get():  # interrupt function
        run = False
        if event.type == pygame.KEYUP:
            if event.key == ord('e'):  # enter the main loop after 'e' is pressed
                run = False

# MAIN LOOP
run = True
for i in range(len(x)):
    for event in pygame.event.get():  # interrupt function
        if event.type == pygame.QUIT:  # force quit with closing the window
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('q'):  # force quit with q button
                run = False

    # update individual link position
    x1 = l1 * np.cos(q[0])
    y1 = l1 * np.sin(q[0])
    x2 = x1 + l2 * np.cos(q[0] + q[1])
    y2 = y1 + l2 * np.sin(q[0] + q[1])
    x3 = x2 + l3 * np.cos(q[0] + q[1] + q[2])
    y3 = y2 + l3 * np.sin(q[0] + q[1] + q[2])
    # x4 = x3 + l4 * np.cos(q[0] + q[1] + q[2] + q[3])
    # y4 = y3 + l4 * np.sin(q[0] + q[1] + q[2] + q[3])

    # real-time plotting
    window.fill((255, 255, 255))  # clear window
    pygame.draw.circle(window, (0, 255, 0), (int(window_scale * pr[0, i]) + xc, int(-window_scale * pr[1, i]) + yc),
                       3)  # draw reference position
    pygame.draw.lines(window, (0, 0, 255), False, [(window_scale * x0 + xc, -window_scale * y0 + yc),
                                                   (window_scale * x1 + xc, -window_scale * y1 + yc),
                                                   (window_scale * x2 + xc, -window_scale * y2 + yc),
                                                   (window_scale * x3 + xc, -window_scale * y3 + yc)], 3)  # draw links
    pygame.draw.circle(window, (0, 0, 0), (int(window_scale * x0) + xc, int(-window_scale * y0) + yc),
                       7)  # draw shoulder / base
    pygame.draw.circle(window, (0, 0, 0), (int(window_scale * x1) + xc, int(-window_scale * y1) + yc),
                       7)  # draw first elbow
    pygame.draw.circle(window, (0, 0, 0), (int(window_scale * x2) + xc, int(-window_scale * y2) + yc),
                       7)  # draw second elbow
    pygame.draw.circle(window, (0, 0, 0), (int(window_scale * x3) + xc, int(-window_scale * y3) + yc),
                       7)  # draw third elbow
    # pygame.draw.circle(window, (255, 0, 0), (int(window_scale * x4) + xc, int(-window_scale * y4) + yc),
    #                    3)  # draw hand / endpoint

    text = font.render("FPS = " + str(round(clock.get_fps())), True, (0, 0, 0), (255, 255, 255))
    window.blit(text, textRect)

    pygame.display.flip()  # update display

    '''*********** Student should fill in ***********'''
    goal = pr[:, i]
    model.state(q, dq)
    p, dq = controller.control(model, goal, dt)
    # log states for analysis
    state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], p[0], p[1]])

    # get joint angles by intergration
    q += dq * dt
    t += dt

    # increase loop counter
    i = i + 1

    # try to keep it real time with the desired step time
    clock.tick(FPS)

    if i >= len(x):
        run = False
    if run == False:
        break

pygame.quit()  # stop pygame

'''ANALYSIS'''

state = np.array(state)

plt.figure(1)
plt.subplot(211)
plt.title("JOINT SPACE BEHAVIOUR")
plt.plot(state[:, 0], state[:, 4], "b", label="shoulder")
plt.plot(state[:, 0], state[:, 5], "r", label="elbow1")
plt.plot(state[:, 0], state[:, 6], "g", label="elbow2")
# plt.plot(state[:, 0], state[:, 7], "m", label="elbow3")
plt.legend()
plt.ylabel("dq [rad/s]")

plt.subplot(212)
plt.plot(state[:, 0], state[:, 1], "b", label="shoulder")
plt.plot(state[:, 0], state[:, 2], "r", label="elbow1")
plt.plot(state[:, 0], state[:, 3], "g", label="elbow2")
# plt.plot(state[:, 0], state[:, 4], "m", label="elbow3")
plt.legend()
plt.ylabel("q [rad]")
plt.xlabel("t [s]")

plt.tight_layout()

plt.figure(2)
plt.title("ENDPOINT SPACE BEHAVIOUR")
plt.plot(0, 0, "ok", label="shoulder")
plt.plot(state[:, 9], state[:, 10], label="trajectory")
plt.plot(state[0, 9], state[0, 10], "xg", label="start point")
plt.plot(state[-4, 7], state[-4, 10], "+r", label="end point")
plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()

plt.tight_layout()
