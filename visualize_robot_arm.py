import numpy as np
import matplotlib.pyplot as plt
import pygame
from numpy import sin, cos

from dynamic_model import RobotArm3dof
from visualization_util import DISPLAY, WINDOW_SCALE

def coordinate_to_display(x, y): # WORKING
    xc, yc = DISPLAY.get_rect().center
    return int(x*WINDOW_SCALE) + xc, int(-y*WINDOW_SCALE) + yc

def display_to_coordinate(x_display, y_display):
    xc, yc = DISPLAY.get_rect().center
    return (x_display - xc)/WINDOW_SCALE, -(y_display - yc)/WINDOW_SCALE

class Display:

    def __init__(self, dt, arm_lengths, start_pos):
        # initialise real-time plot with pygame
        self.arm_lengths = arm_lengths

        self.xc, self.yc = DISPLAY.get_rect().center  # window center
        pygame.display.set_caption('robot arm')

        self.font = pygame.font.Font('freesansbold.ttf', 12)  # printing text font and font size
        self.text = self.font.render('robot arm', True, (0, 0, 0), (255, 255, 255))  # printing text object
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (10, 10)  # printing text position with respect to the top-left corner of the window

        self.clock = pygame.time.Clock()  # initialise clock

        # SIMULATION PARAMETERS
        dts = dt * 1  # desired simulation step time (NOTE: it may not be achieved)
        self.T = 3  # total simulation time

        self.FPS = int(1 / dts)  # refresh rate

        # scaling
        self.window_scale = WINDOW_SCALE  # conversion from meters to pixels

        self.start = start_pos

    def render(self, q, goal, waypoints, inner_waypoints, inner_q, robot:RobotArm3dof):
        # real-time plotting
        DISPLAY.fill((255, 255, 255))  # clear window
        if len(self.arm_lengths) == 3:
            l1, l2, l3 = self.arm_lengths
        else:
            (l1, l2), l3 = self.arm_lengths, None

        # update individual link position
        x0, y0 = self.start
        xbase, ybase = [0, 0]
        # print(q[0])
        # print(l1)
        x1 = x0 + l1 * np.cos(q[0])
        y1 = y0 + l1 * np.sin(q[0])
        # print("xy",x1,y1)
        x2 = x1 + l2 * np.cos(q[0] + q[1])
        y2 = y1 + l2 * np.sin(q[0] + q[1])
        if l3 is not None:
            x3 = x2 + l3 * np.cos(q[0] + q[1] + q[2])
            y3 = y2 + l3 * np.sin(q[0] + q[1] + q[2])
        # x4 = x3 + l4 * np.cos(q[0] + q[1] + q[2] + q[3])
        # y4 = y3 + l4 * np.sin(q[0] + q[1] + q[2] + q[3])
        window_scale = self.window_scale
        xc, yc = self.xc, self.yc

        if l3 is not None:
            xy_list = list(zip([xbase, x0, x1, x2, x3], [ybase, y0, y1, y2, y3]))
        else:
            xy_list = list(zip([xbase, x0, x1, x2], [ybase, y0, y1, y2]))
        xy_list_lines = xy_list.copy()
        points = np.array(xy_list_lines)
        #
        points *= window_scale
        points[:, 1] *= -1
        points[:] += np.array([xc, yc])
        pygame.draw.lines(DISPLAY, (0, 0, 255), False, points, 3)
        xc, yc = DISPLAY.get_rect().center

        def draw_points(xy, color=(0, 0, 0)):
            for x, y in xy:
                pygame.draw.circle(DISPLAY, color,
                                   (int(window_scale * x) + xc, int(-window_scale * y) + yc),
                                   5)

        draw_points(xy_list[0:-1])
        draw_points(xy_list[-1:], color=(255, 0, 0))

        for i in range(0,len(inner_q),20): # draw planned arm positions
            q = inner_q[i]
            # q_pos = np.zeros((len(q),2))
            prev_pos = np.zeros(2)
            q_pos = robot.FK_all_points(q)

            for j in range(len(q)):
                _q_pos = q_pos[j]
                pygame.draw.line(DISPLAY, (220, 220, 220), coordinate_to_display(*prev_pos), coordinate_to_display(*_q_pos), 2)
                pygame.draw.circle(DISPLAY, (200, 200, 200), coordinate_to_display(*_q_pos), 2)

                prev_pos = _q_pos

        for i in range(len(waypoints)):
            w_pos = waypoints[i][:2]
            w_angle = waypoints[i][2]
            if i < len(waypoints) - 1:
                if i == 0:
                    pygame.draw.circle(DISPLAY, (255, 0, 0), coordinate_to_display(*w_pos), 8) # DRAW START POINT
                else:
                    pygame.draw.circle(DISPLAY, (150, 150, 150), coordinate_to_display(*w_pos), 8) # DRAW OTHER WAYPOINTS
                # draw arrow for direction with w_angle
            else:
                pygame.draw.circle(DISPLAY, (0, 255, 0), coordinate_to_display(*w_pos), 4) # DRAW FINAL POINT
            arrow_length = 0.1
            pygame.draw.line(DISPLAY, (0, 0, 0), coordinate_to_display(*w_pos), coordinate_to_display(*(w_pos+arrow_length*np.array([cos(w_angle), sin(w_angle)]))), 2)

        for i in range(len(inner_waypoints)):
            w_pos = inner_waypoints[i, :2]
            w_angle = inner_waypoints[i,2]
            angle_arrow_length = 0.04
            if i < len(inner_waypoints)-1:
                # draw inner waypoint angle
                pygame.draw.line(DISPLAY, (0, 0, 0), coordinate_to_display(*w_pos),
                                 coordinate_to_display(*(w_pos + angle_arrow_length * np.array([cos(w_angle), sin(w_angle)]))), 2)

                w2_pos = inner_waypoints[i + 1, :2]
                pygame.draw.line(DISPLAY, (150, 150, 150), coordinate_to_display(*w_pos), coordinate_to_display(*w2_pos), 2)
                pygame.draw.circle(DISPLAY, (180, 180, 255), coordinate_to_display(*w_pos), 1)
            else:
                pygame.draw.circle(DISPLAY, (0, 255, 0), coordinate_to_display(*w_pos), 1)


        pygame.draw.circle(DISPLAY, (0, 0, 255), coordinate_to_display(*goal[:2]), 4)  # DRAW CURRENT GOAL

        text = self.font.render("FPS = " + str(round(self.clock.get_fps())), True, (0, 0, 0), (255, 255, 255))
        DISPLAY.blit(text, self.textRect)

    def tick(self):
        self.clock.tick(self.FPS)


def plot_state(state):
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