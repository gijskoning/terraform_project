# SIMULATION PARAMETERS
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_UP, K_DOWN

from gym_robotic_arm.constants import ARMS_LENGTHS, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, ARM_WIDTH, \
    CONTROL_DT

from gym_robotic_arm.dynamic_model import RobotArm3dof, PIDController
from serial import SerialException

from sim_utils import length, config_to_polygon_pygame, check_collision, config_to_polygon, arm_to_polygon
from visualization_util import draw_rectangle_from_config, DISPLAY
from visualize_robot_arm import Display
from gym_robotic_arm.arduino_communication import ArduinoControl
from gym_robotic_arm.envs.waveshare_camera import WaveShareCamera

dt = CONTROL_DT
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


def cap_goal(goal):
    local_goal = goal - robot_base
    l = length(local_goal)

    if l > TOTAL_ARM_LENGTH:
        shorter_local_goal = local_goal / l * TOTAL_ARM_LENGTH
        return shorter_local_goal + robot_base
    return goal

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


if __name__ == '__main__':
    arduino = True
    arduino_control = None
    arduino_port = 'COM4'  # Ubuntu desktop bottom right
    do_not_send = False
    if arduino:
        try:
            arduino_control = ArduinoControl(port=arduino_port, do_not_send=do_not_send)
        except IOError as e:
            print(e)
    print("arduino_control", arduino_control)
    robot_base = np.array([0, ZERO_POS_BASE])

    robot_arm = RobotArm3dof(l=ARMS_LENGTHS, reset_q=INITIAL_CONFIG_Q, arduino_control=arduino_control)
    local_endp_start = robot_arm.end_p
    q = robot_arm.q
    controller = PIDController(kp=15, ki=0.1, kd=0.1)

    t = 0.0  # time

    state = []  # state vector
    p = robot_base + local_endp_start
    goal = robot_base + local_endp_start

    gripper = [100,100]

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)
    step = 0
    sent = 2
    camera = WaveShareCamera(1)
    camera.show_feed_continuous()

    while True:
        display.render(q, goal)

        gripper = gripperControl(gripper)

        goal = keyboard_control(dt, goal)
        goal = cap_goal(goal)

        # Control
        local_goal = goal - robot_base

        # F_end can be replaced with RL action. array[2]
        F_end = controller.control_step(robot_arm.FK_end_p(), local_goal, dt)
        p, q, dq = robot_arm.move_endpoint_xz(F_end, gripper, step)
        t += dt

        # Render
        for pol in robot_arm.arm_regions:
            pol = [xy + robot_base for xy in pol]
            draw_rectangle_from_config(pol)
        # save state
        state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], p[0], p[1]])

        # try to keep it real time with the desired step time
        display.tick()
        pygame.display.flip()  # update display
        step += 1
