# SIMULATION PARAMETERS
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_UP, K_DOWN, K_a, K_z, K_s, K_x, K_c, K_d

from gym_robotic_arm.constants import ARMS_LENGTHS, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, ARM_WIDTH, \
    CONTROL_DT

from gym_robotic_arm.dynamic_model import RobotArm3dof, PIDController
# from serial import SerialException

from sim_utils import length, config_to_polygon_pygame, check_collision, config_to_polygon, arm_to_polygon
from visualization_util import draw_rectangle_from_config, DISPLAY
from visualize_robot_arm import Display
# from gym_robotic_arm.arduino_communication import ArduinoControl
# from gym_robotic_arm.envs.waveshare_camera import WaveShareCamera

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
    joint_step_size = step_size*300
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

    for i, key_set in enumerate([[K_a, K_z],[K_s, K_x],[K_d, K_c]]):
        up,down = key_set
        if keys[up]:
            dq_keyboard[i] += joint_step_size
        if keys[down]:
            dq_keyboard[i] -= joint_step_size
    dq_keyboard = dq_keyboard if any(dq_keyboard) != 0 else None
    return goal, dq_keyboard


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
    do_not_send = False
    robot_base = np.array([0, ZERO_POS_BASE])

    robot_arm = RobotArm3dof(l=ARMS_LENGTHS, reset_q=INITIAL_CONFIG_Q)
    local_endp_start = robot_arm.end_p
    q = robot_arm.q
    controller = PIDController(kp=15, ki=0.1, kd=0.1)

    t = 0.0  # time

    state = []  # state vector
    p = robot_base + local_endp_start
    goal = robot_base + local_endp_start

    gripper = [100, 100]

    display = Display(dt, ARMS_LENGTHS, start_pos=robot_base)
    step = 0
    sent = 2

    while True:
        display.render(q, goal)

        gripper = gripperControl(gripper)

        goal, dq_keyboard = keyboard_control(dt, goal)
        goal = cap_goal(goal)

        # Control
        local_goal = goal - robot_base

        # F_end can be replaced with RL action. array[2]
        F_end = controller.control_step(robot_arm.FK_end_p(), local_goal, dt)
        if dq_keyboard is None:
            p, q, dq = robot_arm.move_endpoint_xz(F_end, gripper)

        else:
            p, q, dq = robot_arm.move_joints(dq_keyboard)
            # Set goal exactly to current endpoint
            goal = p + robot_base
        t += dt

        # Render
        for pol in robot_arm.arm_regions:
            pol = [xy + robot_base for xy in pol]
            draw_rectangle_from_config(pol)
        # save state
        if len(q) == 3:
            state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], p[0], p[1]])
        else:
            state.append([t, q[0], q[1], dq[0], dq[1], p[0], p[1]])

        # try to keep it real time with the desired step time
        display.tick()
        pygame.display.flip()  # update display
        step += 1