# SIMULATION PARAMETERS
import numpy as np
import pygame
from pygame import K_RIGHT, K_LEFT, K_SPACE, K_UP, K_DOWN, K_a, K_w, K_z, K_s, K_x, K_c, K_d, K_r

from gym_robotic_arm.constants import ARMS_LENGTHS, TOTAL_ARM_LENGTH, ZERO_POS_BASE, INITIAL_CONFIG_Q, CONTROL_DT
from gym_robotic_arm.dynamic_model import PIDController, RobotArm3dof
import shelve

from sim_utils import length
from visualize_robot_arm import Display, display_to_coordinate

dt = CONTROL_DT
# ROBOT     PARAMETERS
x0 = 0.0  # base x position
y0 = 0.0  # base y position

# PID CONTROLLER PARAMETERS
Kp = 15  # proportional gain
Ki = 0.3  # integral gain
Kd = 0.1  # derivative gain
global_db = shelve.open("cache")


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
    velocity_limits = [2., 2.]
    waypoints = []
    if 'waypoints' in global_db:
        waypoints = global_db['waypoints']

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
    left_click = False
    mouse_released = False
    enable_robot = True
    while True:
        display.render(q, goal, waypoints) # RENDER
        mouse_x_display, mouse_y_display = pygame.mouse.get_pos()
        mouse_x, mouse_y = display_to_coordinate(mouse_x_display, mouse_y_display)
        gripper = gripperControl(gripper)

        new_left_click = pygame.mouse.get_pressed()[0]
        if not new_left_click and left_click:
            mouse_released = True

        if mouse_released:
            goal = np.array([mouse_x, mouse_y])
            waypoints.append(goal)
        new_keyboard_goal, dq_keyboard = keyboard_control(dt, goal)

        keys = pygame.key.get_pressed()
        if keys[K_SPACE]:
            enable_robot = not enable_robot
            print("enable_robot", enable_robot)
        if keys[K_w]:
            if 'waypoints' not in global_db:
                print('save waypoints', waypoints)
                global_db['waypoints'] = waypoints
        if keys[K_r]:
            waypoints = []
            if 'waypoints' in global_db:
                print('reset waypoints', waypoints)
                del global_db['waypoints']
                waypoints = []
        # dq_keyboard = None
        # goal = cap_goal(goal) # cap goal based on arm length

        # Control
        local_goal = goal - robot_base

        # F_end can be replaced with RL action. array[2]
        if enable_robot:
            # gets the end effector goal
            F_end = controller.control_step(robot_arm.FK_end_p(), local_goal, dt)

            if dq_keyboard is None:
                p, q, dq = robot_arm.move_endpoint_xz(F_end, gripper)  # this requests a endpoint force and returns pos, angle,angle_speed
            else:
                # not used for goals
                p, q, dq = robot_arm.move_joints(dq_keyboard)
                # Set goal exactly to current endpoint
                goal = p + robot_base
            # save state
            if len(q) == 3:
                state.append([t, q[0], q[1], q[2], dq[0], dq[1], dq[2], p[0], p[1]])
            else:
                state.append([t, q[0], q[1], dq[0], dq[1], p[0], p[1]])
        t += dt

        # try to keep it real time with the desired step time
        display.tick()
        pygame.display.flip()  # update display
        step += 1
        left_click = new_left_click
        mouse_released = False