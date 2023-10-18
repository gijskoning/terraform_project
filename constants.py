# ARMS_LENGTHS = [187.2, 117.5, 80.4 + 119]  # mm Third arm is last arm + gripper
import numpy as np

# ARMS_LENGTHS = np.array([0.9, 0.3, 0.3])  # mm Third arm is last arm + gripper
ARMS_LENGTHS = np.array([0.4, 0.3, 0.1])  # in Meter. Third arm is last arm + gripper
ARM_WIDTH = 0.05

CONTROL_DT = 0.02  # integration step time

TOTAL_ARM_LENGTH = np.sum(ARMS_LENGTHS)
ZERO_POS_BASE = 0.  # meter Base servo plus base

INITIAL_CONFIG_Q = np.array([np.pi*0.1, -np.pi * 0.1, -np.pi * 0.0])  # Initial robot angles


# PID CONTROLLER PARAMETERS
Kp = 20  # proportional gain
Ki = 0.2  # integral gain
Kd = 0.1  # derivative gain
goal_reached_length = 0.02
goal_reached_angle = 10 * np.pi / 180
inner_waypoint_step_size = 0.01

velocity_constraint = np.array([0.01, 0.2, 0.6])*30 # rad/s