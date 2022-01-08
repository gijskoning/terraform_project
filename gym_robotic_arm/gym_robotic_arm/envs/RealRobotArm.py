from abc import abstractmethod

import gym
from gym_robotic_arm.envs.waveshare_camera import WaveShareCamera
from gym import spaces

from gym_robotic_arm.constants import ARMS_LENGTHS, TOTAL_ARM_LENGTH

from gym_robotic_arm.dynamic_model import RobotArm3dof


class RobotArm(gym.Env):
    # need to copy code from https://github.com/TianhongDai/hindsight-experience-replay
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, robot_controller):
        self.robot_controller = robot_controller

        self._max_episode_steps = 500
        self.episode_step = 0
        self.sim = None


    def step(self, action):
        """"
        action: numpy.ndarray containing 3 floats (x,z,gripper_pos)
            x,z [-1,1] movement of the endpoint (gripper)
            gripper_pos [0,1] 0 gripper fully open and 1 fully closed
        """
        #  todo still need actions defined for z rotation and gripper
        self.robot_controller.move_endpoint_xz(action[:2])
        obs = self._get_observation()
        reward = self._compute_reward()
        # todo improve checking when episode is done
        done = self.episode_step >= self._max_episode_steps  # Done is true means the episode is over.
        self.episode_step += 1
        info = {'is_success': False}
        return obs, reward, done, info  # obs, reward, done, info

    def reset(self):
        # todo Reset environment and return new observation
        self.robot_controller.reset()
        self.episode_step = 0

        obs = self._get_observation()
        return obs

    def _compute_reward(self):
        # todo compute reward based on observation and robot model

        # Basic reward enforcing robot to go to certain goal_x
        end_p = self.robot_controller.FK_end_p()
        goal_x = TOTAL_ARM_LENGTH - 0.1
        return -abs(goal_x - end_p[0])

    @abstractmethod
    def _get_observation(self):
        raise NotImplementedError()

    @abstractmethod
    def render(self, mode='human', height=100, width=100):
        raise NotImplementedError()
        # height and width are used to center crop the image.
        # No need to return

    def seed(self, seed=None):
        # todo could implement seed when using random numbers
        pass

    def close(self):
        pass

    def _render_callback(self):
        # Required by RL repo
        pass

class RealRobotArm(RobotArm):

    def __init__(self, port="COM6", reward_type=None):
        from gym_robotic_arm.arduino_communication import ArduinoControl

        arduino_control = ArduinoControl(port)
        robot_controller = RobotArm3dof(ARMS_LENGTHS, arduino_control=arduino_control)
        super().__init__(robot_controller)
        # todo Waveshare is not yet implemented
        self.camara = WaveShareCamera()

    def _get_observation(self):
        obs = self.camara.get_frame()
        return {"observation": obs, "achieved_goal": None, "desired_goal": None}

    def render(self, mode='human', height=100, width=100):
        # height and width are used to center crop the image.
        # Todo Retrieve camera image and visualize on screen
        # No need to return
        pass
