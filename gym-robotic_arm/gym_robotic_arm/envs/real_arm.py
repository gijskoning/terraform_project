import gym
from gym import spaces

from arduino_communication import ArduinoControl
from constants import ARMS_LENGTHS
from dynamic_model import RobotArm3dof


class RealArm(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, port="COM6"):
        self.observation_space = None  # Not needed. Implemented by EnvWrapper
        self.action_space = spaces.Discrete(4)  # Discrete steps in x,z, z_rot and gripper
        arduino_control = ArduinoControl(port)
        self.robot_controller = RobotArm3dof(ARMS_LENGTHS, arduino_control=arduino_control)

    def step(self, action):
        """"
        action: numpy.ndarray containing 3 floats (x,z,gripper_pos)
            x,z [-1,1] movement of the endpoint (gripper)
            gripper_pos [0,1] 0 gripper fully open and 1 fully closed
        """
        #  todo Need actions defined for z rotation and gripper
        self.robot_controller.move_endpoint_xz(action[:2])
        obs = self._get_observation()
        # todo calculate reward
        reward = self._compute_reward()
        # todo check if done
        done = None  # Done is true means the episode is over.
        return obs, reward, done, []

    def reset(self):
        # todo Reset environment and return new observation
        self.robot_controller.reset()
        obs = self._get_observation()
        return obs

    def _compute_reward(self):
        # todo compute reward based on observation and robot model
        return 0.

    def _get_observation(self):
        # todo fix observation by getting camera image
        obs = None
        return obs

    def render(self, mode='human', height=100, width=100):
        # height and width are used to center crop the image.
        # Todo Retrieve camera image and visualize on screen
        # No need to return
        pass

    def seed(self, seed=None):
        # todo could implement seed when using random numbers
        pass
