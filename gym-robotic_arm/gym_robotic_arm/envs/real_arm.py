import gym
from gym import spaces


class RealArm(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self):
        self.observation_space = None  # Not needed. Implemented by EnvWrapper
        self.action_space = spaces.Discrete(4)  # Discrete steps in x,z, z_rot and gripper

    def step(self, action):
        #  todo Need actions defined for at least x,z movement plus gripper
        pass

    def reset(self):
        # todo Reset environment and return new observation
        pass

    def render(self, mode='human', height=100, width=100):
        # height and width are used to center crop the image.
        # Todo Retrieve camera image
        pass

    def seed(self, seed=None):
        # todo could implement seed when using random numbers
        pass
