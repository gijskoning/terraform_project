import numpy as np
from gym import spaces
from gym_robotic_arm.constants import ARMS_LENGTHS

from gym_robotic_arm.dynamic_model import RobotArm3dof
from gym_robotic_arm.envs.RealRobotArm import RobotArm


class SimulationRobotArm(RobotArm):

    def __init__(self, pixels=False, reward_type=None):
        robot_controller = RobotArm3dof(ARMS_LENGTHS, arduino_control=None)
        super().__init__(robot_controller)

        # todo not correct space yet
        # this is still just a state space
        self.observation_space = spaces.Box(0, 1, shape=(4,), dtype=np.float32)

        self.action_space = spaces.Box(low=-10, high=10, shape=(3,), dtype=np.float32) # three actions with bound of [-10,10]
        self.pixels = pixels  # If state with pixels

    def _get_observation(self):
        # todo fix observation by getting camera image
        if self.pixels:
            obs = None
            raise NotImplementedError()
        else:
            obs = np.zeros(4).astype(np.float32)
            # todo for now only return endpoint controller
            obs[:2] = self.robot_controller.FK_end_p()
        return {"observation": obs, "achieved_goal": np.zeros(4), "desired_goal": np.zeros(4)}

    def render(self, mode='human', height=100, width=100):
        # height and width are used to center crop the image.
        # Todo Retrieve camera image and visualize on screen
        # No need to return
        pass

if __name__ == '__main__':
    # testing
    env = SimulationRobotArm()
    action = env.action_space.sample()

    print(action)
    print(env.action_space.shape)
    obs = env.step(action)[0]["observation"]
    print(obs)
