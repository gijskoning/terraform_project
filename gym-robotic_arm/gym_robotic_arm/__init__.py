from gym.envs.registration import register

register(
    id='RealArm-v0',
    entry_point='gym_robotic_arm.envs:RealArm'#,
    # max_episode_steps=200
)

