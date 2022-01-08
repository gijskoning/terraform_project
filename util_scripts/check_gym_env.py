if __name__ == '__main__':
    # Check if new gym environment is added
    from gym import envs, register

    for i in envs.registry.all():
        print(i.id)

    # import gym_robotic_arm
    # from gym.envs.registration import register
    import gym
    env_dict = gym.envs.registration.registry.env_specs.copy()
    for env in env_dict:
        if "RobotArm" in env:
            print("Env:",env)


    # for i in envs.registry.all():
    #     print(i.id)
    # env = gym.make('RealRobotArm-v0', robot_controller=None)
    print("No error :)")