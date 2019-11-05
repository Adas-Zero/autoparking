from gym.envs.registration import register

register(
        id='autoparkenv-v0',
        entry_point='autoparkenv.envs:AutoParkEnv',)
