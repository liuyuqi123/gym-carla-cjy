from gym.envs.registration import register

register(
    id='carla-v0',
    entry_point='gym_carla.envs:CarlaEnv',
)

# fixme test this
register(
    id='carla-v1',
    entry_point='gym_carla.envs.drl_env.carla_env_1:CarlaEnv_1',
)