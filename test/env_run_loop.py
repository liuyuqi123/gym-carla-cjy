"""
This script is supposed to test usage and run loop of a Carla Env.
"""

import os
import sys
import glob

# import carla path
from carla_config import config

carla_version = config['carla_version']
root_path = config['root_path']

carla_root = os.path.join(root_path, 'CARLA_' + carla_version)
carla_path = os.path.join(carla_root, 'PythonAPI')
sys.path.append(carla_path)
sys.path.append(os.path.join(carla_root, 'PythonAPI/carla'))
sys.path.append(os.path.join(carla_root, 'PythonAPI/carla/agents'))

try:
    sys.path.append(glob.glob(carla_path + '/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from gym_carla.envs.drl_env.carla_env_1 import CarlaEnv_1


class TestEnvRunLoop:

    def __init__(self,
                 params,
                 ):

        self.env = CarlaEnv_1(params)

    def run(self):

        self.env.reset()

        print('')


def main():

    env_name = 'carla-v0',
    discount = 1.0,

    number_of_vehicles = 100,
    number_of_walkers = 0,
    display_size = 256,
    max_past_step = 1,
    dt = 0.1,
    discrete = False,
    discrete_acc = [-3.0, 0.0, 3.0],
    discrete_steer = [-0.2, 0.0, 0.2],
    continuous_accel_range = [-3.0, 3.0],
    continuous_steer_range = [-0.3, 0.3],
    ego_vehicle_filter = 'vehicle.lincoln*',
    port = 2000,
    town = 'Town03',
    task_mode = 'random',
    max_time_episode = 500,
    max_waypt = 12,
    obs_range = 32,
    lidar_bin = 0.5,
    d_behind = 12,
    out_lane_thres = 2.0,
    desired_speed = 8,
    max_ego_spawn_times = 200,
    display_route = True,
    pixor_size = 64,
    pixor = False,
    obs_channels = None,
    action_repeat = 1

    env_params = {
        'number_of_vehicles': number_of_vehicles,
        'number_of_walkers': number_of_walkers,
        'display_size': display_size,  # screen size of bird-eye render
        'max_past_step': max_past_step,  # the number of past steps to draw
        'dt': dt,  # time interval between two frames
        'discrete': discrete,  # whether to use discrete control space
        'discrete_acc': discrete_acc,  # discrete value of accelerations
        'discrete_steer': discrete_steer,  # discrete value of steering angles
        'continuous_accel_range': continuous_accel_range,  # continuous acceleration range
        'continuous_steer_range': continuous_steer_range,  # continuous steering angle range
        'ego_vehicle_filter': ego_vehicle_filter,  # filter for defining ego vehicle
        'port': port,  # connection port
        'town': town,  # which town to simulate
        'task_mode': task_mode,  # mode of the task, [random, roundabout (only for Town03)]
        'max_time_episode': max_time_episode,  # maximum timesteps per episode
        'max_waypt': max_waypt,  # maximum number of waypoints
        'obs_range': obs_range,  # observation range (meter)
        'lidar_bin': lidar_bin,  # bin size of lidar sensor (meter)
        'd_behind': d_behind,  # distance behind the ego vehicle (meter)
        'out_lane_thres': out_lane_thres,  # threshold for out of lane
        'desired_speed': desired_speed,  # desired speed (m/s)
        'max_ego_spawn_times': max_ego_spawn_times,  # maximum times to spawn ego vehicle
        'display_route': display_route,  # whether to render the desired route
        'pixor_size': pixor_size,  # size of the pixor labels
        'pixor': pixor,  # whether to output PIXOR observation
    }

    TestEnvRunLoop(
        env_params,
    )


if __name__ == '__main__':

    main()

