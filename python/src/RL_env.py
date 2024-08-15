import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from keras.optimizers import Adam
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt   #cos(theta1) sin(theta1) cos(theta2) sin(theta2) thetaDot1 thetaDot2
import argparse
import subprocess as subp
import sys
import toml

import gym
from gym import spaces

from stable_baselines.common.env_checker import check_env
from stable_baselines import DQN, PPO2, A2C, ACKTR
from stable_baselines.common.cmd_util import make_vec_env



class CustomEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self):
    super(CustomEnv, self).__init__()
    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions:
    action_dim = 5
    self.tensions = [0, 0, 0, 0, 0]
    self._action_bound = 1
    self.goal_tip = [-0.00757662, -0.126819, 0.0931026]
    action_high = np.array([self._action_bound] * action_dim)
    self.action_space = spaces.Box(0, action_high, dtype=np.float32)
    # self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
    # Example for using image as input:
    self.observation_space = spaces.Box(low=0, high=20,
                                        shape=(5, ), dtype=np.float32)

  def step(self, action):
    observation = action + self.tensions
    self.tensions = observation
    subp.check_call(['./generate_tip',
                    'default.toml',
                    str(observation[0]), 
                    str(observation[1]), 
                    str(observation[2]), 
                    str(observation[3]), 
                    str(observation[4])])
    s = np.loadtxt('./interaction_test.txt')
    reward = np.sum(np.square (s[2] - self.goal_tip[2])) * 10
    done = False
    info = {}
    return np.array(observation), reward, done, info
  def reset(self):
    observation = [20, 0, 0, 0, 0]
    self.tensions = observation
    return np.array(observation)  # reward, done, info can't be included
  def render(self, mode='human'):
    return
  def close (self):
    return 


def populate_parser(parser = None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = "This is what I do!"
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    env = CustomEnv()
    # It will check your custom environment and output additional warnings if needed
    #check_env(env)
    #env = GoLeftEnv(grid_size=10)

    # obs = env.reset()
    # env.render()

    # print(env.observation_space)
    # print(env.action_space)
    # print(env.action_space.sample())

    # GO_LEFT = [1, 1, 1, 1, 1]
    # # Hardcoded best agent: always go left!
    # n_steps = 20
    # for step in range(n_steps):
    #   print("Step {}".format(step + 1))
    #   obs, reward, done, info = env.step(GO_LEFT)
    #   print('obs=', obs, 'reward=', reward, 'done=', done)
    #   env.render()
    #   if done:
    #     print("Goal reached!", "reward=", reward)
    #     break



    # wrap it
    env = make_vec_env(lambda: env, n_envs=1)
    model = PPO2('MlpPolicy', env, verbose=1).learn(5000)
    

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))





