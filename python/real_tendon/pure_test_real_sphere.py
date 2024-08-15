import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from keras.optimizers import Adam
import numpy as np
import matplotlib.pyplot as plt   #cos(theta1) sin(theta1) cos(theta2) sin(theta2) thetaDot1 thetaDot2
import math
import pandas as pd
import toml
from numpy.random import seed
import torch.nn.functional as F
import argparse
import subprocess as subp
import sys
import toml
import similaritymeasures
import copy
from numpy.random import seed
from tensorflow import set_random_seed
# seed(1)
# set_random_seed(1)
round_num = 4

saved_model_path = "./real_saved_models/sphere_test_1634508222/1634508222"
     
trained_policy = tf.contrib.saved_model.load_keras_model(saved_model_path)

e_v_test = np.loadtxt('./csv1/test_sphere_10_real.txt')
radius_list_test = np.loadtxt('./csv1/test_sphere_10_real_r.txt')

test_demonstrations = 1

obs_train_test = np.ones((test_demonstrations, 4))
ac_train_test = np.ones((test_demonstrations,round_num*3))
for i in range(obs_train_test.shape[0]):
    # obs_train_test[i][:3] = e_v_test[i*round_num][:]
    # obs_train_test[i][3] = radius_list_test[i*round_num]
    obs_train_test[i][:3] = [0.082, -0.02, 0.153]
    obs_train_test[i][3] = 0.05

for i in range(ac_train_test.shape[0]):
    for j in range(round_num):
        ac_train_test[i][j*3:j*3+3] = e_v_test[i*round_num + j][:]


exp_mean = []
exp_std = []
for i in range(1):
  exp_df = []
  current_array = []
  NN_array = []
  for iter_i in range(obs_train_test.shape[0]):
    end_list = []
    radius_list = []
    center_list = []
    traj = trained_policy.predict(np.array(obs_train_test[iter_i]).reshape(1,len(obs_train_test[iter_i])))[0]
    traj = traj.reshape(round_num,3)
    if True:
        #traj = traj.reshape(round_num,3)
        previous_configuration = [0,0,0,0]
        for i in range(round_num): 
            start = traj[i][:]
            print(start)
            subp.check_call(['./interactive_control',
                                '--robot', 'phys_robot_limits.toml',
                                '--start0', str(previous_configuration[0]), 
                                '--start1', str(previous_configuration[1]), 
                                '--start2', str(previous_configuration[2]), 
                                '--start3', str(previous_configuration[3]), 
                                '--des0', str(start[0]), 
                                '--des1', str(start[1]), 
                                '--des2', str(start[2])])
            configuration = np.loadtxt('./interaction_ik.txt')
            subp.check_call(['./generate_tip',
                            'phys_robot_limits.toml',
                            str(configuration[0]), 
                                str(configuration[1]), 
                                str(configuration[2]), 
                                str(configuration[3])])
            end_result = np.loadtxt('./interaction_test.txt')
            NN_array.append(start)
            current_array.append(configuration)
            # for end_i in range(end_result.shape[0]):
            #     end_list.append(end_result[end_i])
            end_list.append(end_result[:])
            previous_configuration = copy.deepcopy(configuration)
    end_arr = np.array(end_list)
    ac_train_compare = ac_train_test[iter_i].reshape(round_num,3)
    exp_df.append(similaritymeasures.frechet_dist(end_arr, ac_train_compare).tolist())
  exp_df = np.array(exp_df)
  exp_mean.append(exp_df.mean())
  exp_std.append(exp_df.std())

np.savetxt('./csv/real_sphere_test_NN.txt', NN_array)
pd.DataFrame(current_array).to_csv("./csv/real_sphere_test.csv")
print(exp_mean)
print(exp_std)
