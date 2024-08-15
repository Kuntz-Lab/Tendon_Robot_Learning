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

leverage_num = 10
round_num = 9

e_v = np.loadtxt('./csv1/haptic_history_eight_50_r.txt')
width_list = np.loadtxt('./csv1/haptic_history_eight_50_r_width.txt')
height_list = np.loadtxt('./csv1/haptic_history_eight_50_r_height.txt')

e_v = copy.deepcopy(e_v[:leverage_num*round_num, :])
width_list = copy.deepcopy(width_list[:leverage_num*round_num])
height_list = copy.deepcopy(height_list[:leverage_num*round_num])

obs_train = np.ones((leverage_num, 6))
ac_train = np.ones((leverage_num,round_num*3))
for i in range(obs_train.shape[0]):
    obs_train[i][:3] = e_v[i*round_num][:]
    obs_train[i][3] = width_list[i*round_num]
    obs_train[i][4] = height_list[i*round_num]

for i in range(ac_train.shape[0]):
    for j in range(round_num):
        ac_train[i][j*3:j*3+3] = e_v[i*round_num + j][:]

# print(obs_train)
# print(ac_train)
trained_policy = keras.Sequential([
    layers.Dense(128, activation=tf.nn.relu, input_shape=[6]),
    layers.Dense(128, activation=tf.nn.relu),
    layers.Dense(128, activation=tf.nn.relu),
    layers.Dense(round_num*3)
  ])

optimizer = tf.keras.optimizers.Adam(0.01)
#adam = Adam(lr=0.01)

trained_policy.compile(loss='mean_squared_error',
                optimizer=optimizer,
                metrics=['mean_absolute_error', 'mean_squared_error'])
# policy.fit(obs_train, ac_train,
#                         epochs=5000, validation_split = 0, verbose=2, batch_size=32)

exp_mean = []
exp_std = []
for i in range(1):
  trained_policy.fit(obs_train, ac_train,
                        epochs=3000, validation_split = 0, verbose=2, batch_size=32)
  if True:
    end_list = []
    radius_list = []
    center_list = []
    width_list = []
    height_list = []
    current_array = []
    round_num = 9
    for iter_num in range(50):
        #begin_state = np.ones((1,3,5))
        x_center = 0 #0 + 0.05
        z_center = 0.07
        start_x = np.random.uniform(x_center - 0.04, x_center + 0.04)  # sphere with 0.055, circle with 0.05
        start_y = -0.12   # sphere with -0.10 while circle with -0.14
        start_z = np.random.uniform(z_center - 0.02 , z_center + 0.02)
        radius_min = 0.01
        radius_max = 0.04
        width = np.random.uniform(radius_min,radius_max)
        height = np.random.uniform(radius_min,radius_max)
        factor = 5

        # for i in range(3):
        #     begin_state[0][i][0] = start_x
        #     begin_state[0][i][1] = start_y
        #     begin_state[0][i][2] = start_z
        #     begin_state[0][i][3] = radius
        #print(begin_state)
        input_list = [start_x + width*2, start_y, start_z, width, height, 1]
        
        traj = trained_policy.predict(np.array(input_list[:]).reshape(1,len(input_list[:])))[0]
        #traj = model(obs_train)
        #start = start + delta
        #print(traj)
        traj = traj.reshape(round_num,3)
        previous_configuration = [0,0,0,0,0]
        for i in range(round_num): 
            start = traj[i][:]
            #print(start)
            subp.check_call(['./interactive_control',
                                '--robot', 'sphere_around.toml',
                                '--start0', str(previous_configuration[0]), 
                                '--start1', str(previous_configuration[1]), 
                                '--start2', str(previous_configuration[2]), 
                                '--start3', str(previous_configuration[3]), 
                                '--start4', str(previous_configuration[4]), 
                                '--des0', str(start[0]), 
                                '--des1', str(start[1]), 
                                '--des2', str(start[2])])
            configuration = np.loadtxt('./interaction_ik.txt')
            subp.check_call(['./generate_tip',
                            'sphere_around.toml',
                            str(configuration[0]), 
                                str(configuration[1]), 
                                str(configuration[2]), 
                                str(configuration[3]), 
                                str(configuration[4])])
            end_result = np.loadtxt('./interaction_test.txt')
            current_array.append(configuration)
            end_list.append(end_result[:])
            width_list.append(width)
            height_list.append(height)
            previous_configuration = copy.deepcopy(configuration)
    pd.DataFrame(current_array).to_csv("./csv/test_end_tensions_baseline_eight.csv")
    np.savetxt('./csv/test_end_list_baseline_eight.txt', end_list)
    np.savetxt('./csv/test_width_list_baseline_eight.txt', width_list)
    np.savetxt('./csv/test_height_list_baseline.txt', height_list)

    exp_data = np.loadtxt('./csv/test_end_list_baseline_eight.txt')
    exp_width = np.loadtxt('./csv/test_width_list_baseline_eight.txt')
    exp_height = np.loadtxt('./csv/test_height_list_baseline.txt')
    num_data = np.loadtxt('./csv1/haptic_history_eight_50_r.txt')
    num_width = np.loadtxt('./csv1/haptic_history_eight_50_r_width.txt')
    num_height = np.loadtxt('./csv1/haptic_history_eight_50_r_height.txt')

    num_data = copy.deepcopy(num_data[:leverage_num*round_num, :])
    num_width = copy.deepcopy(num_width[:leverage_num*round_num])
    num_height = copy.deepcopy(num_height[:leverage_num*round_num])
    scale_factor = 40

    #round_num = 9

    for i in range(len(exp_data)):
        if(i%round_num == 0):
            each_start = copy.deepcopy(exp_data[i,:])
        exp_data[i,0] = each_start[0] + ((exp_data[i,0] - each_start[0])/(exp_width[i]*scale_factor))
        exp_data[i,2] = each_start[2] + ((exp_data[i,2] - each_start[2])/(exp_height[i]*scale_factor))

    for i in range(len(num_data)):
        if(i%round_num == 0):
            each_start = copy.deepcopy(num_data[i,:])
        num_data[i,0] = each_start[0] + ((num_data[i,0] - each_start[0])/(num_width[i]*scale_factor))
        num_data[i,2] = each_start[2] + ((num_data[i,2] - each_start[2])/(num_height[i]*scale_factor))




    #change = [[], [], [], []]

    change = []
    for i in range(round_num):
        change.append([])
    # print(exp_data.shape)
    # print(num_data.shape)
    for i in range(int(num_data.shape[0]/round_num)):
        for j in range(1,round_num):
            change[j-1].append(num_data[i*round_num+j,:] - num_data[i*round_num,:])
    mean_change = []
    for i in range(len(change)):
        print(np.array(change[i]).shape)
        mean_change.append(np.mean(np.array(change[i]), axis = 0))
    #print(mean_change)

    exp_df = []
    num_df = []

    exp_current = np.zeros((round_num,3))
    num_current = np.zeros((round_num,3))
    avg_current = np.zeros((round_num,3))
    for i in range(int(exp_data.shape[0]/round_num)):
        exp_current = exp_data[i*round_num:i*round_num+round_num, :]
        avg_current[0,:] = exp_data[i*round_num, :]
        for j in range(1,round_num):
            avg_current[j,:] = avg_current[0,:] + mean_change[j-1]
        exp_df.append(similaritymeasures.frechet_dist(exp_current, avg_current).tolist())
        #num_df.append(similaritymeasures.frechet_dist(num_current, avg_current).tolist())

    for i in range(int(num_data.shape[0]/round_num)):
        num_current = num_data[i*round_num:i*round_num+round_num, :]
        avg_current[0,:] = num_data[i*round_num, :]
        for j in range(1,round_num):
            avg_current[j,:] = avg_current[0,:] + mean_change[j-1]
        #exp_df.append(similaritymeasures.frechet_dist(exp_current, avg_current).tolist())
        num_df.append(similaritymeasures.frechet_dist(num_current, avg_current).tolist())
    print(exp_df)
    print(num_df)
    exp_df = np.array(exp_df)
    num_df = np.array(num_df)
    exp_mean.append(exp_df.mean())
    exp_std.append(exp_df.std())
    if(exp_df.mean() == min(exp_mean)):
        import time
        saved_model_path = "./saved_models/haptic_plane_eight_10_traj_"+str(int(time.time()))
        tf.contrib.saved_model.save_keras_model(trained_policy, saved_model_path)

print(exp_mean)
print(exp_std)
# import time
# saved_model_path = "./saved_models/sphere_history_50_big_5000_"+str(int(time.time()))
# tf.contrib.saved_model.save_keras_model(policy, saved_model_path)

