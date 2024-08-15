import torch
from torch.autograd import Variable
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
from sklearn.kernel_ridge import KernelRidge
# seed(1)
# set_random_seed(1)


e_v = np.loadtxt('./csv1/haptic_train_double.txt')
radius_list = np.loadtxt('./csv1/haptic_train_double_r.txt')

round_num = 5

total_demonstrations = 20

obs_train = np.ones((total_demonstrations, 6))
ac_train = np.ones((total_demonstrations,round_num*3))
for i in range(obs_train.shape[0]):
    obs_train[i][:3] = e_v[i*round_num][:]
    obs_train[i][3] = radius_list[i*round_num][0]
    obs_train[i][4] = radius_list[i*round_num][1]

for i in range(ac_train.shape[0]):
    for j in range(round_num):
        ac_train[i][j*3:j*3+3] = e_v[i*round_num + j][:]


clf = KernelRidge(alpha=0.01, kernel = "rbf", gamma = 10)
clf.fit(obs_train, ac_train)

e_v_test = np.loadtxt('./csv1/haptic_test_double.txt')
radius_list_test = np.loadtxt('./csv1/haptic_test_double_r.txt')

test_demonstrations = 10

obs_train_test = np.ones((test_demonstrations, 6))
ac_train_test = np.ones((test_demonstrations,round_num*3))
for i in range(obs_train_test.shape[0]):
    obs_train_test[i][:3] = e_v_test[i*round_num][:]
    obs_train_test[i][3] = radius_list_test[i*round_num][0]
    obs_train_test[i][4] = radius_list_test[i*round_num][1]

for i in range(ac_train_test.shape[0]):
    for j in range(round_num):
        ac_train_test[i][j*3:j*3+3] = e_v_test[i*round_num + j][:]


exp_df = []
for iter_i in range(obs_train_test.shape[0]):
    end_list = []
    radius_list = []
    center_list = []
    current_array = []
    traj = clf.predict(np.array(obs_train_test[iter_i]).reshape(1,len(obs_train_test[iter_i])))
    if True:
        traj = traj.reshape(round_num,3)
        previous_configuration = [0,0,0,0,0]
        for i in range(round_num): 
            start = traj[i][:]
            print(start)
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
            print(configuration)
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
            previous_configuration = copy.deepcopy(configuration)
    end_arr = np.array(end_list)
    ac_train_compare = ac_train_test[iter_i].reshape(round_num,3)
    exp_df.append(similaritymeasures.frechet_dist(end_arr, ac_train_compare).tolist())
exp_df = np.array(exp_df)
print(exp_df)
print([exp_df.mean(), exp_df.std()])

