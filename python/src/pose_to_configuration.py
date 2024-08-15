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


current_array = []
end_list = []

if True:
        #traj = traj.reshape(round_num,3)
        previous_configuration = [0,0,0,0,0]
        pre_traj = np.array([[0.5733, -0.0485,  0.4555,  0.5490, -0.0232,  0.4553,  0.5733,  0.0029,
          0.4552,  0.5991,  0.0265,  0.4555,  0.5732,  0.0512,  0.4553,  0.5497,
          0.0263,  0.4554,  0.5761,  0.0046,  0.4554,  0.5976, -0.0237,  0.4555,
          0.5762, -0.0492,  0.4552]]).reshape(9,3)
        traj = np.zeros((9,3))
        for i in range(traj.shape[0]):
            traj[i][0] = pre_traj[i][1]
            traj[i][1] = -0.12 + (pre_traj[i][2] - 0.455)
            traj[i][2] = pre_traj[i][0] - 0.505
        print(traj)
        for i in range(traj.shape[0]): 
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
            subp.check_call(['./generate_tip',
                            'sphere_around.toml',
                            str(configuration[0]), 
                                str(configuration[1]), 
                                str(configuration[2]), 
                                str(configuration[3]), 
                                str(configuration[4])])
            end_result = np.loadtxt('./interaction_test.txt')
            current_array.append(configuration)
            # for end_i in range(end_result.shape[0]):
            #     end_list.append(end_result[end_i])
            end_list.append(end_result[:])
            previous_configuration = copy.deepcopy(configuration)

pd.DataFrame(current_array).to_csv("./csv/test_pointsdf.csv")
    