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
from tensorflow import set_random_seed
import numpy as np
import pandas as pd
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
import similaritymeasures
import copy
from numpy.random import seed
from tensorflow import set_random_seed
from matplotlib import pyplot as plt
#plt.style.use('dark_background')
from keras.preprocessing.text import Tokenizer
from keras.preprocessing.sequence import pad_sequences
#from sklearn.model_selection import train_test_split
#from keras.utils import to_categorical
from keras.models import Sequential
from keras.layers import Dense, Dropout, Embedding, LSTM, GlobalMaxPooling1D, SpatialDropout1D
seed(1)
set_random_seed(1)


# Init_XY = np.loadtxt('./csv1/state_haptic_circle_20_variable_history_1.txt')
# #print(Init_XY)
# X_and_Y = Init_XY



# obs_train = X_and_Y
# ac_train = np.loadtxt('./csv1/action_haptic_circle_20_variable_history_1.txt')
# print(obs_train)
# print(ac_train)

e_v = np.loadtxt('./variable_list.txt')

round_num = 4
obs_train = []
ac_train = []
obs_train_1 = []
ac_train_1 = []

leap = 0

for i in range(e_v.shape[0] - 1):
    if(e_v[i][3] == e_v[i+1][3]):
        obs_train.append([e_v[i, 0] - e_v[i, 3], e_v[i, 1] - e_v[i, 4], e_v[i, 2] - e_v[i, 5]])        
        ac_train.append([e_v[i+1, 0] - e_v[i, 0], e_v[i+1, 1] - e_v[i, 4], e_v[i+1, 2] - e_v[i, 2]])
        #continue
    else:
        if(leap == 0):
            leap = 1
            obs_train = []
            ac_train = []
        else:
            obs_train = np.array(obs_train)
            ac_train = np.array(ac_train)
            print(obs_train.shape)
            print(ac_train.shape)
            obs_train_1.append(obs_train)
            ac_train_1.append(ac_train)
            obs_train = []
            ac_train = []

#print(obs_train_1)
obs_train = np.array(obs_train_1)
ac_train = np.array(ac_train_1)
print(obs_train.shape)
print(ac_train.shape)

saved_model_path = "./saved_models_1/gru_10_1616374906/1616374906"
trained_policy = tf.contrib.saved_model.load_keras_model(saved_model_path)




current_array = []
if True:
    end_list = []
    num_df = []
    iter_i = 0
    for k in range(1):#int(Init_XY.shape[0]/4)):
        #start =  X_and_Y[k*4,:] #[ 0, 10.7584, 7.17972,  8.2993, 7.60464] #[0,11,5.5,7.6,6.6]#X_and_Y[18,:]+0.1 #[      0, 9.90974, 6.10127, 7.92678, 5.37951]#[0, 8.40811, 9.1432, 6.74058, 8.7633]#X_and_Y[8,:]
        x_center = 0
        z_center = 0.07
        start_x = np.random.uniform(x_center - 0.04, x_center + 0.04)  # sphere with 0.055, circle with 0.05
        start_y = -0.14   # sphere with -0.10 while circle with -0.14
        start_z = np.random.uniform(z_center - 0.02 , z_center + 0.02)

        # subp.check_call(['./interactive_control',
        #                     '--robot', 'sphere_around.toml',
        #                     '--start0', str(0), 
        #                     '--start1', str(0), 
        #                     '--start2', str(0), 
        #                     '--start3', str(0), 
        #                     '--start4', str(0), 
        #                     '--des0', str(start_x), 
        #                     '--des1', str(start_y), 
        #                     '--des2', str(start_z)])
                
        # start = np.zeros(8,)
        # start[:5] = np.loadtxt('./interaction_ik.txt')
        # #start.extend([0,0,0])
        
        i = 0
        #start = [0, 0, 0.2]
        
        # while(e_v[iter_i, 3] == e_v[iter_i+1, 3]):
        #     iter_i += 1
        # center = [e_v[iter_i, 3] - 0.05, e_v[iter_i, 4], e_v[iter_i, 5]]
        # start = [e_v[iter_i, 3], e_v[iter_i, 4], e_v[iter_i, 5]]
        # ref = [e_v[iter_i, 3], e_v[iter_i, 4], e_v[iter_i, 5]]
        while(e_v[iter_i, 3] == e_v[iter_i+1, 3]):
                iter_i += 1
        center = [e_v[iter_i, 3] - 0.05, e_v[iter_i, 4], e_v[iter_i, 5]]
        start = [e_v[iter_i, 3], e_v[iter_i, 4], e_v[iter_i, 5]]
        ref = [e_v[iter_i, 3], e_v[iter_i, 4], e_v[iter_i, 5]]
        iter_i += 1
        points = [[center[0] + 0.05, center[1], center[2]],[center[0], center[1], center[2] - 0.05], [center[0] - 0.05, center[1], center[2]], [center[0], center[1], center[2] + 0.05], [center[0] + 0.05, center[1], center[2]]]
        previous = points[0]
        previous_2 = points[0]
        print('center')
        print(center)
        print(points)
        start_list = []
        current_configuration = []
        input_list = []
        
        while True:
            i = i + 1
            print(start)
            subp.check_call(['./interactive_control',
                            '--robot', 'sphere_around.toml',
                            '--start0', str(0), 
                            '--start1', str(0), 
                            '--start2', str(0), 
                            '--start3', str(0), 
                            '--start4', str(0), 
                            '--des0', str(start[0]), 
                            '--des1', str(start[1]), 
                            '--des2', str(start[2])])
            configuration = np.loadtxt('./interaction_ik.txt')
            current_configuration.append(configuration)
            
            for j in range(3):
                input_list.append(start[j])
            input_arr = np.array(input_list[:]).reshape(1,i,3)
            #print(input_arr.shape)
            delta = trained_policy.predict(input_arr)[0]
            #print(delta)
            #input_list = [previous_2[0] - ref[0],previous_2[1] - ref[1],previous_2[2] - ref[2], previous[0] - ref[0],previous[1] - ref[1],previous[2] - ref[2],start[0] - ref[0], start[1] - ref[1], start[2] - ref[2]]
            
            # delta = trained_policy.predict(np.array(input_list[:]).reshape(1,len(input_list[:])))[0]
            # previous_2 = copy.deepcopy(previous)
            # previous = copy.deepcopy(start)
            start = [start[0] + delta[-1][0], start[1] + delta[-1][1], start[2] + delta[-1][2]] 
            start_list.append(start)
            #input_list = []
            if(i>10):
                pd.DataFrame(current_configuration).to_csv("./csv1/lstm_test_0.csv")
                #num_df.append(similaritymeasures.frechet_dist(np.array(points), np.array(start_list)).tolist())
                break

