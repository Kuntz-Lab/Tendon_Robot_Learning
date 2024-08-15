import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from keras.optimizers import Adam
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt   #cos(theta1) sin(theta1) cos(theta2) sin(theta2) thetaDot1 thetaDot2

# A = np.loadtxt('/home/gao-4144/yixuan/txt_result/new_fix_3_obstacles_5e2_16.txt')
# obs_train = A[:,4:14]
# ac_train = A[:,14:]
# length = A.shape[0]
# shuffle = np.random.permutation(length)
# obs_train = obs_train[shuffle]
# ac_train = ac_train[shuffle]

#saved_model_path = "/home/gao-4144/yixuan/saved_models/Swimmer1578595377"
saved_model_path = "./saved_models/good_switch_pre_3e5_success_data1610157909"
trained_policy = tf.contrib.saved_model.load_keras_model(saved_model_path)

start = [19.5, 0.5, 0.5, 0.5, 0.5]
goal = [0, 10, 10, 0, 10]

s = start
i = 0
current_array = [] 
while True:
    i = i + 1
    print(s)
    current_array.append(s)
    delta = trained_policy.predict(np.array(s[:]).reshape(1,len(s[:])))[0]
    s = s + delta
    
    if(i>10):
        saved_array = np.array(current_array)
        for j in range(saved_array.shape[0]):
            for k in range(saved_array.shape[1]):
                saved_array[j][k] = "{:.4f}".format(saved_array[j][k])
        pd.DataFrame(saved_array).to_csv("./csv/learning_test_1.csv")
        break

