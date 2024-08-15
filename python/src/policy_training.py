import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from keras.optimizers import Adam
import numpy as np
import matplotlib.pyplot as plt   #cos(theta1) sin(theta1) cos(theta2) sin(theta2) thetaDot1 thetaDot2
import math
import pandas as pd

Dataset2 = pd.read_csv('/Users/huangyixuan/Downloads/chained.csv',sep=r'\s*,\s*',engine = 'python',na_values = '?',index_col=0)
Dataset2.dropna()
#print(Dataset2)
data_with_panda = pd.get_dummies(Dataset2, drop_first = True)
X_and_Y = data_with_panda.values
A = X_and_Y

A_0 = X_and_Y[1:]
A_1 = X_and_Y[:-1]
action = A_0 - A_1


#e_v = np.loadtxt()

print(A.shape)
obs_train = A[:-1,:]
ac_train = action
policy = keras.Sequential([
    layers.Dense(64, activation=tf.nn.relu, input_shape=[5]),
    layers.Dense(64, activation=tf.nn.relu),
    layers.Dense(5)
  ])

optimizer = tf.keras.optimizers.RMSprop(0.001)
adam = Adam(lr=1e-4)

policy.compile(loss='mean_squared_error',
                optimizer=optimizer,
                metrics=['mean_absolute_error', 'mean_squared_error'])
policy.fit(obs_train, ac_train,
                        epochs=500, validation_split = 0.2, verbose=2, batch_size=32)
import time
saved_model_path = "./saved_models/good_switch_pre_3e5_success_data"+str(int(time.time()))
tf.contrib.saved_model.save_keras_model(policy, saved_model_path)