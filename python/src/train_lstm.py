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
# policy = keras.Sequential([
#     layers.LSTM(3, input_shape = (245,3)), 
#     layers.Dense(128, activation=tf.nn.relu),
#     layers.Dense(128, activation=tf.nn.relu),
#     layers.Dense(3)
#   ])

# optimizer = tf.keras.optimizers.RMSprop(0.001)
# adam = Adam(lr=1e-4)

# policy.compile(loss='mean_squared_error',
#                 optimizer=optimizer,
#                 metrics=['mean_absolute_error', 'mean_squared_error'])
# policy.fit(obs_train, ac_train,
#                         epochs=500, validation_split = 0.2, verbose=2, batch_size=2)

# model_lstm = Sequential()
# model_lstm.add(Embedding(input_dim = 3, output_dim = 256, input_length = 245))
# model_lstm.add(SpatialDropout1D(0.3))
# model_lstm.add(LSTM(256))
# model_lstm.add(Dense(256, activation = 'relu'))
# model_lstm.add(Dropout(0.3))
# model_lstm.add(Dense(3, activation = 'softmax'))
# model_lstm.compile(
#     loss='categorical_crossentropy',
#     optimizer='Adam',
#     metrics=['accuracy']
# )
# history = model_lstm.fit(
#     obs_train,
#     ac_train,
#     validation_split = 0.1,
#     epochs = 8,
#     batch_size = 2
# )

model = keras.Sequential()
#model.add(layers.Embedding(input_dim=3, output_dim=64))
#model.add(layers.Embedding(input_dim=1000, output_dim=64))

# The output of GRU will be a 3D tensor of shape (batch_size, timesteps, 256)

#model.add(layers.LSTM(256, return_sequences=True, return_state=True))
model.add(layers.RNN(256, return_sequences=True))

# The output of SimpleRNN will be a 2D tensor of shape (batch_size, 128)
#model.add(layers.SimpleRNN(128))
model.add(layers.Dense(128, activation=tf.nn.relu))
model.add(layers.Dense(128, activation=tf.nn.relu))
model.add(layers.Dense(3))


#model.summary()


model.compile(
    loss='mean_squared_error',
    optimizer='Adam',
    metrics=['mean_absolute_error', 'mean_squared_error']
)



history = model.fit(
    obs_train,
    ac_train,
    validation_split = 0.1,
    epochs = 10,
    batch_size = 2
)



import time
saved_model_path = "./saved_models_1/rnn_10_"+str(int(time.time()))
tf.contrib.saved_model.save_keras_model(model, saved_model_path)

# model_lstm = Sequential()
# model_lstm.add(Embedding(input_dim = 9, output_dim = 256))
# model_lstm.add(SpatialDropout1D(0.3))
# model_lstm.add(LSTM(256))
# model_lstm.add(Dense(256, activation = 'relu'))
# model_lstm.add(Dropout(0.3))
# model_lstm.add(Dense(3, activation = 'softmax'))
# model_lstm.compile(
#     loss='categorical_crossentropy',
#     optimizer='Adam',
#     metrics=['accuracy']
# )
# history = model_lstm.fit(
#     obs_train,
#     ac_train,
#     validation_split = 0.1,
#     epochs = 8,
#     batch_size = 512
# )
# import time
# saved_model_path = "./saved_models_1/test_lstm_"+str(int(time.time()))
# tf.contrib.saved_model.save_keras_model(model_lstm, saved_model_path)