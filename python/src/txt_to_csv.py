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

e_v = np.loadtxt('./csv/end_list_sphere_2_10_test_0.txt')
pd.DataFrame(e_v).to_csv("./csv/end_list_sphere_2_10_test_0.csv")