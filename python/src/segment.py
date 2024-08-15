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


def populate_parser(parser = None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = "This is what I do!"
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    round_num = 7
    
    Dataset2 = pd.read_csv('./csv1/haptic_sphere_history_50.csv',sep=r'\s*,\s*',engine = 'python',na_values = '?',index_col=0)
    Dataset2.dropna()
    #print(Dataset2)
    data_with_panda = pd.get_dummies(Dataset2, drop_first = True)

    previous_history = data_with_panda.values

    previous_end = np.loadtxt('./csv1/haptic_sphere_history_50.txt')

    previous_center = np.loadtxt('./csv1/sphere_history_50_center.txt')

    previous_r =  np.loadtxt('./csv1/sphere_history_50_r.txt')

    total_new = 40

    new_history = previous_history[:total_new*round_num, :]

    new_end = previous_end[:total_new*round_num, :]

    new_center = previous_center[:total_new*round_num, :]

    new_r = previous_r[:total_new*round_num]

    pd.DataFrame(new_history).to_csv("./csv1/haptic_sphere_history_40.csv")
    np.savetxt('./csv1/haptic_sphere_history_40.txt', new_end)
    np.savetxt('./csv1/haptic_sphere_history_40_center.txt', new_center)
    np.savetxt('./csv1/haptic_sphere_history_40_r.txt', new_r)




    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))



