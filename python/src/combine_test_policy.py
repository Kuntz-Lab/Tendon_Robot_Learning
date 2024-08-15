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

    saved_model_path = "./saved_models/good_switch_pre_3e5_success_data1610317944"
    e_v = np.loadtxt('./csv/example.txt')
    trained_policy = tf.contrib.saved_model.load_keras_model(saved_model_path)

    start = [20, 0, 0, 0, 0] 
    goal = [1, 11, 9, 1, 11]
    subp.check_call(['./generate_tip',
                    'default.toml',
                    str(goal[0]), 
                    str(goal[1]), 
                    str(goal[2]), 
                    str(goal[3]), 
                    str(goal[4])])
    s_goal = np.loadtxt('./interaction_test.txt')

    s = e_v[0,:]
    i = 0
    start.extend(s)
    current_array = [] 
    while True:
        i = i + 1
        print(start[:5])
        #print(start)
        subp.check_call(['./generate_tip',
                    'default.toml',
                    str(start[0]), 
                    str(start[1]), 
                    str(start[2]), 
                    str(start[3]), 
                    str(start[4])])
        s = np.loadtxt('./interaction_test.txt')
        start[-3:] = s - s_goal
        current_array.append(start[:5])
        delta = trained_policy.predict(np.array(start[:]).reshape(1,len(start[:])))[0]
        start[:5] = start[:5] + delta
        for k_1 in range(5):
            if(start[k_1] < 0):
                start[k_1] = np.random.uniform(0,0.5)
        
        if(i>30):
            saved_array = np.array(current_array)
            for j in range(saved_array.shape[0]):
                for k in range(saved_array.shape[1]):
                    saved_array[j][k] = "{:.4f}".format(saved_array[j][k])
            #pd.DataFrame(saved_array).to_csv("./csv/learning_test_2.csv")
            break

    
    
    # subp.check_call(['./create_plan', '--default-problem', 'default.toml'])
    # subp.check_call(['./create_plan',
    #                 '--planner-name', 'RRTstar',
    #                 '--problem', 'default.toml'])
    # problem_description = toml.load('default.toml')
    # problem_description['backbone_specs']['E'] = 12
    #toml.write(problem_description)

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))



