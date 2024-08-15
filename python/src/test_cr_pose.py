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
# seed(1000)
# set_random_seed(1000)

def populate_parser(parser = None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = "This is what I do!"
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    saved_model_path = "./saved_models_1/haptic_sphere_20_pose_all_to_cr_128_128_1615442341/1615442341"
    trained_policy = tf.contrib.saved_model.load_keras_model(saved_model_path)

    # Dataset2 = pd.read_csv('./chained.csv',sep=r'\s*,\s*',engine = 'python',na_values = '?',index_col=0)
    # Dataset2.dropna()
    # #print(Dataset2)
    # data_with_panda = pd.get_dummies(Dataset2, drop_first = True)
    # Init_XY = data_with_panda.values
    # #print(Init_XY)
    # X_and_Y = data_with_panda.values
    
    e_v = np.loadtxt('./csv/haptic_sphere_data/haptic_sphere_20_4_test.txt')
    # problem_description['environment']['spheres'][0]['sphere']['center'][0] = np.random.uniform(-0.04, 0.04) #np.random.uniform(-0.04, 0.04)
    # problem_description['environment']['spheres'][0]['sphere']['center'][1] = -0.10 #np.random.uniform(-0.10) #np.random.uniform(-0.18, -0.10)
    # problem_description['environment']['spheres'][0]['sphere']['center'][2] = np.random.uniform(0.05 , 0.09)

    

    # initial state:   [  0  11 5.5 7.6 6.6]
    #   initial tip:     [0.0387463 -0.130919 0.0769672]
    #   desired tip:     [ 0.02 -0.14  0.05]
    #   reached tip:     [0.0233899 -0.134733 0.0632327]
    #   solution:        [      0 10.7584 7.17972  8.2993 7.60464]



    current_array = []
    end_list = []
    num_df = []
    
    for k in range(20):#int(Init_XY.shape[0]/4)):
        #start =  X_and_Y[k*4,:] #[ 0, 10.7584, 7.17972,  8.2993, 7.60464] #[0,11,5.5,7.6,6.6]#X_and_Y[18,:]+0.1 #[      0, 9.90974, 6.10127, 7.92678, 5.37951]#[0, 8.40811, 9.1432, 6.74058, 8.7633]#X_and_Y[8,:]
        x_center = 0
        z_center = 0.07
        start_x = np.random.uniform(x_center - 0.04, x_center + 0.04)  # sphere with 0.055, circle with 0.05
        start_y = -0.10   # sphere with -0.10 while circle with -0.14
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
        start = [0, 0, 0.2]
        center = [start_x, start_y, start_z]
        #center = [e_v[k*4, 0], e_v[k*4, 1], e_v[k*4, 2]]
        #start = [center[0] + 0.055, center[1], center[2]]
        points = [[center[0] + 0.055, center[1], center[2]], [center[0] + 0.036, center[1], center[2] + 0.036], [center[0], center[1], center[2]+0.055], 
        [center[0] - 0.036, center[1], center[2] + 0.036]]
        print('center')
        print(center)
        print(points)
        start_list = []
        while True:
            i = i + 1
            print(start)
            input_list = [start[0], start[1], start[2], center[0], center[1], center[2]]
            
            delta = trained_policy.predict(np.array(input_list[:]).reshape(1,len(input_list[:])))[0]
            start = [start[0] + delta[0], start[1] + delta[1], start[2] + delta[2]] 
            start_list.append(start)
            if(i>4):
                num_df.append(similaritymeasures.frechet_dist(np.array(points), np.array(start_list)).tolist())
                break


    
    
    # subp.check_call(['./create_plan', '--default-problem', 'default.toml'])
    # subp.check_call(['./create_plan',
    #                 '--planner-name', 'RRTstar',
    #                 '--problem', 'default.toml'])
    # problem_description = toml.load('default.toml')
    # problem_description['backbone_specs']['E'] = 12
    #toml.write(problem_description)

    # pd.DataFrame(current_array).to_csv("./csv/end_tensions_haptic_sphere_15_4_test_64_64_64_pose.csv")
    # np.savetxt('./csv/end_list_haptic_sphere_15_4_test_64_64_64_pose.txt', end_list)
    num_df = np.array(num_df)
    print(num_df)
    print(num_df.mean())
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))




