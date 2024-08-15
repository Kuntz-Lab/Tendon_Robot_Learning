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
import os
# from numpy.random import seed
# from tensorflow import set_random_seed
# seed(1000)
# set_random_seed(1000)


def populate_parser(parser = None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.add_argument('problem',
                        help='''
                            sphere problem to view
                            ''')
    parser.add_argument('plan',
                        help='''
                            curve on the sphere
                            ''')
    parser.description = "This is what I do!"
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    
    problem = args.problem
    plan = args.plan
    dir_name = '/home/yhuang/tendon_experiments/build'
    Dataset2 = pd.read_csv(os.path.join(dir_name, plan),sep=r'\s*,\s*',engine = 'python',na_values = '?',index_col=0)
    Dataset2.dropna()
    #print(Dataset2)
    data_with_panda = pd.get_dummies(Dataset2, drop_first = True)
    Init_XY = data_with_panda.values
    print(Init_XY)

    subp.check_call(['./generate_tip',
                        'sphere_around.toml',
                        str(Init_XY[0][0]), 
                        str(Init_XY[0][1]), 
                        str(Init_XY[0][2]), 
                        str(Init_XY[0][3]), 
                        str(Init_XY[0][4])])
    s_end = np.loadtxt('./interaction_test.txt')
    problem_description = toml.load(problem)
    print(s_end)
    problem_description['environment']['spheres'][0]['sphere']['center'][0] = (float)(s_end[0] - 0.055) #np.random.uniform(-0.04, 0.04) #np.random.uniform(-0.04, 0.04)
    problem_description['environment']['spheres'][0]['sphere']['center'][1] = (float)(s_end[1]) #np.random.uniform(-0.10) #np.random.uniform(-0.18, -0.10)
    problem_description['environment']['spheres'][0]['sphere']['center'][2] = (float)(s_end[2])
    print(problem_description['environment']['spheres'][0]['sphere']['center'])
    output_path = os.path.join(dir_name, problem)
    with open(output_path, mode="w") as fh:
        toml.dump(problem_description, fh)
    subp.check_call(['./view_plan',
                        problem,
                        plan])
                
                
                
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))




