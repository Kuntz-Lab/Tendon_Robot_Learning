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

    state = []
    action = []
    expert = []
    end_pose = []
    if True:
        start = [   9.66156,    8.75575,    8.77728, 0.00358065,         20]
        goal = [-0.0673027,       -0.1,  0.0635002]
        for i in range(50):
            #print(points)
            for j in range(len(start)):
                start = [   9.66156,    8.75575,    8.77728, 0.00358065,         20]
                if(start[j] > 20 - i*0.1):
                    start[j] = 20 - i*0.1
            subp.check_call(['./interactive_control',
                        '--robot', 'sphere_around.toml',
                        '--start0', str(start[0]), 
                        '--start1', str(start[1]), 
                        '--start2', str(start[2]), 
                        '--start3', str(start[3]), 
                        '--start4', str(start[4]), 
                        '--des0', str(goal[0]), 
                        '--des1', str(goal[1]), 
                        '--des2', str(goal[2])])
            start = np.loadtxt('./interaction_ik.txt')
    



    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))




