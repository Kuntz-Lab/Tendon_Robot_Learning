#!/usr/bin/env python3

from argparse import RawTextHelpFormatter
from emd import earth_mover_distance
import argparse
import csv
import itertools
import pickle
import sys
import tempfile
import time
import matplotlib

import matplotlib
#matplotlib.use('Agg')
from chamferdist import ChamferDistance
from mpl_toolkits import mplot3d
from scipy.spatial import distance
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch 
import torch.nn as nn
import torch.nn.functional as F
import trimesh
from transformers import GPT2Config
from modeling_gpt2 import GPT2Model

#from learn_actual_pc import DeepDecoder
#from learn_tendon_pc import DeepDecoder
from learn_tendon_shape import DeepNeuralNets, get_data
#from sequential_shape_gpt import ShapePredictionModel as GPT
from sequential_shape_gpt_modified import ShapePredictionModel as GPT
from sequential_shape_gpt import test_fk
from simulated_data_collection import generate_point_cloud
import cpptendon as T
import sim_to_real_property as LS
from test_config import outlier_filter

np.random.seed(seed=1000) #previous: 1000
plt.rcParams.update({'font.size': 14})
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def test_model(model, data, n_sample, test_idx, dim):
    L = 0.2
    test_dataset = np.array(data).reshape(-1, n_sample, 1544) #1544 #3080 #254 
    test_dataset = torch.from_numpy(test_dataset).float().to(device)
    token, label = test_dataset[:,:,:dim], test_dataset[:,:,8:]
    cdist_list = []
    with torch.no_grad():
        output = model(token, label) 
        chamferDist = ChamferDistance()
        for i in range(n_sample):
            if not i:
                print("first state: ", token[0, i])
                ground_truth = np.array(label[test_idx][i].tolist()).reshape(-1,3)
                #ground_truth = outlier_filter(ground_truth)
                result = np.array(output[test_idx][i].tolist()).reshape(-1,3)
                gt_torch = torch.from_numpy(ground_truth).float().to(device)
                gt_torch = gt_torch.reshape(1, ground_truth.shape[0], ground_truth.shape[1])
                res_torch = torch.from_numpy(result).float().to(device)
                res_torch = res_torch.reshape(1, result.shape[0], result.shape[1])

                distance = chamferDist(gt_torch, res_torch).detach().cpu().item()
                cdist_list.append(distance)
            else:
                new_gt =np.array(label[test_idx][i].tolist()).reshape(-1,3)
                #new_gt = outlier_filter(new_gt)
                new_res =np.array(output[test_idx][i].tolist()).reshape(-1,3)  
                gt_torch = torch.from_numpy(new_gt).float().to(device)
                gt_torch = gt_torch.reshape(1, new_gt.shape[0], new_gt.shape[1])
                res_torch = torch.from_numpy(new_res).float().to(device)
                res_torch = res_torch.reshape(1, new_res.shape[0], new_res.shape[1])

                distance = chamferDist(gt_torch, res_torch).detach().cpu().item()
                cdist_list.append(distance)

                ground_truth = np.vstack((ground_truth , new_gt ))
                result = np.vstack((result , new_res))



        test_plot(result, ground_truth)
        cdist_list = np.array(cdist_list)
        
        print("Chamfer distance list: ", cdist_list)
        print("Chamfer distance Mean and Std: ", cdist_list.mean(), cdist_list.std())

def test_plot(r, g):
    r = r[:512]
    g = g[:512]

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    #for i in range(len(r)):
    #    res = np.array(r[i].tolist()).reshape(-1,3)
    #    gt = np.array(g[i].tolist()).reshape(-1,3)
    ax.scatter3D(r[:,0], r[:,1], r[:,2], c='b', marker='X',s=3)
    ax.scatter3D(g[:,0], g[:,1], g[:,2], c='r', marker='X',s=3)
    
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.21)
    ax.set_xticks((-0.2, 0.0, 0.2))
    ax.set_yticks((-0.2, 0.0, 0.2))
    ax.set_zticks((0, 0.2))

    plt.show()

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Testing the GPT model on the trajectory dataset
        How to run (using 3D backbone points):
        e.g., python3 test_gpt_config.py data/pbf_traj_bp.pth data/pbf_traj_bp.pickle --idx 10

        Weight - data/test_gpt_traj_3.pth
        pickle - data/test_gpt_traj.pickle
        test - test_idx (int), e.g., 1 
        '''
    parser.formatter_class = RawTextHelpFormatter
    parser.add_argument('weight', default='data/actual_decoder_model_weights_home_hys.pth')
    parser.add_argument('pickle', default='data/home_data_from_olivia.pickle')
    parser.add_argument('--idx', type=int, default=0, help='Test idx (int)')
    parser.add_argument('-N','--num-samples', type=int, default=50)
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    num_samples = args.num_samples
    test_idx = args.idx
    gpt_config = GPT2Config(
        n_embd= 96, #96, #312, #228, #264,#384,#1536,#768,  #96 works best so far.
        n_layer=12,
        n_head=12,
    )
    with open(args.pickle, 'rb') as fin:
        dataset = pickle.load(fin)

    # Define the input and output dimensions
    input_dim = 4 #4 + 1536  # Joint state + Robot shape
    output_dim = 1536 #15 if pbf #1536  # Shape vector size

    # Create an instance of the shape prediction model
    dnn = GPT(input_dim, output_dim, gpt_config)
    dnn.load_state_dict(torch.load(args.weight))
    dnn.to(device)
    test_model(dnn, dataset, num_samples, test_idx, input_dim)

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
