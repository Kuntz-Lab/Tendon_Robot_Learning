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
    model.eval()
    # pick last K steps
    # use each output as next input autoregressively
    pc_dim = 1536
    state_dim = 4

    K = 50
    L = 0.2
    test_dataset = np.array(data).reshape(-1, len(data), 1544) #1544 #3080 #254 
    test_dataset = torch.from_numpy(test_dataset).float().to(device)
    token, label = test_dataset[:,:,:dim], test_dataset[:,:,8:]
    cdist_list = []
    with torch.no_grad():
        #output = model(token, label) 
        chamferDist = ChamferDistance()
        pcs = torch.zeros((0, pc_dim), device=device, dtype=torch.float32)
        for i in range(len(test_dataset[0])):
            print("current iter: ", i)
            states = test_dataset[:,:i+1,:4]                

            # add padding
            pcs = torch.cat([pcs, torch.zeros((1, pc_dim), device=device)], dim=0)
            gt = np.array(test_dataset[:,i,8:][0].tolist()).reshape(-1,3)
            gt = outlier_filter(gt)

            states_inloop = states.reshape(1, -1, state_dim)[:,-K:]
            pcs_inloop = pcs.reshape(1, -1, pc_dim)[:,-K:]

            # attention mask
            attention_mask = torch.cat([torch.zeros(K-states_inloop.shape[1]), torch.ones(states_inloop.shape[1])])
            attention_mask = attention_mask.to(dtype=torch.long, device=states.device).reshape(1,-1)

            states_inloop = torch.cat(
                [torch.zeros((states_inloop.shape[0], K-states_inloop.shape[1], state_dim), device=states_inloop.device), states_inloop],
                dim=1).to(dtype=torch.float32)

            pcs_inloop = torch.cat(
                [torch.zeros((pcs_inloop.shape[0], K - pcs_inloop.shape[1], pc_dim),
                             device=pcs_inloop.device), pcs_inloop],
                dim=1).to(dtype=torch.float32) 


            pc_preds = model.forward(states_inloop, pcs_inloop)[0,-1]
            
            pcs[-1] = pc_preds

            gt_temp = gt.reshape(-1,3)
            pc_temp = pc_preds.reshape(-1,3)
            gt_torch = torch.from_numpy(gt_temp).float().to(device)
            gt_torch = gt_torch.reshape(1, gt_temp.shape[0], gt_temp.shape[1])
            #res_torch = torch.from_numpy(pc_temp).float().to(device)
            res_torch = pc_temp.reshape(1, pc_temp.shape[0], pc_temp.shape[1])

            dist = chamferDist(gt_torch, res_torch).detach().cpu().item()
            cdist_list.append(dist)

            pc_temp = pc_temp.detach().cpu().numpy()
            if i == 0 or (i+1)%20 == 0:
                test_plot(pc_temp, gt_temp)
        cdist_list = np.array(cdist_list)
        plt.show()
        
        print("Chamfer distance list: ", cdist_list)
        print("Chamfer distance Mean and Std: ", cdist_list.mean(), cdist_list.std())

def test_plot(r, g):
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

    #plt.show()

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
    parser.add_argument('-E','--num-embeds', type=int, default=96)
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    num_samples = args.num_samples
    test_idx = args.idx
    gpt_config = GPT2Config(
        n_embd= args.num_embeds, #96, #312, #228, #264,#384,#1536,#768,  #96 works best so far.
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
