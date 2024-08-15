#!/usr/bin/env python3

import gc
from argparse import RawTextHelpFormatter
from datetime import timedelta
import argparse
import csv
import itertools
import pickle
import sys
import tempfile
import time

import matplotlib
matplotlib.use('Agg')
from mpl_toolkits import mplot3d
from torch.utils.data import DataLoader, TensorDataset
from pointconv_util import PointConvDensitySetAbstraction as PointConv
from pointnet2_utils import PointNetSetAbstractionMsg, PointNetSetAbstraction
from transformers import GPT2Config, GPT2Model
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
torch.cuda.empty_cache()

from emd import earth_mover_distance
import cpptendon as T

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
use_cuda = True

# Define the GPT model for shape prediction
class ShapePredictionModel(nn.Module):
    def __init__(self, state_dim, pc_dim, gpt_config, max_traj_len=50):
        super(ShapePredictionModel, self).__init__()
        self.state_dim = state_dim
        self.pc_dim = pc_dim
        self.hidden_dim = gpt_config.n_embd
        self.chamfer_loss = ChamferLoss()

        # Point cloud encoder (using PointConv)
        self.sa1 = PointConv(npoint=512, nsample=32, in_channel=3, mlp=[64, 64, 128], bandwidth=0.1, group_all=False)
        self.sa2 = PointConv(npoint=128, nsample=64, in_channel=128+3, mlp=[128, 128, 256], bandwidth=0.2, group_all=False)
        self.sa3 = PointConv(npoint=1, nsample=None, in_channel=256+3, mlp=[256, 512, 1024], bandwidth=0.4, group_all=True)
        self.embed_pc = nn.Linear(1024, self.hidden_dim)

        # GPT model
        self.gpt = GPT2Model(gpt_config)

        self.embed_state = nn.Linear(self.state_dim, self.hidden_dim)
        self.embed_ln = nn.LayerNorm(self.hidden_dim)

        self.predict_pc = nn.Linear(self.hidden_dim, pc_dim)

    def forward(self, states, pc, timesteps, attention_mask=None):
        batch_size = states.shape[0]
        trajectory_length = states.shape[1]

        states = states.view(batch_size, trajectory_length, -1)

        if attention_mask is None:
            attention_mask = torch.ones((batch_size, trajectory_length), dtype=torch.long)

        # Point cloud embeddings
        pc_transpose = pc.view(batch_size * trajectory_length, 3, -1)
        l1_xyz, l1_points = self.sa1(pc_transpose, None)
        l2_xyz, l2_points = self.sa2(l1_xyz, l1_points)
        l3_xyz, l3_points = self.sa3(l2_xyz, l2_points)
        x = l3_points.view(-1, 1024)
        pc_embeddings = self.embed_pc(x)

        pc_embeddings = pc_embeddings.view(batch_size, trajectory_length, -1)

        # State embeddings
        state_embeddings = self.embed_state(states)

        # Time embeddings (using Decision Transformer's time embedding)
        time_embeddings = self.gpt.get_input_embeddings()(timesteps)

        state_embeddings = state_embeddings + time_embeddings
        pc_embeddings = pc_embeddings + time_embeddings

        stacked_inputs = torch.stack((state_embeddings, pc_embeddings), dim=1).permute(0, 2, 1, 3).reshape(batch_size, 2 * trajectory_length, self.hidden_dim)
        stacked_inputs = self.embed_ln(stacked_inputs)

        stacked_attention_mask = torch.stack((attention_mask, attention_mask), dim=1).permute(0, 2, 1).reshape(batch_size, 2 * trajectory_length).to(device)

        transformers_outputs = self.gpt(
            inputs_embeds=stacked_inputs,
            attention_mask=stacked_attention_mask
        )
        x = transformers_outputs['last_hidden_state']
        x = x.reshape(batch_size, trajectory_length, 2, self.hidden_dim).permute(0, 2, 1, 3)
        pc_preds = self.predict_pc(x[:, 0])

        return pc_preds

    def get_chamfer_loss(self, inp, out):
        return self.chamfer_loss(inp, out)

class ChamferLoss(nn.Module):
    def __init__(self):
        super(ChamferLoss, self).__init__()
        self.use_cuda = torch.cuda.is_available()

    def batch_pairwise_dist(self, x, y):
        bs, num_points_x, points_dim = x.size()
        _, num_points_y, _ = y.size()
        xx = torch.bmm(x, x.transpose(2, 1))
        yy = torch.bmm(y, y.transpose(2, 1))
        zz = torch.bmm(x, y.transpose(2, 1))
        diag_ind_x = torch.arange(0, num_points_x)
        diag_ind_y = torch.arange(0, num_points_y)
        if x.get_device() != -1:
            diag_ind_x = diag_ind_x.cuda(x.get_device())
            diag_ind_y = diag_ind_y.cuda(x.get_device())
        rx = xx[:, diag_ind_x, diag_ind_x].unsqueeze(1).expand_as(zz.transpose(2, 1))
        ry = yy[:, diag_ind_y, diag_ind_y].unsqueeze(1).expand_as(zz)
        P = (rx.transpose(2, 1) + ry - 2 * zz)
        return P

    def forward(self, preds, gts):
        P = self.batch_pairwise_dist(gts, preds)
        mins, _ = torch.min(P, 1)
        loss_1 = torch.sum(mins)
        mins, _ = torch.min(P, 2)
        loss_2 = torch.sum(mins)
        return loss_1 + loss_2

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
    This code trains a GPT model with the physical robot point clouds. It learns the sequential forward kinematics of the tendon robot in trajectories, i.e., the network takes as input the sequence of the tensions of the tendons and corresponding point cloud shapes, and outputs the estimated shape (point clouds) of the entire robot.
    Training backbone points, e.g., python3 sequential_shape_gpt.py data/pbf_traj_bp.pickle data/pbf_traj_bp_3.pth data/pbf_traj_bp_idx.pickle
    Training full points, e.g., python3 sequential_shape_gpt.py data/pbf_traj_gpt.pickle data/pbf_traj_gpt.pth data/pbf_traj_bp_idx.pickle 
    file - data/pbf_traj_bp_3.pth -- current version of weights
         - data/pbf_traj_bp_4.pth -- Weights currently tested on
    '''
    parser.formatter_class = RawTextHelpFormatter
    parser.add_argument('file', default='actual_pc_data.pickle', help='type a file name you want to read data from')
    parser.add_argument('weight', default='data/actual_decoder_model_weights_3.pth', help='type a file name you want to store the learned model in')
    parser.add_argument('testset', default='data/test_idx_hysteresis.pickle', help='type a file name you want to store indices of the test set in')
    parser.add_argument('--num-epochs', type=int, default=300, help='the number of epochs you want the training procedure to iterate through')
    parser.add_argument('--percent-training-size', type=int, default=80, help='the ratio of the training dataset, e.g., 70%% of the training size will yield 30%% of the test size')
    parser.add_argument('--num-data', type=int, default=512, help='The number of point clouds you want the network model to output')
    return parser

def get_loss(model, pred, label, method='EC2'):
    '''
    ##input
    model: Network model
    pred: network output
    label: Ground Truth
    method: MSE = Mean Squared Error, 
                  EC= Earth Mover + Chamfer distance,
                      data order = p1x, p2x, p3x, ..., p1z, p2z, p3z,...
                  EC2= EMD + Chamfer 
                      data order = p1x, p1y, p1z, ...
    ##output: Loss 
    '''
    if method=='MSE':
        loss_criterion = nn.MSELoss() 
        loss = loss_criterion(pred, label)
    elif method == 'EC':
        d = earth_mover_distance(label.reshape(label.shape[0], 3, -1), pred.reshape(pred.shape[0], 3, -1), transpose=True)
        loss_1 = (d[0] / 2 + d[1

