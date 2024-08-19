# BSD 3-Clause License

# Copyright (c) 2024, The University of Utah
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#! @author Brian Cho


import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.optim import Adam
import matplotlib.pyplot as plt
import numpy as np
import csv
import sys
import os
import pickle
from pointconv_util import PointConvDensitySetAbstraction as PointConv

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class InvDynamicsNetwork(nn.Module):

    def __init__(self):
        super().__init__()

        feature_dim = 3
        self.sa1 = PointConv(npoint=2048, nsample=32, in_channel=feature_dim+3, mlp=[32, 32, 64], bandwidth=0.05, group_all=False)
        self.sa2 = PointConv(npoint=1024, nsample=32, in_channel=64+3, mlp=[64, 64, 64], bandwidth=0.1, group_all=False)
        self.sa3 = PointConv(npoint=256, nsample=32, in_channel=64+3, mlp=[128, 128, 256], bandwidth=0.2, group_all=False)
        self.sa4 = PointConv(npoint=32, nsample=32, in_channel=256+3, mlp=[256, 256, 512], bandwidth=0.4, group_all=False)
        self.fc1 = nn.Linear(32*512,256)
        self.bn1 = nn.BatchNorm1d(256)
        self.fc2 = nn.Linear(256,128)
        self.bn2 = nn.BatchNorm1d(128)
        self.fc3 = nn.Linear(128,5)

    def forward(self, point_cloud):
        point_cloud = point_cloud.transpose(2,1)    
        l0_xyz = point_cloud
        l0_points = None          

        l1_xyz, l1_points = self.sa1(l0_xyz, l0_xyz)
        l2_xyz, l2_points = self.sa2(l1_xyz, l1_points)
        l3_xyz, l3_points = self.sa3(l2_xyz, l2_points)
        l4_xyz, l4_points = self.sa4(l3_xyz, l3_points)
        embedding = torch.reshape(l4_points, (-1, 32*512))
        cloud_embedding = self.fc1(embedding)
        cloud_embedding = self.bn1(cloud_embedding)
        cloud_embedding = F.relu(cloud_embedding)

        cloud_embedding = self.fc2(cloud_embedding)
        cloud_embedding = self.bn2(cloud_embedding)
        cloud_embedding = F.relu(cloud_embedding)

        cloud_embedding = self.fc3(cloud_embedding)
        
        #loss = F.mse_loss(cloud_embedding, action)

        return cloud_embedding 

    def output_context(self, point_cloud):
        point_cloud = point_cloud.transpose(2,1)    
        l0_xyz = point_cloud
        l0_points = None          

        l1_xyz, l1_points = self.sa1(l0_xyz, l0_xyz)
        l2_xyz, l2_points = self.sa2(l1_xyz, l1_points)
        l3_xyz, l3_points = self.sa3(l2_xyz, l2_points)
        l4_xyz, l4_points = self.sa4(l3_xyz, l3_points)
        embedding = torch.reshape(l4_points, (-1, 32*512))
        cloud_embedding = self.fc1(embedding)
        cloud_embedding = self.bn1(cloud_embedding)
        cloud_embedding = F.relu(cloud_embedding)

        cloud_embedding = self.fc2(cloud_embedding)
        cloud_embedding = self.bn2(cloud_embedding)
        cloud_embedding = F.relu(cloud_embedding)

        cloud_embedding = self.fc3(cloud_embedding)
        #cloud_embedding = F.relu(cloud_embedding)
        
        #loss = F.mse_loss(cloud_embedding, context)


        return cloud_embedding

def train_model():
    optimizer = Adam(inv_dyn.parameters(), lr=0.01)
    loss_criterion = nn.MSELoss() #F.mse_loss()
    steps = []
    loss_history = []
    num_train_iters = 100
    for i in range(num_train_iters):
        optimizer.zero_grad()
        pred_action = inv_dyn(demo_state)
        loss = loss_criterion(pred_action, demo_action)
        print("iteration: ", i, "loss: ", loss)
        steps.append(i+1)
        loss_history.append(loss)
        loss.backward()
        optimizer.step()

    plt.plot(steps, loss_history)
    plt.xlabel("Steps")
    plt.ylabel("Loss")
    plt.title("Training Loss (BCO)")
    plt.show()

def get_demo(method='bc'):
    with open('bc_lung_data_storage.pickle', 'rb') as f:
        data = pickle.load(f)
        state = []
        action = []
        for k in data:
            if method == 'bco':
                s_s2 = np.vstack((data[k][1], data[k][2]))
            else:
                s_s2 = data[k][1] 

            acs = data[k][0]
            state.append(s_s2)
            action.append(acs)

        state_torch = torch.from_numpy(np.array(state)).float().to(device)
        action_torch = torch.from_numpy(np.array(action)).float().to(device)

    return state_torch, action_torch

def csv_writer(policy):
    policy = policy.cpu().detach().numpy() #np.array(policy)
    init_state = np.array([0.1, 0.1, 0.1, 3.14, 0.06]) 
    output_file = 'bc_trained_lung_solution.csv'
    with open(output_file, 'w') as fout:
        writer = csv.writer(fout, lineterminator='\n')
        
        header = ['i']
        header.extend(f'tau_{j+1}' for j in range(3))
        header.append('theta')
        header.append('s_start')
        writer.writerow(header)
        
        init_row = [1]
        init_row.extend(init_state)
        writer.writerow(init_row)

        for i in range(len(policy)):
            row = [i+2]
            next_state = init_state + policy[i]
            row.extend(next_state)

            init_state = next_state
            writer.writerow(row)


if __name__ == "__main__":
    demo_state, demo_action = get_demo()
    inv_dyn = InvDynamicsNetwork()
    inv_dyn.to(device)

    initial_output = inv_dyn(demo_state)
    train_model()

    # Validation
    output = inv_dyn(demo_state)
    print("Initial network output (before training) : ", initial_output)
    print("Network output: ", output)
    print("Actual action: ", demo_action)
    
    csv_writer(output)
