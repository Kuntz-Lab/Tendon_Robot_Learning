#!/usr/bin/env python3

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
from transformers import GPT2Model, GPT2Config, GPT2LMHeadModel
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
torch.cuda.empty_cache()

from emd import earth_mover_distance
import cpptendon as T

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Define the GPT model for shape prediction
class ShapePredictionModel(nn.Module):
    def __init__(self, input_dim, output_dim, gpt_config):
        super(ShapePredictionModel, self).__init__()
        self.hidden_dim = gpt_config.n_embd
        #self.hidden_dim_2 = 512 
        self.chamfer_loss = ChamferLoss()
        
        self.gpt = GPT2Model(gpt_config)
        #self.gpt = GPT2LMHeadModel(gpt_config)
        self.fc = nn.Linear(self.hidden_dim, output_dim)
        #self.fc = nn.Linear(self.hidden_dim, self.hidden_dim_2)
        #self.fc_2 = nn.Linear(self.hidden_dim_2, output_dim)
        self.embed = nn.Linear(input_dim, self.hidden_dim)
        
    
    #def forward(self, joint_states, shapes):
    def forward(self, states):
        trajectory_length = states.shape[1]
        batch_size = states.shape[0]
        
        # Reshape the inputs for GPT model
        #joint_states = joint_states.view(batch_size * trajectory_length, -1)
        #shapes = shapes.view(batch_size * trajectory_length, -1)
        states = states.view(batch_size, trajectory_length, -1)
        
        # Encode the inputs
        #input_encoding = torch.cat((joint_states, shapes), dim=1).reshape(batch_size, trajectory_length, -1)
        # Pass the input through GPT model
        input_encoding = self.embed(states)
        gpt_output = self.gpt(inputs_embeds=input_encoding)
        #print("gpt output size: ", gpt_output.logits.shape)
        
        # Feed the GPT output to a linear layer for shape prediction
        output = self.fc(gpt_output['last_hidden_state'])
        #output = self.fc(gpt_output.logits)
        #output = self.fc_2(output)
        
        # Reshape the output back to the trajectory shape
        #output = output.view(batch_size, trajectory_length, -1)
        
        return output

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
        loss_1 = (d[0] / 2 + d[1] *2 + d[2] /3) * 20 
        loss_2 = model.get_chamfer_loss(label.reshape(label.shape[0], 3, -1).permute(0,2,1), pred.reshape(pred.shape[0], 3, -1).permute(0,2,1))
        loss = loss_1 + loss_2

    elif method=='EC2':
        d = earth_mover_distance(label.reshape(label.shape[0]*label.shape[1], -1, 3), pred.reshape(pred.shape[0]*pred.shape[1], -1, 3), transpose=False) 
        loss_1 = torch.sum(d) #(d[0] / 2 + d[1] *2 + d[2] /3) 
        loss_2 = model.get_chamfer_loss(label.reshape(label.shape[0]*label.shape[1], -1, 3), pred.reshape(pred.shape[0]*pred.shape[1], -1, 3))
        loss = loss_1 + loss_2

    elif method=='MC':
        loss_criterion = nn.MSELoss() 
        loss_1 = model.get_chamfer_loss(label.reshape(label.shape[0], -1, 3), pred.reshape(pred.shape[0], -1, 3))
        loss_2 = loss_criterion(pred, label)
        loss = loss_1 + loss_2
    elif method=='C':
        loss = model.get_chamfer_loss(label.reshape(label.shape[0] * label.shape[1],-1, 3), pred.reshape(pred.shape[0] * pred.shape[1], -1, 3))
    return loss 

def train_model(network, optimizer, train_loader, epoch, num_epochs, input_size ,lossfunc='EC2'):
    network.train()

    train_loss = 0
    steps = []
    for i, sample in enumerate(train_loader):
        token, label = sample[:,:,:input_size], sample[:,:,input_size:]
        #print("Testing shapes...: ", token.shape, label.shape, label.shape[1])
        optimizer.zero_grad()
        pred = network(token)

        loss = get_loss(network, pred, label, method=lossfunc)
        loss /= label.shape[1]
        steps.append(i+1)
        loss.backward()
        train_loss += loss.item()
        optimizer.step()
    print(f"Epoch {epoch + 1}/{num_epochs}, Loss: {train_loss / len(train_loader):.4f}")


def test_model(model, test_loader, epoch, robot, input_size, lossfunc='EC2', enable_pbf=False):
    model.eval()
    L=0.2
    test_loss = 0
    num_batch = 0  
    with torch.no_grad():
        for batch_idx, sample in enumerate(test_loader):
            num_batch += 1

            token, label = sample[:,:,:input_size], sample[:,:,input_size:]
            output = model(token)
            test_loss += get_loss(model, output, label, method=lossfunc).item() / label.shape[1]
    test_loss /= num_batch

    test_idx = np.random.randint(len(label))

    print('Test set: Average Loss: {:.10f}\n'.format(test_loss))
    
    if epoch % 30 == 0:
        for i in range(len(token[-1])):
            if enable_pbf:
                if not i:
                    ground_truth = test_fk(label[test_idx][i].tolist(), L, None)
                    result = test_fk(output[test_idx][i].tolist(), L, ground_truth)
                else:
                    ground_truth = np.vstack((ground_truth , test_fk(label[test_idx][i].tolist(), L, None)))
                    result = np.vstack((result , test_fk(output[test_idx][i].tolist(), L, ground_truth)))
            else:
                if not i:
                    ground_truth = np.array(label[test_idx][i].tolist()).reshape(-1,3)
                    result = np.array(output[test_idx][i].tolist()).reshape(-1,3)
                else:
                    ground_truth = np.vstack((ground_truth , np.array(label[test_idx][i].tolist()).reshape(-1,3)))
                    result = np.vstack((result , np.array(output[test_idx][i].tolist()).reshape(-1,3) ))
        test_plot(result, ground_truth)

def test_plot(output, ground):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    
    ax.scatter3D(output[:,0], output[:,1], output[:,2], c='b', marker='X',s=3)
    ax.scatter3D(ground[:,0], ground[:,1], ground[:,2], c='r', marker='X',s=3)

    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.21)
    ax.set_xticks((-0.2, 0.0, 0.2))
    ax.set_yticks((-0.2, 0.0, 0.2))
    ax.set_zticks((0, 0.2))

    for ii in range(0, 360, 36):
        ax.view_init(elev=10, azim=ii)
        fig.savefig(f'../data/gpt_test_{ii+1}.jpg')
    plt.show()

def get_data(robot, 
             file_name='test_tendon_data.pickle', 
             num_test_samples=30,
             test_file='data/fixed_home_30_test_cases.pickle',
             training_size=70,
             N=512,
             traj_length=5,
             input_dim=1540,
             file_type = 'pickle'):
    start = time.time()
    #input_configs = torch.randn(batch_size, trajectory_length, 1504)

    if file_type == 'csv':
        with open(file_name, "r") as fin:
            reader = csv.DictReader(fin)
            header = reader.fieldnames
            commands = list(reader)
        end = time.time()
        print("Time taken to read the file: ", str(timedelta(seconds=int(end-start))))
        print("\n")
        
        ## Generate training dataset
        num_train = int(training_size * len(commands) / 100)
        train_idx = np.random.choice(len(commands), size=num_train, replace=False)
        train_samples = list(commands[i] for i in train_idx)
        test_idx = None
        train_dataset = []
        for i in range(num_train):
            pc = [float(train_samples[i][f'p{j}{c}']) for c in 'xyz' for j in range(1, N+1)]  
            tau = [float(train_samples[i][f't{j+1}']) for j in range(len(robot.tendons))]
            train_dataset.append(tau + pc) 
        #+ [f'p{i}{c}' for c in 'xyz' for i in range(1, N+1)])

        ## Geneate test dataset
        val_idx = np.array([x for x in range(len(commands)) if x not in train_idx])
        num_test = len(val_idx)
        val_samples = list(commands[i] for i in val_idx)

        val_dataset = []
        for i in range(num_test):
            pc = [float(val_samples[i][f'p{j}{c}']) for c in 'xyz' for j in range(1, N+1)]  
            tau = [float(val_samples[i][f't{j+1}']) for j in range(len(robot.tendons))]
            val_dataset.append(tau + pc) 
            
    else:
        with open(file_name, 'rb') as fin:
            ## if data structure is a Python list
            commands = pickle.load(fin)

        end = time.time()
        print("A list size from pickle file: ", len(commands), len(commands[0]))

        with open(test_file, 'rb') as test_case:
            test_idx = pickle.load(test_case)

        #idx = np.array([x for x in range(len(commands)) if x not in test_idx])
        #num_train = 8000
        idx = np.arange(0, len(commands))
        num_train = int(training_size * len(idx) / 100)
        train_idx = np.arange(0, num_train)
        #train_idx = np.random.choice(idx, size=num_train, replace=False)
        train_dataset = list(commands[i] for i in train_idx)

        val_idx = np.array([x for x in idx if x not in train_idx])
        val_dataset = list(commands[i] for i in val_idx)

    ### Reshaping dataset
    #train_dataset = np.array(train_dataset)[:(len(train_dataset) - (len(train_dataset) % traj_length ) )]
    #train_dataset = train_dataset.reshape(-1, traj_length, input_dim) 

    #val_dataset = np.array(val_dataset)[:(len(val_dataset) - (len(val_dataset) % traj_length ) )]
    #val_dataset = val_dataset.reshape(-1, traj_length, input_dim) 

    train_dataset = np.array(train_dataset).reshape(-1, traj_length, input_dim) 
    val_dataset = np.array(val_dataset).reshape(-1, traj_length, input_dim) 
    ## Torchify dataset
    train_dataset = torch.from_numpy(np.array(train_dataset)).float().to(device)
    val_dataset = torch.from_numpy(np.array(val_dataset)).float().to(device)

    print("Training data size: ", train_dataset.shape)
    print("Validation data size: ", val_dataset.shape)
    print("Time taken to get data: ", str(timedelta(seconds=int(time.time()-end))))
    return train_dataset, val_dataset, test_idx

def weights_init(m):
    classname = m.__class__.__name__
    if classname.find('Conv2d') != -1:
        torch.nn.init.xavier_normal_(m.weight.data)
        torch.nn.init.constant_(m.bias.data, 0.0)
    elif classname.find('Linear') != -1:
        torch.nn.init.xavier_normal_(m.weight.data)
        torch.nn.init.constant_(m.bias.data, 0.0)

def test_fk(coef, L, d):
    s = np.linspace(0, 1, 41)
    s2 = s**2
    s3 = s2*s
    s4 = s2*s2
    s5 = s3*s2
    p1 = 1.7321 * s
    p2 = -6.7082 * s + 8.9943 * s2
    p3 = 15.8745 * s - 52.915 * s2 + 39.6863 * s3
    p4 = -30.0 * s + 180.0 * s2 - 315.0 * s3 + 168.0 * s4
    p5 = 49.7494 * s - 464.33 * s2 + 1392.98 * s3 - 1671.6 * s4 + 696.4912 * s5

    x = L * (coef[0]*p1 + coef[1]*p2 + coef[2]*p3 + coef[3]*p4 + coef[4] * p5)
    y = L * (coef[5]*p1 + coef[6]*p2 + coef[7]*p3 + coef[8]*p4 + coef[9] * p5)
    z = L * (coef[10]*p1 + coef[11]*p2 + coef[12]*p3 + coef[13]*p4 + coef[14] * p5)

    xyz = np.transpose([x,y,z])
    return xyz  

def main(arguments):
    full = True
    n_layer = 96 #96 works best so far  #312 if full else 204
    parser = populate_parser()
    args = parser.parse_args(arguments)
    #robot = T.tendon.TendonRobot.from_toml('../data/sim_robot_limits.toml')
    robot = T.tendon.TendonRobot.from_toml('../data/tendon_parameter.toml')
    num_samples=10
    gpt_config = GPT2Config(
        n_embd=n_layer, #228, #240, #264,#384,#1536,#768,  #264 works best so far.
        n_layer=12,
        n_head=12,
    )

    # Define the input and output dimensions
    if full:
        input_dim = 8 #4 + 4 + 1536  # Joint state (previous + current) + Robot shape
        output_dim = 1536 #15 if pbf #1536  # Shape vector size
    else:
        input_dim = 4 + 4 + 123 # 23 if pbf #4 + 1536  # Joint state + Robot shape
        output_dim = 123 #15 if pbf #1536  # Shape vector size

    # Create an instance of the shape prediction model
    dnn = ShapePredictionModel(input_dim, output_dim, gpt_config)

    # Example usage of the model
    trajectory_length = 50 #10 
    batch_size = 8
    learning_rate = 1e-2

    train, test, test_idx = get_data( robot,
                            file_name=args.file,
                            num_test_samples=num_samples,
                            test_file=args.testset,
                            training_size=args.percent_training_size,
                            N=args.num_data,
                            traj_length=trajectory_length,
                            input_dim=input_dim+output_dim,
                          )

    train_loader = DataLoader(train, batch_size=batch_size, shuffle=False)
    test_loader = DataLoader(test, batch_size=batch_size, shuffle=False)
    
    # Transfer learning from a pretrained model with simulation dataset
    dnn.to(device)
    print("====Initializing Model Weights...====\n")
    #dnn.apply(weights_init)

    optimizer = optim.Adam(dnn.parameters(), lr=learning_rate) #AdamW?
    scheduler = optim.lr_scheduler.StepLR(optimizer, 50, gamma=0.1)
    lossfunc = 'EC2'
    for epoch in range(1+args.num_epochs):
        train_model(dnn, optimizer, train_loader, epoch, args.num_epochs, input_dim, lossfunc=lossfunc)
        scheduler.step()
        test_model(dnn, test_loader, epoch, robot, input_dim, lossfunc='C')

        if (epoch) and (epoch % 10 == 0):
            torch.save(dnn.state_dict(), args.weight)
    
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
