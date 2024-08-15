#!/usr/bin/env python3

from datetime import timedelta
import argparse
import csv
import itertools
import pickle
import sys
import tempfile
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import trimesh
torch.cuda.empty_cache()

from emd import earth_mover_distance
import cpptendon as T
from test_config import config_compare

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class DeepDecoder(nn.Module):

    def __init__(self, N, hys=False):
        super().__init__()
        self.N = N
        self.chamfer_loss = ChamferLoss()

        self.input_size = 8  if hys else 4

        self.fc1 = nn.Linear(self.input_size,128)
        self.bn1 = nn.GroupNorm(1,128)
        self.fc2 = nn.Linear(128,256)
        self.bn2 = nn.GroupNorm(1,256)
        self.fc3 = nn.Linear(256,512)
        self.bn3 = nn.GroupNorm(1,512)
        self.fc4 = nn.Linear(512,1024)
        self.bn4= nn.GroupNorm(1,1024)
        self.fc5 = nn.Linear(1024,3*self.N)

        #self.fc1 = nn.Linear(4,128)
        #self.bn1 = nn.GroupNorm(1,128)
        #self.fc2 = nn.Linear(128,512)
        #self.bn2 = nn.GroupNorm(1,512)
        #self.fc3 = nn.Linear(512,1024)
        #self.bn3 = nn.GroupNorm(1,1024)
        #self.fc4 = nn.Linear(1024,2048)
        #self.bn4 = nn.GroupNorm(1,2048)
        #self.fc5 = nn.Linear(2048,3*self.N)

    def forward(self, x):
        x = self.fc1(x)
        x = self.bn1(x)
        x = F.relu(x)

        x = self.fc2(x)
        x = self.bn2(x)
        x = F.relu(x)

        x = self.fc3(x)
        x = self.bn3(x)
        x = F.relu(x)
        
        x = self.fc4(x)
        x = self.bn4(x)
        x = F.relu(x)
        
        x = self.fc5(x)
        return x 

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
    This code trains a deep decoder network model with the physical robot point clouds. It learns the forward kinematics of the tendon robot, i.e., the network takes as input the tensions of the tendons and outputs the corresponding shape (point clouds) of the entire robot.
    '''
    parser.add_argument('file', default='actual_pc_data.pickle', help='type a file name you want to read data from')
    parser.add_argument('weight', default='data/actual_decoder_model_weights_3.pth', help='type a file name you want to store the learned model in')
    parser.add_argument('testset', default='data/test_idx_hysteresis.pickle', help='type a file name you want to store indices of the test set in')
    parser.add_argument('--num-epochs', type=int, default=200, help='the number of epochs you want the training procedure to iterate through')
    parser.add_argument('--percent-training-size', type=int, default=70, help='the ratio of the training dataset, e.g., 70%% of the training size will yield 30%% of the test size')
    parser.add_argument('--num-data', type=int, default=512, help='The number of point clouds you want the network model to output')
    parser.add_argument('--hys', type=int, default=0, help='If Hysteresis, then the network input size should be 8 (current tensions + previous tensions), otherwise 4 (current tensions only).')
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
        d = earth_mover_distance(label.reshape(label.shape[0], -1, 3), pred.reshape(pred.shape[0], -1, 3), transpose=False) 
        loss_1 = torch.sum(d)     #(d[0] / 2 + d[1] *2 + d[2] /3) 
        loss_2 = model.get_chamfer_loss(label.reshape(label.shape[0], -1, 3), pred.reshape(pred.shape[0], -1, 3))
        loss = loss_1 + loss_2

    elif method=='E':
        d = earth_mover_distance(label.reshape(label.shape[0], -1, 3), pred.reshape(pred.shape[0], -1, 3), transpose=False) 
        loss = torch.sum(d)     #(d[0] / 2 + d[1] *2 + d[2] /3) 

    elif method=='MC':
        loss_criterion = nn.MSELoss() 
        loss_1 = model.get_chamfer_loss(label.reshape(label.shape[0], -1, 3), pred.reshape(pred.shape[0], -1, 3))
        loss_2 = loss_criterion(pred, label)
        loss = loss_1 + loss_2
    elif method=='C':
        loss = model.get_chamfer_loss(label.reshape(label.shape[0], -1, 3), pred.reshape(pred.shape[0], -1, 3))
    return loss 

def train_model(network, optimizer, train_loader, epoch, hys=False, lossfunc='EC2'):
    network.train()

    input_size = 8 if hys else 4
    train_loss = 0
    num_batch = 0
    steps = []
    loss_history = []
    for i, sample in enumerate(train_loader):
        config, label = sample[:,:input_size], sample[:,8:]
        num_batch += 1
        optimizer.zero_grad()
        pred = network(config)

        loss = get_loss(network, pred, label, method=lossfunc)
        steps.append(i+1)
        loss_history.append(loss)
        loss.backward()
        train_loss += loss.item()
        optimizer.step()
        #if i % 100 == 0:
        #    print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
        #        epoch+1, i * len(sample), len(train_loader.dataset),
        #        100. * i / len(train_loader), loss.item()))
    print('Current loss function: ', lossfunc)
    print('====> Epoch: {} Average Training Loss: {:.6f}'.format(
              epoch+1, train_loss/num_batch)) 


def test_model(model, test_loader, hys=False, lossfunc='EC2'):
    model.eval()
    test_loss = 0
    input_size = 8 if hys else 4
    correct = 0
    num_batch = 0  
    with torch.no_grad():
        for batch_idx, sample in enumerate(test_loader):
            num_batch += 1

            config, label = sample[:,:input_size], sample[:,8:]
            output = model(config)
            test_loss += get_loss(model, output, label, method=lossfunc)
    test_loss /= num_batch

    print('Test set: Average Loss: {:.10f}\n'.format(test_loss))

def get_data(robot, 
             file_name='test_tendon_data.pickle', 
             num_test_samples=30,
             model_path='data/decoder_model_weights.pth',
             test_file='data/fixed_home_30_test_cases.pickle',
             training_size=70,
             N=512,
             file_type = 'pickle'):
    start = time.time()

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
            ## if data structure is a Python dictionary
            #data_dict = pickle.load(fin)
            #commands = list(data_dict.values()) 
        print("A list size from pickle file: ", len(commands), len(commands[0]))

        ## Normalize
        #commands = np.array(commands)
        #for i in range(len(robot.tendons)):
        #    commands[:,i] *= 20 / robot.tendons[i].max_tension

        with open(test_file, 'rb') as test_case:
            test_idx = pickle.load(test_case)

        #with open('data/fixed_home_30_test_cases.pickle', 'rb') as test_case:
        #    test_idx = pickle.load(test_case)

        idx = np.array([x for x in range(len(commands)) if x not in test_idx])
        num_train = int(training_size * len(idx) / 100)
        train_idx = np.random.choice(idx, size=num_train, replace=False)
        train_dataset = list(commands[i] for i in train_idx)

        val_idx = np.array([x for x in idx if x not in train_idx])
        val_dataset = list(commands[i] for i in val_idx)

        #num_train = int(training_size * len(commands) / 100)
        #train_idx = np.random.choice(len(commands), size=num_train, replace=False)
        #train_dataset = list(commands[i] for i in train_idx)

        #idx = np.array([x for x in range(len(commands)) if x not in train_idx])
        #test_idx = np.random.choice(idx, size=num_test_samples, replace=False) 
        #val_idx = np.array([y for y in idx if y not in test_idx])

        #val_dataset = list(commands[i] for i in val_idx)

    ## Torchify dataset
    train_dataset = torch.from_numpy(np.array(train_dataset)).float().to(device)
    val_dataset = torch.from_numpy(np.array(val_dataset)).float().to(device)

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

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    robot = T.tendon.TendonRobot.from_toml('../data/sim_robot_limits.toml')
    model_path='data/decoder_model_weights_with_disks_2.pth'
    num_samples=10
    train, test, test_idx = get_data( robot,
                            file_name=args.file,
                            num_test_samples=num_samples,
                            model_path=model_path,
                            test_file=args.testset,
                            training_size=args.percent_training_size,
                            N=args.num_data,
                          )

    #test_idx_storage = args.testset
    #with open(test_idx_storage, 'wb') as ff:
    #    pickle.dump(list(test_idx), ff)
    train_loader = torch.utils.data.DataLoader(train, batch_size=16, shuffle=True)
    test_loader = torch.utils.data.DataLoader(test, batch_size=16, shuffle=True)
    
    # Network model
    dnn = DeepDecoder(args.num_data, args.hys)

    # Transfer learning from a pretrained model with simulation dataset
    #dnn.load_state_dict(torch.load(model_path))
    dnn.to(device)
    print("====Initializing Model Weights...====\n")
    dnn.apply(weights_init)
    #print("Model initilized..\n")

    optimizer = optim.Adam(dnn.parameters(), lr=0.01)
    scheduler = optim.lr_scheduler.StepLR(optimizer, 50, gamma=0.1)
    weight_storage = 'data/weight_Dec.pth'
    lossfunc = 'EC2'
    for epoch in range(1+args.num_epochs):
        train_model(dnn, optimizer, train_loader, epoch, hys=args.hys, lossfunc=lossfunc)
        scheduler.step()
        test_model(dnn, test_loader, hys=args.hys, lossfunc='C')

        if epoch %10 == 0:
            torch.save(dnn.state_dict(), args.weight)
    
    config_compare(
                   num_samples=num_samples,
                   model_path=args.weight,
                   filename=args.file,
                   idx = args.testset,
                   hys=args.hys
                   )
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
