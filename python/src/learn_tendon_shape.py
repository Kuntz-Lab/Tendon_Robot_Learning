#!/usr/bin/env python3

import argparse
import csv
import os
import pickle
import sys

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

import cpptendon as T

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class DeepNeuralNets(nn.Module):

    def __init__(self):
        super().__init__()

        self.fc1 = nn.Linear(4,32)
        #self.bn1 = nn.BatchNorm1d(32)
        self.fc2 = nn.Linear(32,128)
        #self.bn2 = nn.BatchNorm1d(128)
        self.fc3 = nn.Linear(128,64)
        #self.bn3 = nn.BatchNorm1d(64)
        self.fc4 = nn.Linear(64,15)

    def forward(self, x):
        x = self.fc1(x)
        #x = self.bn1(x)
        x = F.relu(x)

        x = self.fc2(x)
        #x = self.bn2(x)
        x = F.relu(x)

        x = self.fc3(x)
        #x = self.bn3(x)
        x = F.relu(x)
        
        x = self.fc4(x)
        return x 

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'TODO'
    parser.add_argument('csv', nargs='?', default='pbf_data.csv')
    parser.add_argument('--num-epochs', type=int, default=50)
    parser.add_argument('--percent-training-size', type=int, default=70)
    return parser

def train_model(network, optimizer, train_loader, epoch):
    network.train()

    loss_criterion = nn.MSELoss() 
    train_loss = 0
    num_batch = 0
    steps = []
    loss_history = []
    for i, sample in enumerate(train_loader):
        config, label = sample[:,:4], sample[:,4:]
        num_batch += 1
        optimizer.zero_grad()
        pred = network(config)
        loss = loss_criterion(pred, label)

        steps.append(i+1)
        loss_history.append(loss)
        loss.backward()
        train_loss += loss.item()
        optimizer.step()
        if i % 1000 == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                epoch+1, i * len(sample), len(train_loader.dataset),
                100. * i / len(train_loader), loss.item()))
    print('====> Epoch: {} Average loss: {:.6f}'.format(
              epoch, train_loss/num_batch)) 


def test_model(model, test_loader):
    model.eval()
    test_loss = 0
    loss_criterion = nn.MSELoss()
    correct = 0
    num_batch = 0  
    for batch_idx, sample in enumerate(test_loader):
        num_batch += 1

        config, label = sample[:,:4], sample[:,4:]
        output = model(config)
        test_loss = loss_criterion(output, label)

    test_loss /= num_batch

    print('Test set: Average loss: {:.10f}\n'.format(test_loss))

def get_data(robot, csvfile='pbf_data.csv', training_size=70):
    
    with open(csvfile, "r") as fin:
        reader = csv.DictReader(fin)
        header = reader.fieldnames
        commands = list(reader)

    ## Generate training dataset
    num_train = int(training_size * len(commands) / 100)
    train_idx = np.random.choice(len(commands), size=num_train, replace=False)
    train_samples = list(commands[i] for i in train_idx)

    train_dataset = []
    for i in range(num_train):
        coef = [float(train_samples[i][f'coeff{j+1}']) for j in range(15)]  
        tau = [float(train_samples[i][f't{j+1}']) for j in range(len(robot.tendons))]
        train_dataset.append(tau + coef) 

    ## Geneate test dataset
    test_idx = np.array([x for x in range(len(commands)) if x not in train_idx])
    num_test = len(test_idx)
    test_samples = list(commands[i] for i in test_idx)

    test_dataset = []
    for i in range(num_test):
        coef = [float(test_samples[i][f'coeff{j+1}']) for j in range(15)]  
        tau = [float(test_samples[i][f't{j+1}']) for j in range(len(robot.tendons))]
        test_dataset.append(tau + coef) 
        
    ## Torchify dataset
    train_dataset = torch.from_numpy(np.array(train_dataset)).float().to(device)
    test_dataset = torch.from_numpy(np.array(test_dataset)).float().to(device)
    return train_dataset, test_dataset

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    robot = T.tendon.TendonRobot.from_toml('../data/default.toml')
    train, test = get_data( robot,
                            csvfile=args.csv,
                            training_size=args.percent_training_size,
                          )

    train_loader = torch.utils.data.DataLoader(train, batch_size=64, shuffle=True)   
    test_loader = torch.utils.data.DataLoader(test, batch_size=64, shuffle=True)
    dnn = DeepNeuralNets()
    dnn.to(device)

    optimizer = optim.Adam(dnn.parameters(), lr=0.01)
    scheduler = optim.lr_scheduler.StepLR(optimizer, 10, gamma=0.1)

    for epoch in range(1+args.num_epochs):
        train_model(dnn, optimizer, train_loader, epoch)
        scheduler.step()
        test_model(dnn, test_loader)

        if epoch %2 == 0:
            torch.save(dnn.state_dict(), 'data/model_weights.pth')
    
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
