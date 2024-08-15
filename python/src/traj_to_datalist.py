#!/usr/bin/env python3

import argparse
import csv
import os
import pickle
import sys
from itertools import chain

import numpy as np
import pandas as pd
import torch

device = torch.device("cuda")

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'This codebase converts the tendon robot trajectory in pickle file to a dataset list in pickle, i.e., each row of the dataset is current tendon + previous tendon + current point cloud.'
    parser.add_argument('pkl', default='data/data_storage_hysteresis_trajectory.pickle')
    parser.add_argument('pkl_to', default='data/hysteresis_trajectory.pickle')
    return parser

def pkl_to_list(file_fro = 'data/data_storage_hysteresis_trajectory.pickle',
               file_to = 'data/hysteresis_trajectory.pickle',
               collection=True,
               gpt=True,
               hys=True):
    with open(file_fro, 'rb') as fin:
        commands = pickle.load(fin)
    data = []

    if hys:
        if collection:
            # Or remove "command" type from the data
            if gpt:
                for key, cmd in commands.items():
                    data2 = []
                    for i in range(len(cmd)-1):
                        if i % 2:
                            continue
                        if i == 0:
                            #tau_prev = [-0.01 for _ in range(4)]
                            continue
                        else:
                            if i == 2:
                                tau_prev = [-0.01 for _ in range(4)] #-0.01
                            else:
                                tau_prev = list(cmd[i-2]['len'])
                        tau_curr = list(cmd[i]['len'])
                        tau = tau_prev + tau_curr

                        data_list_prev = np.array([list(cmd[i-1][k])[:-1] for k in range(len(cmd[i-1]))])
                        idx_prev = np.random.choice(len(data_list_prev), size=512, replace=False)
                        data_list_prev = data_list_prev[idx_prev].tolist()
                        pc_prev = list(chain.from_iterable(data_list_prev))

                        data_list = np.array([list(cmd[i+1][k])[:-1] for k in range(len(cmd[i+1]))])
                        idx = np.random.choice(len(data_list), size=512, replace=False)
                        data_list = data_list[idx].tolist()
                        pc = list(chain.from_iterable(data_list))

                        data2.append(tau + pc_prev + pc)
                    data += data2 
                    
            else:
                for key, cmd in commands.items():
                    data2 = []
                    for i in range(len(cmd)-1):
                        if i % 2:
                            continue
                        if i == 0:
                            #tau_prev = [-0.01 for _ in range(4)]
                            continue
                        else:
                            if i == 2:
                                tau_prev = [-0.01 for _ in range(4)] #-0.01
                            else:
                                tau_prev = list(cmd[i-2]['len'])
                        tau_curr = list(cmd[i]['len'])
                        tau = tau_curr + tau_prev
                        data_list = np.array([list(cmd[i+1][k])[:-1] for k in range(len(cmd[i+1]))])
                        idx = np.random.choice(len(data_list), size=512, replace=False)
                        data_list = data_list[idx].tolist()
                        pc = list(chain.from_iterable(data_list))
                        data2.append(tau + pc)
                    data += data2 
        else:
            commands = commands[1]
            for i in range(len(commands)):
                if i % 2:
                    continue
                if i == 0:
                    tau_prev = [-0.01 for _ in range(4)]
                else:
                    tau_prev = list(commands[i-2]['len'])
                tau_curr = list(commands[i]['len'])
                tau = tau_curr + tau_prev
                data_list = np.array([list(commands[i+1][k])[:-1] for k in range(len(commands[i+1]))])
                idx = np.random.choice(len(data_list), size=512, replace=False)
                data_list = data_list[idx].tolist()
                pc = list(chain.from_iterable(data_list))
                data.append(tau + pc)

    else:
        for k in commands.keys():
            tau = list(commands[k][0]['tau'])    
            data_list = np.array([list(commands[k][1][i])[:-1] for i in range(len(commands[k][1]))])
            idx = np.random.choice(len(data_list), size=512, replace=False)
            data_list = data_list[idx].tolist()
            
            pc = list(chain.from_iterable( data_list  ))
            data.append(tau + pc)

    with open(file_to, 'wb') as fout:
        pickle.dump(data, fout)
    
def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    pkl_to_list(file_fro=args.pkl,
                file_to=args.pkl_to)

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
