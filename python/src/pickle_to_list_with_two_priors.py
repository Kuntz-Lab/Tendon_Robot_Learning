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
    parser.description = 'Convert a pickle file about the tendon robot data collection to a pickle file with a list composed of current tension, previous tension, and second previous tension, suitable for network training.'
    parser.add_argument('pkl', default='data_storage_hysteresis.pickle')
    parser.add_argument('pkl_to', default='hysteresis_integrated.pickle')
    parser.add_argument('-H', '--hys', type=int, default=1)
    return parser

def pkl_to_list(file_fro = 'data_storage_hysteresis.pickle',
               file_to = '../data/test_hysteresis_train.pickle',
               hys=True):
    with open(file_fro, 'rb') as fin:
        commands = pickle.load(fin)
    data = [] 

    if hys:
        for k in commands.keys():
            tau_curr = list(commands[k][2]['tau'])
            tau_prev = list(commands[k][0]['tau'])
            tau_prev2 = [0.0 for _ in range(4)]
            tau = tau_curr + tau_prev + tau_prev2
            data_list = np.array([list(commands[k][3][i])[:-1] for i in range(len(commands[k][3]))])
            idx = np.random.choice(len(data_list), size=512, replace=False)
            data_list = data_list[idx].tolist()
            pc = list(chain.from_iterable( data_list  ))
            data.append(tau + pc)
        for j in commands.keys():
            if j==1:
                continue
            tau_curr = list(commands[j][0]['tau'])
            tau_prev = [0.0 for _ in range(4)]
            tau_prev2 = list(commands[j-1][2]['tau'])
            tau = tau_curr + tau_prev + tau_prev2
            data_list = np.array([list(commands[j][1][i])[:-1] for i in range(len(commands[j][1]))])
            idx = np.random.choice(len(data_list), size=512, replace=False)
            data_list = data_list[idx].tolist()
            pc = list(chain.from_iterable( data_list  ))
            data.append(tau + pc)

    else:
        for k,row in enumerate(commands):
            if not k:
                continue
            new_row = row[:8] + commands[k-1][:4] + row[8:]
            
            data.append(new_row)

    with open(file_to, 'wb') as fout:
        pickle.dump(data, fout)
    
def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    pkl_to_list(file_fro=args.pkl,
                file_to=args.pkl_to,
                hys=args.hys,
                )

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
