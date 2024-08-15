#!/usr/bin/env python3

import argparse
import csv
import os
import pickle
import sys

import numpy as np
import pandas as pd
import torch

device = torch.device("cuda")

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'TODO'
    parser.add_argument('csv', nargs='?', default='simulated_data_6.csv')
    return parser

def csv_to_pkl(csvfile = 'simulated_data_6.csv',
               pklfile = 'data/optimized_phys_robot_2.pickle'):
    data_read = pd.read_csv(csvfile)
    data = [list(data_read[i]) for i in data_read.columns] 
    data = np.array(data).transpose().tolist()

    with open(pklfile, 'wb') as fin:
        pickle.dump(data, fin)
    
def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    csv_to_pkl(csvfile=args.csv)

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
