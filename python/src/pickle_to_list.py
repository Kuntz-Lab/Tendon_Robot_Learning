#!/usr/bin/env python3

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
    parser.description = 'Convert a pickle file about the tendon robot data collection to a pickle file with a list suitable for network training.'
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
            tau_curr = list(commands[k][2]['len'])
            tau_prev = list(commands[k][0]['len'])
            tau = tau_curr + tau_prev
            data_list = np.array([list(commands[k][3][i])[:-1] for i in range(len(commands[k][3]))])
            idx = np.random.choice(len(data_list), size=512, replace=False)
            data_list = data_list[idx].tolist()
            pc = list(chain.from_iterable( data_list  ))
            data.append(tau + pc)
    else:
        for k in commands.keys():
            #tau_prev = [-0.01 for _ in range(4)]
            #tau_curr = list(commands[k][0]['len'])    
            #tau = tau_curr + tau_prev
            tau = list(commands[k][0]['len'])    

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
                file_to=args.pkl_to,
                hys=args.hys,
                )

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
