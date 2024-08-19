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
import datetime
import itertools
import sys
import pickle

import numpy as np
from numpy import pi, sin, cos

from cpptendon.tendon import TendonRobot
from cpptendon.controller import levenberg_marquardt, Bounds

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Convert tendon lengths to tendon tension
        '''
    parser.add_argument('robot_toml')
    parser.add_argument('data_len')
    parser.add_argument('-o', '--output', default='data/traj_zero_slack_tau_converted.pickle',
                        help='output pickle file (default is traj_zero_slack_tau_converted.pickle)')

    return parser

def len_to_tau(robot, length):
    tau_data = []
    for row in length:
        l = np.array(row[:4])
        pos_idx = np.where(l > 0)[0]
        taus = np.array([0.] * robot.state_size())
        home_shape = robot.home_shape(taus)
        bounds=Bounds()
        bounds.lower=[0] * len(pos_idx)
        bounds.upper = [robot.tendons[i].max_tension*2 for i in pos_idx]

        def f(x):
            taus[pos_idx] = x
            _, solved_lengths = robot.shape_and_lengths(taus, home_shape)
            return solved_lengths

        result = levenberg_marquardt(
                f=f,
                bounds=bounds,
                initial_state=np.array(taus[pos_idx]),
                des=np.array(l),
                max_iters=1000,
                stop_threshold_err=1e-10,
                )

        taus[pos_idx] = result.state
        #tau_data.append(taus)
        tau_data.append(list(taus) + row[8:])
    return tau_data
    

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    robot = TendonRobot.from_toml(args.robot_toml)

    with open(args.data_len, 'rb') as fin:
        length = pickle.load(fin)
    tau_shape = len_to_tau(robot, length)

    with open(args.output, 'wb') as fout:
        pickle.dump(tau_shape, fout)


if __name__ == '__main__':
    main(sys.argv[1:])
