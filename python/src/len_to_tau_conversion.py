#!/usr/bin/env python3

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
