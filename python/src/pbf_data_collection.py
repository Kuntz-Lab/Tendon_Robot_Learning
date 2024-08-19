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
import itertools
import pickle
import sys

import numpy as np

import cpptendon as T

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'To get coefficients of Polynomial Basis Function (PBF)'
    parser.add_argument('toml', nargs='?', default='default.toml')
    parser.add_argument('--num-tensions', type=int, default=21)
    parser.add_argument('--num-coeffs', type=int, default=15)
    #parser.add_argument('csv', nargs='?', default='data/pbf_data.csv')
    return parser

def data_collection(tomlfile='default.toml',
                    num_tensions=21,
                    data_storage='data/pbf_data.csv',
                    num_coeffs=15
                    ):

    #d = robot.home_shape().p
    b = T.controller.Bounds()
    b.lower = -100 * np.ones(num_coeffs)
    b.upper = 100 * np.ones(num_coeffs)
    L = 0.2
    robot = T.tendon.TendonRobot.from_toml(tomlfile)

    with open(data_storage, 'w') as store:
        writer = csv.writer(store)
        writer.writerow([f't{i+1}' for i in range(len(robot.tendons))] 
                        + [f'coeff{i+1}' for i in range(num_coeffs)])

        config_space = [np.linspace(0.0, t.max_tension, num_tensions) for t in robot.tendons]

        for config in itertools.product(*config_space):
            backbone = robot.forward_kinematics(config)
             
            result = T.controller.levenberg_marquardt(lambda x: np.linalg.norm(fk(x, L, backbone), axis=1), 
                                                      bounds=b, 
                                                      initial_state=np.ones(num_coeffs),
                                                      des=np.zeros(len(backbone)), 
                                                      max_iters=500)

            writer.writerow(list(config) + list(result.state))

#def fknorm(x): return np.linalg.norm(fk(x, L, d), axis=1) 
def fk(coef, L, d):
    s = np.linspace(0, 1, len(d))
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
    delta = np.array(d)
    delta[:, 0] -= x
    delta[:, 1] -= y
    delta[:, 2] -= z
    return delta


def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    data_collection(
        tomlfile=args.toml,
        num_tensions=args.num_tensions,
        num_coeffs=args.num_coeffs,
        )

    return 0 

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
