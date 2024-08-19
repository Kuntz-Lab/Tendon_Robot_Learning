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


import sys
import numpy as np
import csv
import argparse
import trimesh
import pickle
from cpptendon import collision, tendon

# This script reads a plan sequence(from CSV files), convert it to point cloud data(via trimesh), and store them in a separate file as dataset (pickle).

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
       Generate a tendon robot simulation pc dataset for each particular control (tendon configuration). 
    '''
    parser.add_argument('robot_toml')
    parser.add_argument('-i', '--input', default='solution-lung-2.csv')
    parser.add_argument('-o', '--output', default='pc_sim.pickle', help='output pickle file (default is pc_sim.pickle')
    return parser


def csv_reader(csv_file):
    rows = []
    fields = []

    with open(csv_file, 'r') as f:
        csvreader = csv.reader(f)
        fields = next(csvreader)

        for row in csvreader:
            rows.append(row)

        ### Only tendon robot
        #indices = [fields.index(i) for i in ['i', 'tau_1', 'tau_2', 'tau_3', 'tau_4', 'tau_5']]

        ### With the collapsed lung env
        indices = [fields.index(i) for i in ['i', 'tau_1', 'tau_2', 'tau_3', 'theta', 's_start']]

    data = np.array([[float(row[i]) for i in indices] for row in rows])
    return data[:, 1:]

def plan_to_stl(robot, plan_seq):
    for i,plan in enumerate(plan_seq): 
        backbone_shape = robot.forward_kinematics(plan)
        shape = collision.CapsuleSequence(backbone_shape, robot.r)
        mesh = shape.to_mesh(n_circle_pts=16, add_spherical_caps=False)
        mesh.to_stl(f'tempo_test_lung_sequence_{i+1}.stl', binary=True)


def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    
    robot = tendon.TendonRobot.from_toml(args.robot_toml)
    plan_seq = csv_reader(args.input)
    plan_to_stl(robot, plan_seq)


if __name__=='__main__':
    main(sys.argv[1:])

