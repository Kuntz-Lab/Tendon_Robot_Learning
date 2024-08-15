#!/usr/bin/env python3

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

