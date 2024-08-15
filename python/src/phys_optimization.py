#!/usr/bin/env python3

import argparse
import csv
import itertools
import pickle
import sys

import torch
import trimesh
import numpy as np
import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from chamferdist import ChamferDistance
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score
from sklearn.linear_model import LinearRegression
from scipy.optimize import least_squares
from scipy.spatial import distance

import cpptendon as T
from cpptendon.controller import levenberg_marquardt, Bounds
from simulated_data_collection import generate_point_cloud
from test_sim_config import len_to_tau
from emd import earth_mover_distance

np.random.seed(1000)
device = torch.device('cuda')
chamferDist = ChamferDistance()

def generate_pc_from_phys(obs, robot, config_idx=8):
    obs = torch.from_numpy(np.array(obs)).float().to(device)
    num_disk = 9
    N=512
    point_cloud = []

    for i in range(len(obs)):
        generate_point_cloud(obs[i, :4].tolist(), robot, num_disk, f'data/phys_{i+1}.stl') 
        mesh = trimesh.load(f'data/phys_{i+1}.stl', file_type='stl')
        if num_disk:
            for j in range(num_disk):
                disk = trimesh.load(f'data/disk{j}_test.stl', file_type='stl')
                if not j:
                    disk_pts = np.array(disk.sample(N, return_index=False))
                else:
                    disk_pts = np.vstack((disk_pts, np.array(disk.sample(N, return_index=False))))
            result = np.array(mesh.sample(4*N, return_index=False))
            result = np.vstack((result, disk_pts))
            result = result[np.random.choice(len(result), size=N, replace=False)]
            point_cloud.append(result)
    return np.array(point_cloud)

def comparison_plotting(gt, res, count):
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c='r', s=1)
    ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X', s=3)
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.21)
    plt.axis('off')
    plt.grid(b=None)
    plt.show()

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'To find the parameter E (Youngs Modulus) to match the backbone shape of simulation and physical robot'
    parser.add_argument('toml', nargs='?', default='sim_robot_limits.toml')
    #parser.add_argument('csv', nargs='?', default='data/pbf_data.csv')
    return parser

def find_E(tomlfile='../data/optimized_robot_parameters.toml',
            #data_storage='data/tau_pc_optimized.pickle', 
            #data_storage='data/pbm_opt_data.pickle', #dataset with the weights hanging on tendons(DW),
            data_storage='data/pbm_opt_data_kgf.pickle', #dataset with the weights hanging on tendons(DW),
            DW=False,
                    ):
    L = 0.2

    if DW:
        data_storage='data/pbm_opt_data_kgf.pickle'
    else:
        data_storage='data/data_displacement_hys_traj.pickle'
    
    #temp_data = 'data/traj_zero_slack_tau_converted_2.pickle'

    #with open(temp_data, 'rb') as fin:
    #    commands = np.array(pickle.load(fin))[np.random.choice(5000, size=6, replace=False)]

    with open(data_storage, 'rb') as fin:
        #commands = np.array(pickle.load(fin))[[2,3]]
        full_commands = pickle.load(fin)
        random_idx = np.random.choice(len(full_commands), size=50, replace=False)
        commands = np.array(full_commands)[random_idx]

    print("Indices for optimization of Physics-based model on dataset: ", random_idx)

    E = []
    test_idx = 1 if DW else 30
    #test_idx = 3 #DW
    robot = T.tendon.TendonRobot.from_toml(tomlfile)
    commands = len_to_tau(commands, robot)
    init = 10.0
    #init = 10 #8.3e+02 #3.036 #9.1e-01

    param = optimize_phys(commands, robot, init, hys=False)
    print("Young's Modulus: ", param[0], "e+08")
    robot.specs.E = param[0] * 1.0e+08
    #robot.tendons[3].C = [-2.0944, param[0]]


    test_gen_pc = generate_pc_from_phys(commands, robot, config_idx=4)[test_idx]
    test_obs_pc = np.array(commands[test_idx][4:]).reshape(-1,3)
    
    ## if not DW
    #test_gen_pc = generate_pc_from_phys(full_commands, robot, config_idx=4)[test_idx]
    #test_obs_pc = np.array(full_commands[test_idx][4:]).reshape(-1,3)
    comparison_plotting(test_gen_pc, test_obs_pc, 1)

def optimize_phys(commands, robot, init_val, hys=False):
    #print("idx: ", idx)
    #config = np.array(commands[idx][:4])
    config_idx = 8 if hys else 4
    bounds = Bounds()
    ## only Stiffness
    #bounds.lower= [8e-01]
    #bounds.upper=[1e+03]

    bounds.lower= [1]
    bounds.upper=[10]

    ## Stiffness + tendon angles
    #bounds.lower = [-60]
    #bounds.upper = [0]

    pc_obs = []
    for idx in range(len(commands)):
        pc = np.array(commands[idx][config_idx:]).reshape(-1,3)
        #pc = pc[np.random.choice(len(pc), size=512, replace=False)]
        pc_obs.append(pc)
        #pc = outlier_filter(pc[np.random.choice(len(pc), size=600, replace=False)])

    pc_obs = np.array(pc_obs)

    result = least_squares(lambda x:func(x, robot, commands, pc_obs),
                           x0=[init_val],
                           #bounds=(0, 100),
                           method='lm',
                           diff_step=0.1,
                           #x_scale=1, 
                           )

    print()
    print('LevmarResult: (Stiffness parameter)')
    print('  state:               ', result.x)
    print('  cost:                ', result.cost)
    print('  residuals:           ', result.fun)
    print('  success:             ', result.success)
    print('  num_fk_calls:        ', result.nfev)
    print('  term_reason:         ', result.status)
    print('  message:             ', result.message)
    print()

    #result = levenberg_marquardt(lambda x: func(x, robot, commands, pc_obs),
    #                             bounds=bounds,
    #                             initial_state=np.array([init_val]), #Only Stiffness
    #                             #initial_state=np.array([-47.124]),
    #                             des=np.zeros(1),
    #                             max_iters=100,
    #                             #mu_init=1.0e+6,
    #                             #stop_threshold_err=1e-6,
    #                             )
    #                             

    #print()
    #print('LevmarResult: (Stiffness parameter)')
    #print('  state:               ', result.state)
    #print('  fout:                ', result.fout)
    #print('  err_init:            ', result.err_init)
    #print('  err:                 ', result.err)
    #print('  JT_err:              ', result.JT_err)
    #print('  Dp:                  ', result.Dp)
    #print('  mu_over_JTJ:         ', result.mu_over_JTJ)
    #print('  iters:               ', result.iters)
    #print('  term_condition:      ', result.term_condition)
    #print('  num_fk_calls:        ', result.num_fk_calls)
    #print('  num_jacobian_calls:  ', result.num_jacobian_calls)
    #print('  num_linear_solves:   ', result.num_linear_solves)
    #print('  term_reason:         ', result.term_reason)
    #print()
    return result.x 

def func(param, robot, obs, gt):
    delta = 0
    robot.specs.E = param[0] * 1.0e+08
    #robot.tendons[3].C = [-2.0944, param[0]]

    pc_gen = generate_pc_from_phys(obs, robot, config_idx=4)
    
    for n,pc in enumerate(pc_gen):
        res_torch = torch.from_numpy(pc).float().to(device)
        res_torch = res_torch.reshape(1, pc.shape[0], pc.shape[1])

        gt_torch = torch.from_numpy(gt[n]).float().to(device)
        gt_torch = gt_torch.reshape(1, gt[n].shape[0], gt[n].shape[1])

        delta += chamferDist(gt_torch, res_torch).detach().cpu().item() 

    return np.array([delta])


def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    find_E(
        tomlfile=args.toml,
        )

    return 0 

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))

