#!/usr/bin/env python3

import argparse
import csv
import itertools
import pickle
import sys

import torch
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

device = torch.device('cuda')
chamferDist = ChamferDistance()

def outlier_filter(data, k=3, r=0.0064): #r=0.0045 
    print("r: ", r)
    D = distance.squareform(distance.pdist(data)) 
    closest = np.argsort(D, axis=1)                            
    k_neighbors = closest[:, 1:k+1]
    delete_index = []                                          
    for i, point in enumerate(data):                       
        k_points = data[k_neighbors[i]]                        
        dist_vec = np.linalg.norm(point - k_points, axis=1)    
        #if np.all(dist_vec > r):                               
        if np.any(dist_vec > r):                               
            delete_index.append(i)               
    return np.delete(data, delete_index, axis=0)

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

def find_E(tomlfile='../data/sim_robot_limits.toml',
            data_storage='data/pbm_opt_data_2.pickle',
                    ):
    #data_storage='data/test_tendon_data.pickle',

    #d = robot.home_shape().p
    L = 0.2
    robot = T.tendon.TendonRobot.from_toml(tomlfile)

    with open(data_storage, 'rb') as fin:
        commands = pickle.load(fin)

    E = []
    #idx = np.random.randint(len(commands))
    idx = np.arange(len(commands))
    init = 1.0e+8

    for ii in idx:
        print("Before LS: ", robot.specs.E)
        param = compare(commands, robot, init, ii)
        print(param)

        E.append(param)
        init = param #[0]

    print("Young's Modulus: ", E)

def compare(commands, robot, init_val, idx, hys=False):
    #print("idx: ", idx)
    config_idx = 8 if hys else 4
    config = np.array(commands[idx][:4])
    pc = np.array(commands[idx][config_idx:]).reshape(-1,3)
    
    pc = outlier_filter(pc[np.random.choice(len(pc), size=600, replace=False)])

    result = least_squares(lambda x:func(x, robot, config, pc),
                           x0=[init_val],
                           bounds=(1e+06, 5e+12),
                           )
    robot.specs.E = result.x
    #robot.specs.E = 1.31e+08 #86462550#1.072e+8
    #print("LS results: ",result.x)
    print("After LS: ", robot.specs.E)

    sim_shape = np.array(robot.forward_kinematics(config))
    #comparison_plotting(sim_shape, pc, 1)

    return robot.specs.E

def polynomial_regression3d(x, y, z, degree):
    # sort data to avoid plotting problems
    x, y, z = zip(*sorted(zip(x, y, z)))

    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    
    data_yz = np.array([y,z])
    data_yz = data_yz.transpose()

    polynomial_features= PolynomialFeatures(degree=degree)
    x_poly = polynomial_features.fit_transform(x[:, np.newaxis])


    model = LinearRegression()
    model.fit(x_poly, data_yz)
    y_poly_pred = model.predict(x_poly)

    rmse = np.sqrt(mean_squared_error(data_yz,y_poly_pred))
    r2 = r2_score(data_yz,y_poly_pred)
    poly = np.hstack((x.reshape(-1,1), y_poly_pred))

    return poly

def func(param, robot, config, gt):
    robot.specs.E = param[0] 
    print("Stiffness in optimization func: ",robot.specs.E)
    backbone = np.array(robot.forward_kinematics(config))
    poly = polynomial_regression3d(gt[:,0], gt[:,1], gt[:,2], 5)
    idx = np.linspace(0, len(gt)-1, num=len(backbone), endpoint=True, dtype=int)
    p = poly[idx]
    
    comparison_plotting(backbone, p, 1)
    #res_torch = torch.from_numpy(backbone).float().to(device)
    #res_torch = res_torch.reshape(1, backbone.shape[0], backbone.shape[1])

    #gt_torch = torch.from_numpy(p).float().to(device)
    #gt_torch = gt_torch.reshape(1, p.shape[0], p.shape[1])

    #delta = chamferDist(gt_torch, res_torch).detach().cpu().item() 
    delta = np.linalg.norm(backbone - p, axis=1).mean()
    return delta

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
    find_E(
        tomlfile=args.toml,
        )

    return 0 

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))

