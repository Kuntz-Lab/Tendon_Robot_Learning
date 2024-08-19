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


from emd import earth_mover_distance
import argparse
import csv
import pickle
import sys
import tempfile
import itertools
import time

from chamferdist import ChamferDistance
from scipy.spatial import distance
from mpl_toolkits import mplot3d
import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch 
import torch.nn as nn
import torch.nn.functional as F
import trimesh

#from learn_tendon_pc import DeepDecoder
#from learn_actual_pc import DeepDecoder
from learn_tendon_shape import DeepNeuralNets, get_data
from simulated_data_collection import generate_point_cloud
import sim_to_real_property as LS
import cpptendon as T
from cpptendon.controller import Bounds, levenberg_marquardt

from TendonRobot import TendonRobotInterface
from CosseratRod import CosseratRodModel


np.random.seed(10)
plt.rcParams.update({'font.size': 17})
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def box_plot(data):
    print("A list of Chamfer distances: ", data)
    data = np.array(data)
    print("Mean and Std: ", data.mean(), data.std())
    fig, ax = plt.subplots(figsize= (7,4))
    plt.boxplot(data, vert=False)
    ax.set_xlabel('Chamfer distance (m)')
    ax.set(yticklabels=[])
    plt.show()

def test_plot(output, ground):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    #color = ['r', 'g', 'b', 'c', 'y']
    for i in range(len(output)):
        res = np.array(output[i].tolist()).reshape(-1,3)
        gt = np.array(ground[i, 4:].tolist()).reshape(-1,3)
        ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X',s=3)
        ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c='r', marker='X',s=3)
        #ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c=color[i], marker='X',s=3)

        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(0, 0.21)
        ax.set_xticks((-0.2, 0.0, 0.2))
        ax.set_yticks((-0.2, 0.0, 0.2))
        ax.set_zticks((0, 0.2))

def comparison_plotting(gt, res, count, idx, disp, tension):
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c='r', s=1)
    ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X',s=3)
    #ax.scatter3D(res[0,:], res[1,:], res[2,:], c='b', marker='X',s=3)
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.21)
    #ax.set_title(f"{idx[count]}, {disp[:4]}, \n{tension[:4]}", loc='center')

    #ax.legend(['Simulated robot', 'Physical robot'])
    #plt.grid(b=None)
    #plt.axis('off')

    #fig.savefig(f"../data/test_{count}.jpg")
    #for ii in range(0, 360, 36):
    #    ax.view_init(elev=10., azim=ii)
    #    fig.savefig(f"../data/learned_pc{ii}.jpg")

def len_to_tau(dataset, robot):
    taus = np.array([0.] * robot.state_size())
    home_shape = robot.home_shape(taus)
    bounds = Bounds()
    bounds.lower = [0,0,0,0]
    bounds.upper = [robot.tendons[i].max_tension  for i in range(len(robot.tendons))]
    data = []
   
    for row in dataset:

        ## Levenberg-Marquardt
        #length = row[:4]
        #def f(x):
        #    taus = x
        #    _, solved_lengths = robot.shape_and_lengths(taus, home_shape)
        #    return solved_lengths

        #result = levenberg_marquardt(f=f,
        #                             bounds=bounds,
        #                             initial_state=np.array([0,0,0,0]),
        #                             des=np.array(length),
        #                             max_iters=1000,
        #                             stop_threshold_err=1e-10,
        #                             )
        #row = np.array(row)
        #tau_pc = np.hstack(( result.state, row[8:] ))

        ## Alternative -- ratio
        length = row[:4]
        tau_res = np.zeros(4)
        for i in range(4):
            if length[i] <= 0:
                tau_res[i] = 0
            else:
                tau_res[i] = bounds.upper[i] * (length[i] / 0.04)
        row = np.array(row)
        tau_pc = np.hstack(( tau_res, row[8:] ))

        data.append(tau_pc) 

    return np.array(data)

def config_compare(csvfile='pbf_data.csv',
                    num_samples = 10,
                    #model_path='data/decoder_model_weights_with_disks_2.pth',
                    model_path='data/actual_decoder_model_weights_home_hys.pth',
                    file_name = 'data/data_fixed_first.pickle',
                    sample_method='from_file',
                    #tomlfile='../data/tendon_parameter.toml',
                    tomlfile='../data/optimized_robot_parameters.toml',
                    pc_enable=True,
                    N=512,
                    num_disk=9,
                    idx = None,#'data/data_fixed_third_idx.pickle',
                    actual=True,
                    normalize=True,
                    hys=False,
                    in_disp=True):
                    
    plt.gca().set_aspect('equal')
    robot = T.tendon.TendonRobot.from_toml(tomlfile)
    L = robot.specs.L

    with open(file_name, 'rb') as fin:
        commands = pickle.load(fin)
    #idx = np.random.choice(len(commands), size=num_samples, replace=False)
        
    with open(idx, 'rb') as dd:
        idx = pickle.load(dd)

    #idx = np.array([2819, 5596, 31, 543, 2999, 2867, 1702])
    test_dataset = list(commands[i] for i in idx)
    dataset_disp = test_dataset.copy()
    if in_disp:
        test_dataset = len_to_tau(test_dataset, robot)
    test_dataset = torch.from_numpy(np.array(test_dataset)).float().to(device)
    
    tension_size = 8 if hys else 4

    print("tension size: ", tension_size)
    ## Test
    #output = dnn(test_dataset[:,:tension_size])
    print("Comparison between simulated and physical robot's point clouds\n")

    box_plot_data = []
    end_point_error = []
    point_cloud = []
    distance_error = 0
    c_time = []
    for i in range(len(idx)):
        # Erase!
        if pc_enable:
            chamferDist = ChamferDistance()
            start = time.time()
            print("current tension: ", test_dataset[i, :tension_size])
            ground_truth= np.array(test_dataset[i, tension_size:].tolist()).reshape(-1,3)

            #generate_point_cloud(test_dataset[i, :4].tolist(), robot, num_disk, f'data/ground_truth_{i+1}.stl') 
            generate_point_cloud(test_dataset[i, :4].tolist(), robot, num_disk, f'data/ground_truth_{i+1}.stl') 
            mesh = trimesh.load(f'data/ground_truth_{i+1}.stl', file_type='stl')
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


                #robot = ContinuumRobot(0.2, 9.1e7, 0.0006, np.array([[0],[0],[-9.81]]), 1.25)
                #model = CosseratRodModel(robot)
                #result = model.Solve(np.array(test_dataset[i, :4].tolist()))
            else:
                result = np.array(mesh.sample(N, return_index=False))

            #print("Computation time for generating full point cloud of physics-based model: ", time.time() - start)
            c_time.append(time.time()-start)
            #print("ground truth: \n",ground_truth)
            #print("length of ground truth: \n",len(ground_truth))
            gt_torch = torch.from_numpy(ground_truth).float().to(device)
            gt_torch = gt_torch.reshape(1, ground_truth.shape[0], ground_truth.shape[1])

            ############################# 
            #### End point error

            ### Least Squares optimization to fit a backbone shape to point clouds
            gt_backbone, gt_end = identify_end_point(ground_truth)
            rs_backbone, rs_end = identify_end_point(result)

            ##comparison_plotting(gt_backbone, rs_backbone, gt_end, rs_end, 1, end_point=True)
            end_point_dist = np.linalg.norm(gt_backbone[-1] - rs_backbone[-1])
            ##print("end point dist: ", end_point_dist, " m")
            end_point_error.append(end_point_dist)
            ############################# 

            #when in order of p1x, p2x, ..., p1z, p2z, ...
            #result = np.array(output[i].tolist()).reshape(3,-1).T

            #when in order of p1x, p1y, p1z, ...
            #result = np.array(output[i].tolist()).reshape(-1,3)
            #print("result: \n",result)
            #print("length of result: \n",len(result))
            res_torch = torch.from_numpy(result).float().to(device)
            res_torch = res_torch.reshape(1, result.shape[0], result.shape[1])

            distance_error += chamferDist(gt_torch, res_torch).detach().cpu().item()
            distance= chamferDist(gt_torch, res_torch).detach().cpu().item()
            box_plot_data.append(distance)

            
        else:
            ground_truth = np.array(robot.forward_kinematics(test_dataset[i, :4].tolist()))
            result = test_fk(output[i].tolist(), L, ground_truth)
            distance_error += np.linalg.norm(ground_truth - result, axis=1).max()
            #print(f"\nAverage maximum L2 distance up to {i+1}th configuration: \n{distance_error / (i+1)} (m), \n{100 * distance_error / (i+1)} (cm), \n{1000*distance_error/ (i+1)} (mm)")
        ##Add Loss here
        comparison_plotting(ground_truth, result, i, idx, dataset_disp[i], test_dataset[i])
    c_time = np.array(c_time)
    print("Avg computation time: ", c_time.mean(), c_time.std())
    error = np.array(end_point_error)
    print("End-point error Mean and Std: ", error.mean(), error.std())

    print("robot: ",robot)
    print("E: ", robot.specs.E)
    box_plot(box_plot_data)
    #test_plot(point_cloud, test_dataset)
    plt.show()        

def test_fk(coef, L, d):
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

    xyz = np.transpose([x,y,z])
    return xyz

class DeepDecoder(nn.Module):

    def __init__(self, N, hys=False):
        super().__init__()
        self.N = N
        self.chamfer_loss = ChamferLoss()
        self.input = 8 if hys else 4
        self.fc1 = nn.Linear(self.input,128)
        self.bn1 = nn.GroupNorm(1,128)
        self.fc2 = nn.Linear(128,256)
        self.bn2 = nn.GroupNorm(1,256)
        self.fc3 = nn.Linear(256,512)
        self.bn3 = nn.GroupNorm(1,512)
        self.fc4 = nn.Linear(512,1024)
        self.bn4= nn.GroupNorm(1,1024)
        self.fc5 = nn.Linear(1024,3*self.N)

        #self.fc1 = nn.Linear(4,128)
        #self.bn1 = nn.GroupNorm(1,128)
        #self.fc2 = nn.Linear(128,512)
        #self.bn2 = nn.GroupNorm(1,512)
        #self.fc3 = nn.Linear(512,1024)
        #self.bn3 = nn.GroupNorm(1,1024)
        #self.fc4 = nn.Linear(1024,2048)
        #self.bn4 = nn.GroupNorm(1,2048)
        #self.fc5 = nn.Linear(2048,3*self.N)

    def forward(self, x):
        x = self.fc1(x)
        x = self.bn1(x)
        x = F.relu(x)

        x = self.fc2(x)
        x = self.bn2(x)
        x = F.relu(x)

        x = self.fc3(x)
        x = self.bn3(x)
        x = F.relu(x)
        
        x = self.fc4(x)
        x = self.bn4(x)
        x = F.relu(x)
        
        x = self.fc5(x)
        return x 

    def get_chamfer_loss(self, inp, out):
        return self.chamfer_loss(inp, out)

class ChamferLoss(nn.Module):
    def __init__(self):
        super(ChamferLoss, self).__init__()
        self.use_cuda = torch.cuda.is_available()

    def batch_pairwise_dist(self, x, y):
        bs, num_points_x, points_dim = x.size()
        _, num_points_y, _ = y.size()
        xx = torch.bmm(x, x.transpose(2, 1))
        yy = torch.bmm(y, y.transpose(2, 1))
        zz = torch.bmm(x, y.transpose(2, 1))
        diag_ind_x = torch.arange(0, num_points_x)
        diag_ind_y = torch.arange(0, num_points_y)
        if x.get_device() != -1:
            diag_ind_x = diag_ind_x.cuda(x.get_device())
            diag_ind_y = diag_ind_y.cuda(x.get_device())
        rx = xx[:, diag_ind_x, diag_ind_x].unsqueeze(1).expand_as(zz.transpose(2, 1))
        ry = yy[:, diag_ind_y, diag_ind_y].unsqueeze(1).expand_as(zz)
        P = (rx.transpose(2, 1) + ry - 2 * zz)
        return P

    def forward(self, preds, gts):
        P = self.batch_pairwise_dist(gts, preds)
        mins, _ = torch.min(P, 1)
        loss_1 = torch.sum(mins)
        mins, _ = torch.min(P, 2)
        loss_2 = torch.sum(mins)
        return loss_1 + loss_2

def outlier_filter(data, k=3, r=0.0045): #r=0.0035 
    D = distance.squareform(distance.pdist(data)) 
    closest = np.argsort(D, axis=1)                            
    k_neighbors = closest[:, 1:k+1]
    delete_index = []                                          
    for i, point in enumerate(data):                       
        k_points = data[k_neighbors[i]]                        
        dist_vec = np.linalg.norm(point - k_points, axis=1)    
        if np.all(dist_vec > r):                               
            delete_index.append(i)               
    return np.delete(data, delete_index, axis=0)

def identify_end_point(pc):

    pc = outlier_filter(pc)
    #ls_backbone = LS.compare(test_dataset.tolist(), robot, i, hys=hys)
    poly = LS.polynomial_regression3d(pc[:,0], pc[:,1], pc[:,2], 5)
    idx = np.linspace(0, len(pc)-1, num=41, endpoint=True, dtype=int)
    backbone = poly[idx]
    backbone = backbone[backbone[:,2].argsort()]
    eb = backbone[-1]

    dist = np.linalg.norm(eb-pc, axis=1)
    end = pc[dist.argsort()[0]]
    #print("optimized backbone points: \n",ls_backbone)
    return backbone, end

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
    This codebase is for plotting sim-to-real-gap, i.e., it plots the simulated robot shape and physical robot shape by reading dataset composed of tension configuration + corresponding robot point clouds (.pickle file).
    Only from home position:
    pickle: data/home_data_from_olivia.pickle
    Weight: data/actual_decoder_model_weights_home_hys.pth


    Both home position and random position 1: 

    pickle - data/hysteresis_integrated.pickle
    Weight - data/hysteresis_integrated.pth
    test - data/hysteresis_integrated_idx.pickle
    '''
    #parser.add_argument('csv', nargs='?', default='pbf_data.csv')
    #parser.add_argument('weight', default='data/actual_decoder_model_weights_home_hys.pth')
    parser.add_argument('pickle', default='data/data_from_olivia.pickle')
    parser.add_argument('test', default='data/traj_test_idx_0_4.pickle')
    #parser.add_argument('test', default='data/home_data_from_olivia.pickle')
    parser.add_argument('--num-samples', type=int, default=10)
    parser.add_argument('--hys', type=int, default=False)
    return parser

class ContinuumRobot(TendonRobotInterface):

    def __init__(self, length: float, youngs_modulus: float, shear_modulus: float, g: np.ndarray, rho: float):
        self.length = length
        self.g = g
        self.rho = rho
        self.youngs_modulus = youngs_modulus
        self.shear_modulus = shear_modulus
    
    def GetRadius(self, s: float):
        segment = s * 45
        seg_int = round(segment)
        prop = 0.0055 * 45
        if (segment + prop) > seg_int + 1:
            return 0.0108
        else:
            return 0.0058

    def GetCrossSectionArea(self, s: float):
        return np.pi * self.GetRadius(s)**2
    
    def GetSecondMomentsOfArea(self, s: float) -> tuple:
        i = np.pi * self.GetRadius(s)**4 / 4
        return (i, i)
    
    def GetTendonPositions(self, s: float) -> np.ndarray:
        r = 0.0108
        del_theta = (np.pi * 15) * s
        r_0 = r*np.array([[np.cos(-np.pi/6)],[np.sin(-np.pi/6)],[0.]])
        r_1 = r*np.array([[np.cos(np.pi/2)],[np.sin(np.pi/2)],[0.]])
        r_2 = r*np.array([[np.cos(np.pi*7/6)],[np.sin(np.pi*7/6)],[0.]])
        r_3 = r*np.array([[np.cos(np.pi*7/6 + del_theta)],[np.sin(np.pi*7/6 + del_theta)],[0.]])
        return np.concatenate(([r_0], [r_1], [r_2], [r_3]))
    
    def GetTendonPositionDerivatives(self, s: float) -> np.ndarray:
        r = 0.0108
        slope = np.pi * 15
        r_0 = np.array([[0.],[0.],[0.]])
        r_1 = np.array([[0.],[0.],[0.]])
        r_2 = np.array([[0.],[0.],[0.]])
        r_3 = r*np.array([[-slope*np.sin(np.pi*7/6 + slope*s)],[slope*np.cos(np.pi*7/6+slope*s)],[0.]])
        return np.concatenate(([r_0], [r_1], [r_2], [r_3]))
    
    def GetTendonPositionSecondDerivatives(self, s: float) -> np.ndarray:
        r = 0.0108
        slope = np.pi * 15
        r_0 = np.array([[0.],[0.],[0.]])
        r_1 = np.array([[0.],[0.],[0.]])
        r_2 = np.array([[0.],[0.],[0.]])
        r_3 = r*np.array([[-(slope**2)*np.sin(np.pi*7/6 + slope*s)],[-(slope**2)*np.cos(np.pi*7/6 + slope*s)],[0.]])
        return np.concatenate(([r_0], [r_1], [r_2], [r_3]))

    def GetPStar(self, s: float):
        return np.array([[0],[0],[s]])
    
    def GetRStar(self, s: float):
        return np.array([[1,0,0],
                         [0,1,0],
                         [0,0,1]])
    
    def GetUStar(self, s: float):
        return np.array([[0],[0],[0]])
    
    def GetUStarDot(self, s: float):
        return np.array([[0],[0],[0]])
    
    def GetVStar(self, s: float):
        return np.array([[0],[0],[1]])
    
    def GetVStarDot(self, s: float):
        return np.array([[0],[0],[0]])

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    config_compare(
        num_samples=args.num_samples,
        file_name=args.pickle,
        idx=args.test,
        hys=args.hys,
    )
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))

