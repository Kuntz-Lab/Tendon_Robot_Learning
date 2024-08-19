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
import itertools
import pickle
import sys
import tempfile
import time

from chamferdist import ChamferDistance
from mpl_toolkits import mplot3d
from scipy.spatial import distance
import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'figure.max_open_warning': 0})
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

np.random.seed(seed=1000) #previous: 1000
plt.rcParams.update({'font.size': 18})
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def box_plot(data):
    print("A list of Chamfer distances: ", data)
    data = np.array(data)
    median = np.array(np.median(data)).reshape(-1,1)
    median_idx = np.argsort(np.linalg.norm(data.reshape(-1,1) - median, axis=1))[0]

    print("Mean and Std: ", data.mean(), data.std())
    print("Worst, Median, Best: ", data.max(), np.median(data), data.min())
    print("Worst, Median, Best indices: ", data.argmax(), median_idx, data.argmin())
    print("Worst idx: ", data.argmax())
    #print("Max and Min: ", data.max(), data.min())
    #print("Max idx: ", np.argmax(data))
    #print("idx that exceed 0.02 (m): ", np.where(data > 0.02)) 
    #print("Values that exceed 0.02 (m): ", data[np.where(data > 0.02)])
    fig, ax = plt.subplots(figsize= (7,4))
    plt.boxplot(data, vert=True)
    ax.set_xlabel('Chamfer distance (m)')
    ax.set(yticklabels=[])
    plt.show()

def test_plot(output, ground):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    #color = ['r', 'g', 'b', 'c', 'y']
    for i in range(len(output)):
        res = np.array(output[i].tolist()).reshape(-1,3)
        gt = np.array(ground[i, 8:].tolist()).reshape(-1,3)
        ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X',s=3)
        ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c='r', marker='X',s=3)
        #ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c=color[i], marker='X',s=3)

        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(0, 0.21)
        #ax.set_xticks((-0.2, 0.0, 0.2))
        #ax.set_yticks((-0.2, 0.0, 0.2))
        #ax.set_zticks((0, 0.2))
    #fig.savefig(f"../data/traj_combined_april.jpg")

def comparison_plotting(gt, res, count, cdist, end_point=False, metric=None, ax=None):
    #print("Number of points in ground truth and model prediction: ", len(gt), len(res))
    
    if ax is None:
        fig = plt.figure()
        ax = plt.axes(projection='3d')

    ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c='r', s=5)
    ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X',s=3)
    if end_point:
        #ax.scatter3D(gt[-1, 0], gt[-1,1], gt[-1,2], c='c', s=50)
        #ax.scatter3D(res[-1, 0], res[-1,1], res[-1,2], c='g', s=50)
        ax.scatter3D(gt_end[0], gt_end[1], gt_end[2], c='c', s=50)
        ax.scatter3D(rs_end[0], rs_end[1], rs_end[2], c='g', s=50)

    ax.set_xlim(-0.15, 0.15)
    ax.set_ylim(-0.15, 0.15)
    ax.set_zlim(0, 0.20)
    #ax.set_xlim(-0.3, 0.3)
    #ax.set_ylim(-0.3, 0.3)
    #ax.set_zlim(0, 0.21)
    ax.set_xticks((-0.1, 0.0, 0.1))
    ax.set_yticks((-0.1, 0.0, 0.1))
    ax.set_zticks((0, 0.2))
    if metric is not None:
        ax.set_title(f'current index is {count} and Chamfer is {cdist[count]}')

    #ax.legend(['Ground Truth', 'Learned'])
    plt.axis('off')
    plt.grid(b=None)
    #plt.show()

    #fig.savefig(f"../data/traj_test_april_{count}.jpg")
    #for ii in range(0, 360, 36):
    #    ax.view_init(elev=10., azim=ii)
    #    fig.savefig(f"../data/learned_pc{ii}.jpg")

def config_compare(csvfile='pbf_data.csv',
                    num_samples = 10,
                    #model_path='data/decoder_model_weights_with_disks_2.pth',
                    model_path='data/actual_decoder_model_weights_home_hys.pth',
                    file_name = 'data/home_data_from_olivia.pickle',
                    sample_method='from_file',
                    tomlfile='../data/sim_robot_limits.toml',
                    pc_enable=True,
                    N=512,
                    num_disk=9,
                    idx = 'data/fixed_home_30_test_cases.pickle',
                    actual=True,
                    normalize=True,
                    trajectory=True,
                    hys=0):
                    

    print(num_samples)
    plt.gca().set_aspect('equal')
    robot = T.tendon.TendonRobot.from_toml(tomlfile)
    L = robot.specs.L
    tension_size = 8 if hys else 4

    #dnn = DeepNeuralNets()
    dnn = DeepDecoder(N, hys=hys)
    dnn.load_state_dict(torch.load(model_path))
    dnn.to(device)

    ## Random samples for test
    if sample_method == 'random':
        #norm = [robot.tendons[i].max_tension/20 for i in range len(robot.tendons)] if normalize  else [[1.0] * 4]
        test_dataset = np.random.random_sample((num_samples, len(robot.tendons))) * robot.tendons[0].max_tension
        #for i in range(len(robot.tendons)):
        #    test_dataset[:, i] *= robot.tendons[i].max_tension
    ## Sample Test dataset from data collection
    elif sample_method == 'from_file':
        #file_name = 'data/test_hysteresis_integrated.pickle'
        #with open('data/test_hysteresis_idx.pickle', 'rb') as ff:
        #    idx = pickle.load(ff)
        with open(file_name, 'rb') as fin:
            ## if data structure is a Python list
            commands = pickle.load(fin)
        #print("idx len: ", len(commands))
        if idx is None:
            #idx = np.random.choice(len(commands), size=num_samples, replace=False)
            ## For trajectory analysis
            #idx = np.random.choice(len(commands), size=len(commands), replace=False)
            idx = np.arange(len(commands))
        else:
            print(idx)
            with open(idx, 'rb') as ff:
                idx = pickle.load(ff)

        print("Test index: ", idx)
        test_dataset = list(commands[i] for i in idx)
    else:
        if not pc_enable:
            with open(csvfile, "r") as fin:
                reader = csv.DictReader(fin)
                header = reader.fieldnames
                commands = list(reader)
            idx = np.random.choice(len(commands), size=num_samples, replace=False)
            samples = list(commands[i] for i in idx)
            test_dataset = []
            for i in range(num_samples):
                coef = [float(samples[i][f'coeff{j+1}']) for j in range(15)]  
                tau = [float(samples[i][f't{j+1}']) for j in range(len(robot.tendons))]
                test_dataset.append(tau + coef)

    test_dataset = torch.from_numpy(np.array(test_dataset)).float().to(device)
    
    ## Test
    start=time.time()
    output = dnn(test_dataset[:,:tension_size])
    print("The computation time: ", time.time() - start)
    print("Comparison between ground truth and learned point clouds\n")

    #daniel_data = []

    #test_plot(output)
    box_plot_data = []
    end_point_error = []
    distance_error = 0
    for i in range(len(output)):
        if pc_enable:
            chamferDist = ChamferDistance()
            if actual:
                ground_truth = np.array(test_dataset[i, 8:].tolist()).reshape(-1,3)
                ground_truth = outlier_filter(ground_truth)
            else:
                generate_point_cloud(test_dataset[i, :tension_size].tolist(), robot, num_disk, f'data/ground_truth_{i+1}.stl') 
                mesh = trimesh.load(f'data/ground_truth_{i+1}.stl', file_type='stl')
                if num_disk:
                    for j in range(num_disk):
                        disk = trimesh.load(f'data/disk{j}_test.stl', file_type='stl')
                        if not j:
                            disk_pts = np.array(disk.sample(N, return_index=False))
                        else:
                            disk_pts = np.vstack((disk_pts, np.array(disk.sample(N, return_index=False))))
                    ground_truth = np.array(mesh.sample(4*N, return_index=False))
                    ground_truth = np.vstack((ground_truth, disk_pts))
                    ground_truth = ground_truth[np.random.choice(len(ground_truth), size=N, replace=False)]
                else:
                    ground_truth = np.array(mesh.sample(N, return_index=False))

            ## Torchify point clouds
            #print("ground truth: \n",ground_truth)
            #print("length of ground truth: \n",len(ground_truth))
            gt_torch = torch.from_numpy(ground_truth).float().to(device)
            gt_torch = gt_torch.reshape(1, ground_truth.shape[0], ground_truth.shape[1])

            #when in order of p1x, p2x, ..., p1z, p2z, ...
            #result = np.array(output[i].tolist()).reshape(3,-1).T

            #when in order of p1x, p1y, p1z, ...
            result = np.array(output[i].tolist()).reshape(-1,3)
            #daniel_data.append(np.array(output[i].tolist()))

            ## Least Squares optimization to fit a backbone shape to point clouds
            gt_backbone, gt_end = identify_end_point(ground_truth)
            rs_backbone, rs_end = identify_end_point(result)

            #comparison_plotting(gt_backbone, rs_backbone, 1, end_point=False)
            #end_point_dist = np.linalg.norm(gt_backbone[-1] - rs_backbone[-1])
            end_point_dist = np.linalg.norm(gt_end - rs_end)
            #print("end point dist: ", end_point_dist, " m")
            if end_point_dist < 0.02:
                end_point_error.append(end_point_dist)

            #result = outlier_filter(result)

            if i == 23:# or i==5 or i==0 or i==25:
                with open('data/outlier_pc.pickle', 'wb') as outlier:
                    pickle.dump(result, outlier)
            #print("result: \n",result)
            #print("length of result: \n",len(result))
            #print("Current index: ", i+1)
            res_torch = torch.from_numpy(result).float().to(device)
            res_torch = res_torch.reshape(1, result.shape[0], result.shape[1])

            distance_error += chamferDist(gt_torch, res_torch).detach().cpu().item()
            distance = chamferDist(gt_torch, res_torch).detach().cpu().item()
            box_plot_data.append(distance)
            #print(f"\nAverage Chamfer distance up to {i+1}th configuration: \n{distance_error / (i+1)} (m)")

            
        else:
            ground_truth = np.array(robot.forward_kinematics(test_dataset[i, :4].tolist()))
            result = test_fk(output[i].tolist(), L, ground_truth)
            distance_error += np.linalg.norm(ground_truth - result, axis=1).max()
            print(f"\nAverage maximum L2 distance up to {i+1}th configuration: \n{distance_error / (i+1)} (m), \n{100 * distance_error / (i+1)} (cm), \n{1000*distance_error/ (i+1)} (mm)")
        ##Add Loss here
        #if i == 0 or i==1 or i==2 or i==3 or i==4:
        comparison_plotting(ground_truth, result, i, box_plot_data, metric='dist')
    if trajectory:
        test_plot(output, test_dataset)

    
    #print("Storing data...")
    #fff = open('data/non_hysteresis_point_cloud_data.pickle','wb')
    #pickle.dump(np.array(daniel_data), fff)
    #fff.close()

    error = np.array(end_point_error)
    print("error array size: ", error.shape)
    print("End-point error Mean and Std: ", error.mean(), error.std())
    box_plot(box_plot_data)
    #with open('data/box_plot_traj_nonhys.pickle', 'wb') as box:
    #    pickle.dump(box_plot_data, box)
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
        print("input size of the network: ", self.input)
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


def outlier_filter(data, k=3, r=0.0035): #r=0.0045 
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

    base = np.array([0,0,0])
    pc = outlier_filter(pc, r=0.001)
    #ls_backbone = LS.compare(test_dataset.tolist(), robot, i, hys=hys)
    poly = LS.polynomial_regression3d(pc[:,0], pc[:,1], pc[:,2], 3)
    idx = np.linspace(0, len(pc)-1, num=41, endpoint=True, dtype=int)
    backbone = poly[idx]
    #backbone = backbone[backbone[:,2].argsort()]
    eb = backbone[-1]

    dist = np.linalg.norm(base-pc, axis=1)
    end = pc[dist.argsort()[-1]]
    #print("optimized backbone points: \n",ls_backbone)
    return backbone, end

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Only from home position (input size: 4):
        Weight: data/actual_decoder_model_weights_3.pth
        pickle: data/hysteresis_integrated_input_four.pickle
        
        Only from home position (input size: 8):
        Weight: data/data_fixed_first.pth
        pickle: data/data_fixed_first.pickle
        test: data/fixed_home_30_test_cases.pickle


        Both home position and random position 1: 

        Weight - data/hysteresis_integrated.pth
        pickle - data/hysteresis_integrated.pickle
        test - data/hysteresis_integrated_idx.pickle
        '''
    #parser.add_argument('csv', nargs='?', default='pbf_data.csv')
    parser.add_argument('weight', default='data/actual_decoder_model_weights_home_hys.pth')
    parser.add_argument('pickle', default='data/home_data_from_olivia.pickle')
    parser.add_argument('test', default=None)
    parser.add_argument('-N','--num-samples', type=int, default=10)
    parser.add_argument('-H','--hys', type=int, default=0)
    return parser


def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    config_compare(
        num_samples=args.num_samples,
        model_path=args.weight,
        file_name=args.pickle,
        idx= args.test,
        hys=args.hys,
    )
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
