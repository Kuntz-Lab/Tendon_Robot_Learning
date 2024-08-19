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

from chamferdist import ChamferDistance
from mpl_toolkits import mplot3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch 
import torch.nn as nn
import torch.nn.functional as F
import trimesh
from scipy.spatial import Delaunay

from learn_tendon_pc import DeepDecoder
from test_config import identify_end_point
#from learn_actual_pc import DeepDecoder
from learn_tendon_shape import DeepNeuralNets, get_data
from simulated_data_collection import generate_point_cloud
import cpptendon as T

plt.rcParams.update({'font.size': 17})
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def test_plot(output):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    for i in range(len(output)):
        res = np.array(output[i].tolist()).reshape(-1,3)
        ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X',s=3)

        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(0, 1)

def comparison_plotting(gt, res, count):
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter3D(gt[:,0], gt[:,1], gt[:,2], c='r', s=1)
    ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X',s=3)
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.21)
    #ax.legend(['From config1', 'From config2'])
    plt.axis('off')
    plt.grid(b=None)

    #fig.savefig(f"../data/test_{count+1}.jpg")
    #for ii in range(0, 360, 36):
    #    ax.view_init(elev=10., azim=ii)
    #    fig.savefig(f"../data/learned_pc{ii}.jpg")

def config_compare(csvfile='pbf_data.csv',
                    num_samples = 30,
                    model_path='data/decoder_model_weights_with_disks.pth',
                    #model_path='data/test_actual_decoder_model_weights.pth',
                    sample_method='from_file',
                    tomlfile='../data/sim_robot_limits.toml',
                    pc_enable=True,
                    N=512,
                    num_disk=9,
                    idx_path=None,
                    actual=True,
                    normalize=True):
                    
    plt.gca().set_aspect('equal')
    robot = T.tendon.TendonRobot.from_toml(tomlfile)
    L = robot.specs.L

    #dnn = DeepNeuralNets()
    #dnn = DeepDecoder(N)
    #dnn.load_state_dict(torch.load(model_path))
    #dnn.to(device)

    ## Random samples for test
    if sample_method == 'random':
        #norm = [robot.tendons[i].max_tension/20 for i in range len(robot.tendons)] if normalize  else [[1.0] * 4]
        test_dataset = np.random.random_sample((num_samples, len(robot.tendons))) * robot.tendons[0].max_tension
        #for i in range(len(robot.tendons)):
        #    test_dataset[:, i] *= robot.tendons[i].max_tension
    ## Sample Test dataset from data collection
    elif sample_method == 'from_file':
        #with open('data/test_idx.pickle', 'rb') as ff:
        #    idx = pickle.load(ff)

        #file_name = 'data/data_fixed_first.pickle'
        #file_hysteresis = '../data/data_fixed_22.pickle'
        file_name = 'data/hys_quantification_1.pickle'
        file_hysteresis = 'data/hys_quantification_2.pickle'

        with open(file_name, 'rb') as fin:
            ## if data structure is a Python list
            commands = pickle.load(fin)

        with open(file_hysteresis, 'rb') as fin2:
            ## if data structure is a Python list
            hys_pc = pickle.load(fin2)

        #if idx_path is None:
        #    idx = np.random.choice(len(hys_pc), size=num_samples, replace=False)
        #else:
        #    with open(idx_path, "rb") as ff:
        #        idx = pickle.load(ff)
        #print("indices: ", idx)
        #test_dataset = list(commands[i] for i in idx)
        #hys_dataset = list(hys_pc[i] for i in idx ) 
        test_dataset = list(commands)
        hys_dataset = list(hys_pc) 
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
    hys_dataset = torch.from_numpy(np.array(hys_dataset)).float().to(device)

    ## Test
    #output = dnn(test_dataset[:,:4])
    print("Comparison between ground truth and learned point clouds\n")

    #test_plot(output)
    box_plot_data = []
    end_point_error = []
    distance_error = 0
    print("Len commands: ", len(commands))
    for i in range(len(commands)):
        if pc_enable:
            chamferDist = ChamferDistance()
            #generate_point_cloud(test_dataset[i, :4].tolist(), robot, num_disk, f'data/ground_truth_{i+1}.stl') 
            #mesh = trimesh.load(f'data/ground_truth_{i+1}.stl', file_type='stl')
            #if num_disk:
            #    for j in range(num_disk):
            #        disk = trimesh.load(f'data/disk{j}_test.stl', file_type='stl')
            #        if not j:
            #            disk_pts = np.array(disk.sample(N, return_index=False))
            #        else:
            #            disk_pts = np.vstack((disk_pts, np.array(disk.sample(N, return_index=False))))
            if actual:
                ground_truth = np.array(test_dataset[i, 8:].tolist()).reshape(-1,3)
                
                hys_truth = np.array(hys_dataset[i, 8:].tolist()).reshape(-1,3)
            else:
                if num_disk:
                    ground_truth = np.array(mesh.sample(4*N, return_index=False))
                    ground_truth = np.vstack((ground_truth, disk_pts))
                    ground_truth = ground_truth[np.random.choice(len(ground_truth), size=N, replace=False)]
                else:
                    ground_truth = np.array(mesh.sample(N, return_index=False))

            print("ground truth: \n",ground_truth)
            print("length of ground truth: \n",len(ground_truth))
            gt_torch = torch.from_numpy(ground_truth).float().to(device)
            gt_torch = gt_torch.reshape(1, ground_truth.shape[0], ground_truth.shape[1])

            #when in order of p1x, p2x, ..., p1z, p2z, ...
            #result = np.array(output[i].tolist()).reshape(3,-1).T

            #when in order of p1x, p1y, p1z, ...
            #result = np.array(output[i].tolist()).reshape(-1,3)
            result = np.array(robot.forward_kinematics(test_dataset[i, :4].tolist()))
            print("Tensions from home: ", test_dataset[i, :8].tolist())
            print("Tensions from rand: ", hys_dataset[i, :8].tolist())

            #print("result: \n",result)
            #print("length of result: \n",len(result))
            res_torch = torch.from_numpy(hys_truth).float().to(device)
            res_torch = res_torch.reshape(1, hys_truth.shape[0], hys_truth.shape[1])

            ## End-tip position error computation
            gt_backbone, gt_end = identify_end_point(ground_truth)
            rs_backbone, rs_end = identify_end_point(hys_truth)
            end_point_dist = np.linalg.norm(gt_backbone[-1] - rs_backbone[-1])
            end_point_error.append(end_point_dist)

            distance = chamferDist(gt_torch, res_torch).detach().cpu().item()
            distance_error += distance
            box_plot_data.append(distance)
            #distance_error += chamferDist(gt_torch, res_torch).detach().cpu().item()
            print(f"\nAverage Chamfer distance up to {i+1}th configuration: \n{distance_error / (i+1)} (m)")

            
        else:
            ground_truth = np.array(robot.forward_kinematics(test_dataset[i, :4].tolist()))
            result = test_fk(output[i].tolist(), L, ground_truth)

            print(f"\nAverage maximum L2 distance up to {i+1}th configuration: \n{distance_error / (i+1)} (m), \n{100 * distance_error / (i+1)} (cm), \n{1000*distance_error/ (i+1)} (mm)")
        ##Add Loss here
        if i % 100 == 0:
            comparison_plotting(ground_truth, hys_truth, i)
    error = np.array(end_point_error)
    print("Avg end-point error: ", error.mean(), error.std())
    box_plot_data = np.array(box_plot_data)
    print("Total length of Chamfer distance list: ", len(box_plot_data))
    print("Mean and Std of hysteresis error: ", box_plot_data.mean(), box_plot_data.std())
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

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'TODO'
    #parser.add_argument('csv', nargs='?', default='pbf_data.csv')
    #parser.add_argument('idx', nargs='?', default='pbf_data.csv')
    parser.add_argument('--num-samples', type=int, default=30)
    return parser

def compute_delaunay_tetra_circumcenters(dt):
    """
    Compute the centers of the circumscribing circle of each tetrahedron in the Delaunay triangulation.
    :param dt: the Delaunay triangulation
    :return: array of xyz points
    """
    simp_pts = dt.points[dt.simplices]
    # (n, 4, 3) array of tetrahedra points where simp_pts[i, j, :] holds the j'th 3D point (of four) of the i'th tetrahedron
    assert simp_pts.shape[1] == 4 and simp_pts.shape[2] == 3

    # finding the circumcenter (x, y, z) of a simplex defined by four points:
    # (x-x0)**2 + (y-y0)**2 + (z-z0)**2 = (x-x1)**2 + (y-y1)**2 + (z-z1)**2
    # (x-x0)**2 + (y-y0)**2 + (z-z0)**2 = (x-x2)**2 + (y-y2)**2 + (z-z2)**2
    # (x-x0)**2 + (y-y0)**2 + (z-z0)**2 = (x-x3)**2 + (y-y3)**2 + (z-z3)**2
    # becomes three linear equations (squares are canceled):
    # 2(x1-x0)*x + 2(y1-y0)*y + 2(z1-z0)*y = (x1**2 + y1**2 + z1**2) - (x0**2 + y0**2 + z0**2)
    # 2(x2-x0)*x + 2(y2-y0)*y + 2(z2-z0)*y = (x2**2 + y2**2 + z2**2) - (x0**2 + y0**2 + z0**2)
    # 2(x3-x0)*x + 2(y3-y0)*y + 2(z3-z0)*y = (x3**2 + y3**2 + z3**2) - (x0**2 + y0**2 + z0**2)

    # building the 3x3 matrix of the linear equations
    a = 2 * (simp_pts[:, 1, 0] - simp_pts[:, 0, 0])
    b = 2 * (simp_pts[:, 1, 1] - simp_pts[:, 0, 1])
    c = 2 * (simp_pts[:, 1, 2] - simp_pts[:, 0, 2])
    d = 2 * (simp_pts[:, 2, 0] - simp_pts[:, 0, 0])
    e = 2 * (simp_pts[:, 2, 1] - simp_pts[:, 0, 1])
    f = 2 * (simp_pts[:, 2, 2] - simp_pts[:, 0, 2])
    g = 2 * (simp_pts[:, 3, 0] - simp_pts[:, 0, 0])
    h = 2 * (simp_pts[:, 3, 1] - simp_pts[:, 0, 1])
    i = 2 * (simp_pts[:, 3, 2] - simp_pts[:, 0, 2])

    v1 = (simp_pts[:, 1, 0] ** 2 + simp_pts[:, 1, 1] ** 2 + simp_pts[:, 1, 2] ** 2) - (simp_pts[:, 0, 0] ** 2 + simp_pts[:, 0, 1] ** 2 + simp_pts[:, 0, 2] ** 2)
    v2 = (simp_pts[:, 2, 0] ** 2 + simp_pts[:, 2, 1] ** 2 + simp_pts[:, 2, 2] ** 2) - (simp_pts[:, 0, 0] ** 2 + simp_pts[:, 0, 1] ** 2 + simp_pts[:, 0, 2] ** 2)
    v3 = (simp_pts[:, 3, 0] ** 2 + simp_pts[:, 3, 1] ** 2 + simp_pts[:, 3, 2] ** 2) - (simp_pts[:, 0, 0] ** 2 + simp_pts[:, 0, 1] ** 2 + simp_pts[:, 0, 2] ** 2)

    # solve a 3x3 system by inversion (see https://en.wikipedia.org/wiki/Invertible_matrix#Inversion_of_3_%C3%97_3_matrices)
    A = e*i - f*h
    B = -(d*i - f*g)
    C = d*h - e*g
    D = -(b*i - c*h)
    E = a*i - c*g
    F = -(a*h - b*g)
    G = b*f - c*e
    H = -(a*f - c*d)
    I = a*e - b*d

    det = a*A + b*B + c*C

    # multiplying inv*[v1, v2, v3] to get solution point (x, y, z)
    x = (A*v1 + D*v2 + G*v3) / det
    y = (B*v1 + E*v2 + H*v3) / det
    z = (C*v1 + F*v2 + I*v3) / det

    return (np.vstack((x, y, z))).T

def compute_voronoi_vertices_and_edges(points, r_thresh=np.inf):
    """
    Compute (finite) Voronoi edges and vertices of a set of points.
    :param points: input points.
    :param r_thresh: radius value for filtering out vertices corresponding to
    Delaunay tetrahedrons with large radii of circumscribing sphere (alpha-shape condition).
    :return: array of xyz Voronoi vertex points and an edge list.
    """
    dt = Delaunay(points)
    xyz_centers = compute_delaunay_tetra_circumcenters(dt)

    # filtering tetrahedrons that have radius > thresh
    simp_pts_0 = dt.points[dt.simplices[:, 0]]
    radii = np.linalg.norm(xyz_centers - simp_pts_0, axis=1)
    is_in = radii < r_thresh

    # build an edge list from (filtered) tetrahedrons neighbor relations
    edge_lst = []
    for i in range(len(dt.neighbors)):
        if not is_in[i]:
            continue  # i is an outside tetra
        for j in dt.neighbors[i]:
            if j != -1 and is_in[j]:
                edge_lst.append((i, j))

    return xyz_centers, edge_lst

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    config_compare(
        num_samples=args.num_samples,
        #idx_path=args.idx,
    )
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
