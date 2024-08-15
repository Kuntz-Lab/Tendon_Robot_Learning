from mpl_toolkits import mplot3d
import argparse
import matplotlib.pyplot as plt
import numpy as np
import sys
import trimesh
import math

import cpptendon as T
import tempfile 
from scipy.spatial.transform import Rotation as R
from simulated_data_collection import quaternion_from_forward_and_up_vectors

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'A constant curvature kinematic model of a tendon-driven continuum robot with 1 straight tendon routed alongside its backbone.'
    #parser.add_argument('toml', nargs='?', default='sim_robot_limits.toml')
    #parser.add_argument('csv', nargs='?', default='data/pbf_data.csv')
    return parser

def backbone_plotting(backbone):
    backbone = np.array(backbone)
    #print(backbone)

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter3D(backbone[:,0], backbone[:,1], backbone[:,2], c='r', s=2)
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.21)

    plt.show()

def _point_cloud(backbone, robot_info, trans_mat, filename):
    #(L, backbone_pts, disk_r, num_tendon, num_disks)
    num_disk, disk_r = robot_info[-1], robot_info[2]
    if num_disk:
        _disk_mesh(backbone, num_disk, disk_r, trans_mat)        

    shape = T.collision.CapsuleSequence(backbone, 0.002)
    mesh = shape.to_mesh(n_circle_pts=16, add_spherical_caps=False)
    mesh.to_stl(filename , binary=True)
    
def _disk_mesh(backbone, num_disk, disk_r, trans_mat):
    # Disks attached along the backbone shape
    backbone = np.array(backbone)
    l = []
    disk_loc = np.array_split(backbone, num_disk)
    for i in range(len(disk_loc)):
        l.append(disk_loc[i][-1])
    vec = []
    idx = [4, 9, 14, 19, 24, 28, 32, 36, 40]
    for i in range(len(disk_loc)):
        #idx = np.where(backbone == l[i])[0][0]
        vec.append(backbone[idx[i]] - backbone[idx[i]-1])
    #disk_idx = np.arange(len(trans_mat))[5::5]
    #for i in disk_idx:
        #T = trans_mat[i].reshape(4,4).T
    for i in range(len(vec)):
        r = R.from_quat(quaternion_from_forward_and_up_vectors(vec[i], np.array([0,0,1])))
        rot_mat = r.as_matrix()
        r_size = len(rot_mat)
        T = np.zeros((r_size+1, r_size+1))
        T[0:r_size, 0:r_size] = rot_mat
        T[0:r_size, r_size] = l[i].ravel()
        T[r_size, r_size] = 1.0
        #print(T)
        cyl = trimesh.primitives.Cylinder(radius=disk_r-0.002, height=0.003, transform=T)
        cyl.export(f'data/disk{i}_cc.stl')

def get_robot_info():
    L = 0.2
    backbone_pts = 41
    #disk_r = 0.015
    disk_r = 0.00815
    #disk_r = 0.01085
    #beta = 2*np.pi/3
    num_tendon = 3
    num_disks = 9
    return (L, backbone_pts, disk_r, num_tendon, num_disks)

def robot_dependent_mapping(joint_value, robot_info):
    L, d = robot_info[0], robot_info[2] 
    beta = 2 * np.pi / 3
    
    if robot_info[3]==1:
        phi = 0
        for i,v in enumerate(joint_value):
            if v > 0 or v == 0:
                kappa = 0
            else:
                kappa = -v / (L * d)           
        print("kappa: ", kappa)
    elif robot_info[3]==2:
        if joint_value[0] <= 0:
            phi = 0
        else: 
            phi = np.pi
        kappa = np.abs((-joint_value[0] + joint_value[1]) / (d * (2*L + joint_value[0] + joint_value[1])))
        print("kappa: ", kappa)

    else:
        phi = np.arctan2(joint_value[0]* np.cos(beta)-joint_value[1], -joint_value[0]*np.sin(beta)) 
        kappa1 = -joint_value[0] / (d * np.cos(-phi) * L) 
        kappa2 = -joint_value[1] / (d * np.cos( beta - phi) * L) 
        kappa3 = -joint_value[2] / (0.0079 * np.cos(2 * beta - phi) * L) 

        print("phi: ", phi)
        print("kappas: ", kappa1, kappa2, kappa3)
        
        kappa = kappa2
        
    return kappa, phi


def robot_independent_mapping(kappa, phi, robot_info):
    #s = np.linspace(0, L, num_pts) 
    length, num_pts = robot_info[0], robot_info[1]

    c_p = np.cos(phi)
    s_p = np.sin(phi)
    T_base = np.eye(4)
    g = np.zeros((num_pts, 16))
    g_local = np.zeros((num_pts, 16))
    
    for j in range(num_pts):
        c_ks = np.cos(kappa * j * (length/ (num_pts-1) ))
        s_ks = np.sin(kappa * j * (length/ (num_pts-1) ))

        T_temp = np.array([
            [c_p * c_ks, -s_p, c_p * s_ks, 0],
            [s_p * c_ks, c_p, s_p * s_ks, 0],
            [-s_ks, 0, c_ks, 0],
            [0, 0, 0, 0]
        ])

        if kappa != 0:
            T_temp[:,3] = [c_p*(1-c_ks)/kappa, s_p*(1-c_ks)/kappa, s_ks/kappa, 1]
        else:
            T_temp[:,3] = [0, 0, j * (length / (num_pts-1) ), 1]
        g[j, :] = (T_base @ T_temp).T.reshape((1,16)) 

    return g

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    N = 10000 # the number of points consisting of the entire robot shape (point cloud)
    robot_info = get_robot_info() #(L, backbone_pts, disk_r, num_tendon, num_disks)

    tendon_range = np.array([-0.04, 0.01])
    control = np.random.uniform(low=tendon_range[0], high=tendon_range[1], size=(robot_info[3],)) # tendon displacement -0.04(fully pulled) ~ 0.01 (slack)
    if robot_info[3] == 2:
        control[1] = -control[0]
    elif robot_info[3] > 2:
        control[-1] = -np.sum(control[:-1])

    control = np.array([0, 0.01, -0.01])

    #control = np.clip(control, tendon_range[0], tendon_range[1])

    print("Current number of tendons routed along the robot length: ", robot_info[3], "\nPull the tendons by: ", control)
    kappa, phi = robot_dependent_mapping(control, robot_info)
    g = robot_independent_mapping(kappa, phi, robot_info)

    backbone = []
    for i in range(len(g)):
        backbone.append(np.array(g[i, -4:-1]))
    
    with tempfile.NamedTemporaryFile() as stlfile:
        _point_cloud(backbone, robot_info, g, stlfile.name)
        mesh = trimesh.load(stlfile.name, file_type='stl')
        for i in range(robot_info[-1]):
            disk = trimesh.load(f'data/disk{i}_cc.stl', file_type='stl')
            if not i:
                disk_pts = np.array(disk.sample(N, return_index=False))
            else:
                disk_pts = np.vstack((disk_pts, np.array(disk.sample(N, return_index=False))))
        pts = np.array(mesh.sample(N*4, return_index=False))
        pts = np.vstack((pts, disk_pts))
        pts = pts[np.random.choice(len(pts), size=N, replace=False)]

    backbone_plotting(pts)
    return 0 

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
