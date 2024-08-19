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
import math
import pickle
import sys
import tempfile

import numpy as np
import trimesh

from cpptendon import collision, tendon
from scipy.spatial.transform import Rotation as R
from TendonRobot import TendonRobotInterface
from CosseratRod import CosseratRodModel

def simulated_data_collection(tomlfile='default.toml',
                              num_tensions=15,
                              data_storage='data/simulated_data_6.csv',
                              num_disk=9,
                              N=512):
    robot = tendon.TendonRobot.from_toml(tomlfile)
    with tempfile.NamedTemporaryFile() as stlfile:
        with open(data_storage, 'w') as store:
            writer = csv.writer(store)
            writer.writerow([f't{i+1}' for i in range(len(robot.tendons))]
                            + [f'p{i}{c}'for i in range(1, N+1) for c in 'xyz' ])
            config_space = [np.linspace(0.0, t.max_tension, num_tensions)
                            for t in robot.tendons]
            for config in itertools.product(*config_space):
                generate_point_cloud(config, robot, num_disk, stlfile.name)

                mesh = trimesh.load(stlfile.name, file_type='stl')
                for i in range(num_disk):
                    disk = trimesh.load(f'data/disk{i}_test.stl', file_type='stl')
                    if not i:
                        disk_pts = np.array(disk.sample(N, return_index=False))
                    else:
                        disk_pts = np.vstack((disk_pts, np.array(disk.sample(N, return_index=False))))
                pts = np.array(mesh.sample(N*4, return_index=False))
                pts = np.vstack((pts, disk_pts))
                pts = pts[np.random.choice(len(pts), size=N, replace=False)]
                writer.writerow(list(config) + list(pts.flatten()))

def generate_point_cloud(config, robot, num_disk, filename, backbone_r=0.002):

    #robot = ContinuumRobot(0.2, 9.1e7, 0.0006, np.array([[0],[0],[-9.81]]), 1.25)
    #model = CosseratRodModel(robot)
    #backbone = model.Solve(np.array(config))
    backbone = robot.forward_kinematics(config)
    if num_disk:
        generate_disk_mesh(robot, backbone, num_disk)
    shape = collision.CapsuleSequence(backbone, backbone_r)
    mesh = shape.to_mesh(n_circle_pts=16, add_spherical_caps=False)
    mesh.to_stl(filename, binary=True)

def generate_disk_mesh(robot, backbone, num_disk):
    # Disks attached along the backbone shape
    backbone = np.array(backbone)
    l = []
    disk_loc = np.array_split(backbone, num_disk)
    for i in range(len(disk_loc)):
        l.append(disk_loc[i][-1])
    vec = []
    for i in range(len(disk_loc)):
        idx = np.where(backbone == l[i])[0][0]
        vec.append(backbone[idx] - backbone[idx-1])
    for i in range(len(vec)):
        r = R.from_quat(quaternion_from_forward_and_up_vectors(vec[i], np.array([0,0,1])))
        rot_mat = r.as_matrix()
        r_size = len(rot_mat)
        T = np.zeros((r_size+1, r_size+1))
        T[0:r_size, 0:r_size] = rot_mat
        T[0:r_size, r_size] = l[i].ravel()
        T[r_size, r_size] = 1.0
        cyl = trimesh.primitives.Cylinder(radius=robot.r - 0.005, height=0.003, transform=T)
        cyl.export(f'data/disk{i}_test.stl')
    
def normalize(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm

def quaternion_from_forward_and_up_vectors(forward, up):
    """
    Inspired in the Unity LookRotation Quaternion function,
    https://answers.unity.com/questions/467614/what-is-the-source-code-of-quaternionlookrotation.html
    this returns a quaternion from two orthogonal vectors (x, y, z)
    representing forward and up.
    """
    v0 = normalize(forward)
    v1 = normalize(np.cross(normalize(up), v0))
    v2 = np.cross(v0, v1)
    m00, m01, m02 = v1
    m10, m11, m12 = v2
    m20, m21, m22 = v0

    num8 = (m00 + m11) + m22

    if num8 > 0.0:
        num = math.sqrt(num8 + 1.0)
        w = num * 0.5
        num = 0.5 / num
        x = (m12 - m21) * num
        y = (m20 - m02) * num
        z = (m01 - m10) * num
        return x, y, z, w

    if (m00 >= m11) and (m00 >= m22):
        num7 = math.sqrt(((1.0 + m00) - m11) - m22)
        num4 = 0.5 / num7
        x = 0.5 * num7
        y = (m01 + m10) * num4
        z = (m02 + m20) * num4
        w = (m12 - m21) * num4
        return x, y, z, w

    if m11 > m22:
        num6 = math.sqrt(((1.0 + m11) - m00) - m22)
        num3 = 0.5 / num6
        x = (m10 + m01) * num3
        y = 0.5 * num6
        z = (m21 + m12) * num3
        w = (m20 - m02) * num3
        return x, y, z, w

    num5 = math.sqrt(((1.0 + m22) - m00) - m11)
    num2 = 0.5 / num5
    x = (m20 + m02) * num2
    y = (m21 + m12) * num2
    z = 0.5 * num5
    w = (m01 - m10) * num2
    return x, y, z, w

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


def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'TODO'
    parser.add_argument('toml', nargs='?', default='default.toml')
    parser.add_argument('--num-tensions', type=int, default=21)
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    simulated_data_collection(
        tomlfile=args.toml,
        num_tensions=args.num_tensions,
    )
    return 0

if __name__=="__main__":
    sys.exit(main(sys.argv[1:]))
