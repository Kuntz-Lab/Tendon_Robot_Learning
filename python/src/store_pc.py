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

import numpy as np
import csv
import trimesh
import pickle

def csv_reader(csv_file):
    rows = []
    fields = []

    with open(csv_file, 'r') as f:
        csvreader = csv.reader(f)
        fields = next(csvreader)

        for row in csvreader:
            rows.append(row)

        ### With only tendon robot
        #indices = [fields.index(i) for i in ['i', 'tau_1', 'tau_2', 'tau_3', 'tau_4', 'tau_5']]

        ### With a collapsed lung env
        indices = [fields.index(i) for i in ['i', 'tau_1', 'tau_2', 'tau_3', 'theta', 's_start']]

    data = np.array([[float(row[i]) for i in indices] for row in rows])
    return data[:, 1:]

def main(method='bc'):
    sol = csv_reader('solution-lung-2.csv')
    data_storage = 'bc_lung_data_storage.pickle'
    pc = {}
    data = {}

    env = trimesh.load('env-mesh.stl', file_type='stl')
    env_pts = np.array(trimesh.sample.sample_surface_even(env, count=1024)[0])

    n_plan = len(sol)
    action_list = []
    for i in range(n_plan-1):
        action_list.append(sol[i+1] - sol[i])        
    action_array = np.array(action_list)

    for i in range(n_plan):
        mesh = trimesh.load(f'lung_sequence_{i+1}.stl', file_type='stl')
        pts = np.array(trimesh.sample.sample_surface_even(mesh, count=1024)[0])
        pts = np.vstack((pts, env_pts))
        pc[i] = pts
         
    for i in range(n_plan-1):
        if method == 'bco':
            data[i] = [action_array[i], pc[i], pc[i+1]]
        else:
            data[i] = [action_array[i], pc[i]]
            
    pickle.dump(data, open(data_storage, 'wb'))

if __name__=="__main__":
    method = 'bc'
    main(method=method)
