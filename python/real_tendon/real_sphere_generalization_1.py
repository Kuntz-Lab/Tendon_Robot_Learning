import numpy as np
import pandas as pd
import matplotlib.pyplot as plt   #cos(theta1) sin(theta1) cos(theta2) sin(theta2) thetaDot1 thetaDot2
import argparse
import subprocess as subp
import sys
import toml


def populate_parser(parser = None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = "This is what I do!"
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    state = []
    action = []
    expert = []
    end_pose = []
    radius_list = []
    center_list = []
    for iter_k in range(10):
        problem_description = toml.load('phys_robot_limits.toml')
        r = np.random.uniform(0.04,0.06)
        problem_description['environment']['spheres'][0]['sphere']['center'][0] = np.random.uniform(-0.03, 0.03)
        problem_description['environment']['spheres'][0]['sphere']['center'][1] = -0.12
        problem_description['environment']['spheres'][0]['sphere']['center'][2] = np.random.uniform(0.05 , 0.09)
        problem_description['environment']['spheres'][0]['sphere']['radius'] = r
        output_path = "./phys_robot_limits.toml"
        with open(output_path, mode="w") as fh:
            toml.dump(problem_description, fh)
        o_1 = problem_description['environment']['spheres']
        start = [0, 0, 0, 0]
        center = [o_1[0]['sphere']['center'][0], o_1[0]['sphere']['center'][1], o_1[0]['sphere']['center'][2]]
        
        # points = [[center[0] + r, center[1], center[2]], [center[0] + r/(np.sqrt(2)), center[1], center[2] + r/(np.sqrt(2))], [center[0], center[1], center[2]+r], 
        # [center[0] - r/(np.sqrt(2)), center[1], center[2] + r/(np.sqrt(2))], [center[0] - r, center[1], center[2]], [center[0] - r/(np.sqrt(2)), center[1], center[2] + r/(np.sqrt(2))],
        # [center[0], center[1], center[2]+r], [center[0] + r/(np.sqrt(2)), center[1], center[2] + r/(np.sqrt(2))], [center[0] + r, center[1], center[2]]]

        points = [[center[0] + r, center[1], center[2]], [center[0] + r/(np.sqrt(2)), center[1], center[2] + r/(np.sqrt(2))], [center[0], center[1], center[2]+r], 
        [center[0] - r/(np.sqrt(2)), center[1], center[2] + r/(np.sqrt(2))]]

        
        pre_start = []
        noise = 0.001
        for i in range(len(points)):
            #print(points)
            subp.check_call(['./interactive_control',
                        '--robot', 'phys_robot_limits.toml',
                        '--start0', str(start[0]), 
                        '--start1', str(start[1]), 
                        '--start2', str(start[2]), 
                        '--start3', str(start[3]), 
                        '--des0', str(points[i][0]+ np.random.uniform(-noise,noise)), 
                        '--des1', str(points[i][1]+ np.random.uniform(-noise,noise)), 
                        '--des2', str(points[i][2]+ np.random.uniform(-noise,noise))])
            pre_start = start
            start = np.loadtxt('./interaction_ik.txt')
            expert.append(start)
            end_pose.append(points[i])
            radius_list.append(r)
            center_list.append([center[0], center[1], center[2]])
            if(i != 0):
                print(points[i-1])
                state.append([pre_start[0],pre_start[1],pre_start[2],pre_start[3]])
                action.append(start - pre_start)
    # np.savetxt('./csv/state_sphere_2_3.txt', state)
    # np.savetxt('./csv/action_sphere_2_3.txt', action)
    # np.savetxt('./csv/end_sphere_2_3.txt', end_pose)
    # pd.DataFrame(expert).to_csv("./csv1/sphere_50_real.csv")
    # np.savetxt('./csv1/sphere_50_real_expert.txt', end_pose)
    # np.savetxt('./csv1/sphere_50_real_r.txt', radius_list)
    # np.savetxt('./csv1/sphere_50_real_center.txt', center_list)
    pd.DataFrame(expert).to_csv("./csv1/test_sphere_10_real.csv")
    np.savetxt('./csv1/test_sphere_10_real_expert.txt', end_pose)
    np.savetxt('./csv1/test_sphere_10_real_r.txt', radius_list)
    np.savetxt('./csv1/test_sphere_10_real_center.txt', center_list)
    



    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))




