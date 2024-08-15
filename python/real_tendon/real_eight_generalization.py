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
    width_list = []
    height_list = []
    for iter_k in range(50):
        problem_description = toml.load('phys_robot_limits.toml')
        problem_description['environment']['spheres'][0]['sphere']['center'][0] = np.random.uniform(-0.02, 0.02)
        problem_description['environment']['spheres'][0]['sphere']['center'][1] = -0.02
        problem_description['environment']['spheres'][0]['sphere']['center'][2] = np.random.uniform(0.16 , 0.20)
        output_path = "./phys_robot_limits.toml"
        with open(output_path, mode="w") as fh:
            toml.dump(problem_description, fh)
        o_1 = problem_description['environment']['spheres']
        start = [0, 0, 0, 0] #[0, 11, 5.5, 7.6, 6.6]
        center = [o_1[0]['sphere']['center'][0], o_1[0]['sphere']['center'][1], o_1[0]['sphere']['center'][2]]
        width = np.random.uniform(0.02,0.04)
        height = np.random.uniform(0.02,0.04)
        # points = [[center[0] + 0.05, center[1], center[2]],[center[0] + 0.025, center[1], center[2] - 0.025], [center[0], center[1], center[2]], [center[0] - 0.025, center[1], center[2] + 0.025], [center[0] - 0.05, center[1], center[2]],
        #         [center[0] - 0.025, center[1], center[2] - 0.025], [center[0], center[1], center[2]], [center[0] + 0.025, center[1], center[2] + 0.025], [center[0] + 0.05, center[1], center[2]]]
        points = [[center[0] + width*2, center[1], center[2]],[center[0] + width, center[1], center[2] - height], [center[0], center[1], center[2]], [center[0] - width, center[1], center[2] + height], [center[0] - width*2, center[1], center[2]],
                [center[0] - width, center[1], center[2] - height], [center[0], center[1], center[2]], [center[0] + width, center[1], center[2] + height], [center[0] + 2*width, center[1], center[2]]]

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
                        '--des0', str(points[i][0] + np.random.uniform(-noise,noise)), 
                        '--des1', str(points[i][1] + np.random.uniform(-noise, noise)), 
                        '--des2', str(points[i][2] + np.random.uniform(-noise, noise))])
            pre_start = start
            start = np.loadtxt('./interaction_ik.txt')
            expert.append(start)
            end_pose.append(points[i])
            #radius_list.append(r)
            width_list.append(width)
            height_list.append(height)
            if(i != 0):
                print(points[i-1])
                state.append([pre_start[0],pre_start[1],pre_start[2],pre_start[3]])
                action.append(start - pre_start)
    #pd.DataFrame(expert).to_csv("./csv1/test.csv")
    pd.DataFrame(expert).to_csv("./csv1/history_eight_50_real.csv")
    # np.savetxt('./csv1/state_circle_1_100.txt', state)
    # np.savetxt('./csv1/action_circle_1_100.txt', action)
    np.savetxt('./csv1/history_eight_50_real_expert.txt', end_pose)
    np.savetxt('./csv1/history_eight_50_real_width.txt', width_list)
    np.savetxt('./csv1/history_eight_50_real_height.txt', height_list)
    # pd.DataFrame(expert).to_csv("./csv1/expert_circle_1_100.csv")
    



    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))




