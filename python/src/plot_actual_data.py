from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pickle

import cpptendon as T


def plotting(point):
    idx = np.random.randint(0, len(point))
    print("index: ",idx)
    res = np.array(point[idx]).reshape(-1,3)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter3D(res[:,0], res[:,1], res[:,2], c='b', marker='X',s=3)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(0, 1)

    plt.show()

def main(filename='data/test_tendon_data.pickle'):
    with open(filename, 'rb') as fin:
        commands = pickle.load(fin)

    tau = [commands[i][:4] for i in range(len(commands))]
    pc = [commands[i][4:] for i in range(len(commands))]
    plotting(pc)

if __name__=="__main__":
    main()
