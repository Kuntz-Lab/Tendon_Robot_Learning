import numpy as np
import matplotlib.pyplot as plt
import trimesh
from mpl_toolkits import mplot3d

if __name__=="__main__":

#mesh = trimesh.load('sequence_1.stl', file_type='stl')
#demo_pts = np.array(trimesh.sample.sample_surface_even(mesh, count=1024)[0])
#
#mesh = trimesh.load('trained_sequence_1.stl', file_type='stl'
#)
#trained_pts = np.array(trimesh.sample.sample_surface_even(mesh, count=1024)[0])

    for i in range(9):
        demo_mesh = trimesh.load(f'sequence_{i+1}.stl', file_type='stl')
        dpts = np.array(trimesh.sample.sample_surface_even(demo_mesh, count=1024)[0])
        #demo_pts = np.vstack((demo_pts, dpts))

        trained_mesh = trimesh.load(f'trained_sequence_{i+1}.stl', file_type='stl')
        tpts = np.array(trimesh.sample.sample_surface_even(trained_mesh, count=1024)[0])
        #trained_pts = np.vstack((trained_pts, tpts))

        fig = plt.figure(figsize=(5,3))
        
        ax = plt.axes(projection='3d')
        ax.scatter3D(dpts[:,0],dpts[:,1],dpts[:,2], color='blue', s=5)
        plt.axis('off')
        plt.grid(b=None)
        #ax.scatter3D(tpts[:,0],tpts[:,1],tpts[:,2], color='red')
        #plt.title("Trained(red) vs Demo(blue)")
    plt.show()

    #fig = plt.figure()
    #ax = plt.axes(projection='3d')
    #ax.scatter3D(demo_pts[:,0],demo_pts[:,1],demo_pts[:,2], color='blue')
    #ax.scatter3D(trained_pts[:,0],trained_pts[:,1],trained_pts[:,2], color='red')
    #plt.title("Trained(red) vs Demo(blue)")
    #plt.show()
