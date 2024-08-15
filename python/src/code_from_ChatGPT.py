"""
If you want the computed space curve to represent the backbone shape of the point set and not pass through all the points, you can use a non-parametric regression technique such as Gaussian Process Regression (GPR). GPR models a curve that is a probabilistic function of the input data and can capture complex relationships between inputs and outputs.

Here's an example code in Python using the GPy library to fit a Gaussian Process Regression model to 3D point clouds:

Code..

In this code, the RBF (Radial Basis Function) kernel is used to define the Gaussian Process Regression model. The optimize function is used to fit the model to the data. The predict function is used to generate the mean and variance of the model at a set of evaluation points. Finally, the plot_surface function from matplotlib is used to plot the fitted curve as a 3D surface, where the color of the surface is determined by the mean of the model. The cm.coolwarm colormap from matplotlib is used to color the surface.

"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import GPy
import numpy as np

# Define your set of 3D points
points = np.array([[0,0,0], [1,1,1], [2,0,2], [3,1,1]])

# Fit a Gaussian Process Regression model to the data
kernel = GPy.kern.RBF(input_dim=3, ARD=True)
model = GPy.models.GPRegression(points, np.zeros((points.shape[0], 1)), kernel)
model.optimize()

# Evaluate the GPR model at a set of points to generate the fitted curve
x_min, x_max = points[:,0].min(), points[:,0].max()
y_min, y_max = points[:,1].min(), points[:,1].max()
z_min, z_max = points[:,2].min(), points[:,2].max()
x_eval = np.linspace(x_min, x_max, num=100)
y_eval = np.linspace(y_min, y_max, num=100)
z_eval = np.linspace(z_min, z_max, num=100)
X, Y, Z = np.meshgrid(x_eval, y_eval, z_eval)
inputs = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T
mean, var = model.predict(inputs)

# Plot the 3D points and the fitted curve
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:,0], points[:,1], points[:,2], c='r', marker='o')
ax.plot_surface(X, Y, Z.reshape(X.shape), facecolors=plt.cm.coolwarm(mean/mean.max()))
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()



##2

import numpy as np
    ...: import matplotlib.pyplot as plt
    ...: from scipy.interpolate import BSpline
    ...:
    ...: points = np.random.rand(1000, 3)
    ...: knots = np.linspace(0, 1, points.shape[0] + 3)
    ...: degree = 3
    ...: c = points.flatten()
    ...: bspline = BSpline(knots, c, degree)
    ...:
    ...: u = np.linspace(0, 1, num=100)
    ...: spline = np.zeros((u.shape[0], 3))
    ...: for i in range(u.shape[0]):
    ...:     spline[i] = bspline(u[i])
    ...:
    ...: fig = plt.figure()
    ...: ax = fig.add_subplot(111, projection='3d')
    ...: ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='r')
    ...: ax.plot(spline[:, 0], spline[:, 1], spline[:, 2], color='b')
    ...: plt.show()

