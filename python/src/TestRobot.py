import numpy as np
from TendonRobot import TendonRobotInterface
from CosseratRod import CosseratRodModel
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

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
    

def PlotBackbone(points):
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter3D(points[0,:], points[1,:], points[2,:], c='r', s=1)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_xlim([-0.2, 0.2])
    ax.set_ylim([-0.2, 0.2])

    plt.show()

def main():

    robot = ContinuumRobot(0.2, 9.1e7, 0.0006, np.array([[0],[0],[-9.81]]), 1.25)
    model = CosseratRodModel(robot)
    tensions = np.array([10, 0, 0, 0])
    backbone = model.Solve(tensions)
    print(backbone)
    PlotBackbone(backbone)


if __name__ == "__main__":
    main()
