import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import root
import matplotlib.pyplot as plt

E = 200e9
shear_modulus = 80e9
rad = 0.001
rho = 8000
g = np.array([[0],[0],[9.81]])
L = 0.5
num_tendons = 4

tendon_offset = 0.01506

def rot(theta):
    return tendon_offset * np.array([[np.cos(theta)],[np.sin(theta)],[0]])

r = np.array([rot(0), rot(np.pi/2), rot(np.pi), rot(np.pi*3/2)])

p0 = np.array([[0],[0],[0]])
R0 = np.array([[1,0,0],
               [0,1,0],
               [0,0,1]])

area = np.pi*(rad**2)
I = np.pi*(rad**4)/4
J = 2*I
Kse = np.array([[shear_modulus*area,0,0],
                [0,shear_modulus*area,0],
                [0,0,E*area]])
Kbt = np.array([[E*I,0,0],
                [0,E*I,0],
                [0,0,shear_modulus*J]])

tau = np.array([30,5,0,0])

init_guess = np.zeros(6)

def Hat(vec: np.ndarray) -> np.ndarray:
    """Hat operator"""
    return np.array([[0, -vec[2, 0], vec[1, 0]],
                        [vec[2, 0], 0, -vec[0, 0]],
                        [-vec[1, 0], vec[0, 0], 0]])

def CosseratTendonRobotOde(s, y):
    R = np.reshape(y[3:12], (3,3))
    v = np.reshape(y[12:15], (3,1))
    u = np.reshape(y[15:], (3,1))

    a = np.zeros((3,1))
    b = np.zeros((3,1))
    A = np.zeros((3,3))
    G = np.zeros((3,3))
    H = np.zeros((3,3))

    for i in range(num_tendons):
        pb_si = np.reshape(np.cross(np.reshape(u, 3), np.reshape(r[i],3)),(3,1)) + v
        pb_s_norm = np.linalg.norm(pb_si)
        pb_si_hat = Hat(pb_si)
        A_i = -(pb_si_hat @ pb_si_hat) * (tau[i]/(pb_s_norm**3))
        G_i = -A_i @ Hat(r[i])
        a_i = A_i @ np.reshape(np.cross(np.reshape(u,3),np.reshape(pb_si,3)),(3,1))

        a += a_i
        b += np.reshape(np.cross(np.reshape(r[i],3), np.reshape(a_i,3)),(3,1))
        A += A_i
        G += G_i
        H += Hat(r[i]) @ G_i
    
    K = np.zeros((6,6))
    K[:3,:3] = (A + Kse)[:,:]
    K[:3,3:] = G[:,:]
    K[3:,:3] = np.transpose(G, (1,0))[:,:]
    K[3:,3:] = (H + Kbt)[:,:]

    nb = Kse @ (v - np.array([[0],[0],[1]]))
    mb = Kbt @ u

    rhs = np.array([-np.reshape(np.cross(np.reshape(u,3),np.reshape(nb,3)),(3,1)) - np.transpose(R, (1,0)) @ (rho * area * g) - a, 
                    -np.reshape(np.cross(np.reshape(u,3),np.reshape(mb,3)),(3,1)) - np.reshape(np.cross(np.reshape(v,3),np.reshape(nb,3)),(3,1)) - b])
    
    rhs = np.reshape(rhs, (6,1))
    p_s = np.reshape(R @ v, 3)
    R_s = np.reshape(R @ Hat(u), 9)
    vs_and_us = np.reshape(np.linalg.inv(K) @ rhs, 6)

    y_s = np.concatenate((p_s, R_s, vs_and_us))
    return y_s

def ShootingFunction(guess):
    nb0 = np.reshape(guess[:3], (3,1))
    v0 = np.linalg.inv(Kse) @ nb0 + np.array([[0],[0],[1]])
    v0 = np.reshape(v0, 3)
    u0 = guess[3:]
    y0 = np.concatenate((np.reshape(p0, 3), np.reshape(R0, 9), v0, u0))

    vals = solve_ivp(CosseratTendonRobotOde, (0, L), y0).y

    vL = np.reshape(vals[12:15,-1], (3,1))
    uL = np.reshape(vals[15:,-1],(3,1))

    nbL = Kse @ (vL - np.array([[0],[0],[1]]))
    mbL = Kbt @ uL

    force_error = -nbL
    moment_error = -mbL
    for i in range(num_tendons):
        pb_si = np.reshape(np.cross(np.reshape(uL,3),np.reshape(r[i],3)),(3,1)) + vL
        Fb_i = -tau[i]*pb_si/np.linalg.norm(pb_si)
        force_error += Fb_i
        moment_error += np.reshape(np.cross(np.reshape(r[i],3), np.reshape(Fb_i,3)),(3,1))
    
    force_error = np.reshape(force_error, 3)
    moment_error = np.reshape(moment_error,3)
    print("Force Error:", force_error, "\tMoment Error:",moment_error)
    return np.concatenate((force_error, moment_error))



optim = root(ShootingFunction, init_guess, method='lm').x
n0 = np.reshape(optim[:3],(3,1))
v0 = np.linalg.inv(Kse) @ n0 + np.array([[0],[0],[1]])
v0 = np.reshape(v0,3)
init = np.concatenate((np.reshape(p0,3), np.reshape(R0,9), v0, optim[3:]))
backbone = solve_ivp(CosseratTendonRobotOde, (0,L), init, max_step=0.01).y

def PlotBackbone(points):
    fig = plt.figure()

    ax = fig.add_subplot(projection='3d')
    ax.scatter(points[0,:], points[1,:], points[2,:])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])

    plt.show()

PlotBackbone(backbone)
