import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import root
from TendonRobot import TendonRobotInterface

class CosseratRodModel():
    def __init__(self, tendon_robot: TendonRobotInterface):
        self.tr = tendon_robot
    
    def GetShearExtensionMatrix(self, s: float) -> np.ndarray:
        """Gets K_se at arc length s of the robot"""
        area = self.tr.GetCrossSectionArea(s)
        val_1 = self.tr.shear_modulus*area
        val_2 = self.tr.youngs_modulus*area
        return np.array([[val_1,0,0],
                         [0,val_1,0],
                         [0,0,val_2]])
    
    def GetBendingTorsionMatrix(self, s: float) -> np.ndarray:
        """Gets K_bt at arc length s of the robot"""
        I_xx, I_yy = self.tr.GetSecondMomentsOfArea(s)
        ym = self.tr.youngs_modulus
        val_1 = ym*I_xx
        val_2 = ym*I_yy
        val_3 = ym*(I_xx + I_yy)
        return np.array([[val_1,0,0],
                         [0,val_2,0],
                         [0,0,val_3]])
    
    def GetPDotIB(self, u: np.ndarray, v: np.ndarray, s: float, i: int) -> np.ndarray:
        """Gets the derivative of tendon i in body frame coordinates at arc length s"""
        return np.reshape(np.cross(np.reshape(u,3), np.reshape(self.tr.GetTendonPositions(s)[i], 3)),(3,1)) + self.tr.GetTendonPositionDerivatives(s)[i] + v
    
    def Hat(vec: np.ndarray) -> np.ndarray:
        """Hat operator"""
        return np.array([[0, -vec[2, 0], vec[1, 0]],
                         [vec[2, 0], 0, -vec[0, 0]],
                         [-vec[1, 0], vec[0, 0], 0]])

    def RodOde(self, s, y):
        R = np.reshape(y[3:12], (3,3))
        v = np.reshape(y[12:15], (3,1))
        u = np.reshape(y[15:], (3,1))

        a = np.zeros((3,1))
        b = np.zeros((3,1))
        A = np.zeros((3,3))
        G = np.zeros((3,3))
        H = np.zeros((3,3))

        Kse = self.GetShearExtensionMatrix(s)
        Kbt = self.GetBendingTorsionMatrix(s)

        for i in range(self.tensions.shape[0]):
            r_i = self.tr.GetTendonPositions(s)[i]
            r_i_dot = self.tr.GetTendonPositionDerivatives(s)[i]
            r_i_dot_dot = self.tr.GetTendonPositionSecondDerivatives(s)[i]
            pb_si = self.GetPDotIB(u, v, s, i)
            pb_s_norm = np.linalg.norm(pb_si)
            pb_si_hat = CosseratRodModel.Hat(pb_si)
            A_i = -(pb_si_hat @ pb_si_hat) * (self.tensions[i]/(pb_s_norm**3))
            G_i = -A_i @ CosseratRodModel.Hat(r_i)
            a_i = A_i @ (np.reshape(np.cross(np.reshape(u,3),np.reshape(pb_si,3)),(3,1)) + np.reshape(np.cross(np.reshape(u,3),np.reshape(r_i_dot,3)),(3,1)) + r_i_dot_dot)

            a += a_i
            b += np.reshape(np.cross(np.reshape(r_i,3),np.reshape(a_i,3)),(3,1))
            A += A_i
            G += G_i
            H += CosseratRodModel.Hat(r_i) @ G_i

        K = np.zeros((6,6))
        K[:3,:3] = (A + Kse)[:,:]
        K[:3,3:] = G[:,:]
        K[3:,:3] = np.transpose(G, (1,0))[:,:]
        K[3:,3:] = (H + Kbt)[:,:]

        nb = Kse @ (v - self.tr.GetVStar(s))
        mb = Kbt @ (u - self.tr.GetUStar(s))
        u_star_dot = self.tr.GetUStarDot(s)
        v_star_dot = self.tr.GetVStarDot(s)
        rho = self.tr.rho
        g = self.tr.g
        area = self.tr.GetCrossSectionArea(s)

        rhs = np.array([Kse @ v_star_dot - np.reshape(np.cross(np.reshape(u,3),np.reshape(nb,3)),(3,1)) - np.transpose(R, (1,0)) @ (rho * area * g) - a,
                        Kbt @ u_star_dot - np.reshape(np.cross(np.reshape(u,3),np.reshape(mb,3)),(3,1)) - np.reshape(np.cross(np.reshape(v,3),np.reshape(nb,3)),(3,1)) - b])
        
        rhs = np.reshape(rhs, (6,1))
        p_s = np.reshape(R @ v,3)
        R_s = np.reshape(R @ CosseratRodModel.Hat(u), 9)
        vs_and_us = np.reshape(np.linalg.inv(K) @ rhs, 6)

        return np.concatenate((p_s, R_s, vs_and_us))

    def Objective(self, guess: np.ndarray) -> np.ndarray:
        """Compute loss """
        p0 = np.reshape(self.tr.GetPStar(0), 3)
        R0 = np.reshape(self.tr.GetRStar(0), 9)
        nb0 = np.reshape(guess[0:3], (3, 1))
        v0 = np.linalg.inv(self.GetShearExtensionMatrix(0)) @ nb0 + self.tr.GetVStar(0)
        v0 = np.reshape(v0, 3)
        u0 = guess[3:]
        y0 = np.concatenate([p0, R0, v0, u0])

        vals = solve_ivp(self.RodOde, (0, self.tr.length), y0).y

        vL = np.reshape(vals[12:15, -1], (3, 1))
        uL = np.reshape(vals[15:, -1], (3, 1))

        nbL = self.GetShearExtensionMatrix(self.tr.length) @ (vL - self.tr.GetVStar(self.tr.length))
        mbL = self.GetBendingTorsionMatrix(self.tr.length) @ (uL - self.tr.GetUStar(self.tr.length))

        force_error = -nbL
        moment_error = -mbL
        for i in range(self.tensions.shape[0]):
            r_i = self.tr.GetTendonPositions(self.tr.length)[i]
            pb_si = self.GetPDotIB(uL, vL, self.tr.length, i)
            Fb_i = -self.tensions[i] * pb_si/np.linalg.norm(pb_si)
            force_error += Fb_i
            moment_error += np.reshape(np.cross(np.reshape(r_i,3),np.reshape(Fb_i,3)),(3,1))

        force_error = np.reshape(force_error, 3)
        moment_error = np.reshape(moment_error, 3)
        print("force error:", force_error, "\tmoment error:", moment_error)
        return np.concatenate([force_error, moment_error])

    def Solve(self, tensions):
        self.tensions = tensions

        init = np.zeros(6)

        optim = root(self.Objective, init, method='lm', options={"ftol": 1e-12, "max_iter": 500, "factor": 0.5, "epsfcn": 1e-8}).x
        p0 = np.reshape(self.tr.GetPStar(0), 3)
        R0 = np.reshape(self.tr.GetRStar(0), 9)
        n0 = np.reshape(optim[:3],(3,1))
        v0 = np.linalg.inv(self.GetShearExtensionMatrix(0)) @ n0 + self.tr.GetVStar(0)
        v0 = np.reshape(v0,3)

        init_vals = np.concatenate([p0, R0, v0, optim[3:]])

        final_vals = solve_ivp(self.RodOde, (0, self.tr.length), init_vals, max_step=0.01).y
        return final_vals

    
    