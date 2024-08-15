import abc
import numpy as np

class TendonRobotInterface(metaclass=abc.ABCMeta):
    @classmethod
    def __subclasshook__(cls, subclass):
        return (hasattr(subclass, 'youngs_modulus') and
                hasattr(subclass, 'shear_modulus') and
                hasattr(subclass, 'GetTendonPositions') and
                callable(subclass.GetTendonPositions) and
                hasattr(subclass, 'length') and
                hasattr(subclass, 'GetSecondMomentsOfArea') and
                callable(subclass.GetSecondMomentsOfArea) and
                hasattr(subclass, 'GetPStar') and
                callable(subclass.GetPStar) and
                hasattr(subclass, 'GetRStar') and
                callable(subclass.GetRStar) and 
                hasattr(subclass, 'GetVStar') and
                callable(subclass.GetVStar) and
                hasattr(subclass, 'GetUStar') and
                callable(subclass.GetUStar) and
                hasattr(subclass, 'rho') and
                hasattr(subclass, 'g') or
                NotImplemented)
    
    @abc.abstractmethod
    def GetTendonPositions(self, s: float) -> np.ndarray:
        """Returns an array of tendon positions at arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetTendonPositionDerivatives(self, s: float) -> np.ndarray:
        """Returns derivative of tendon positions at arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetTendonPositionSecondDerivatives(self, s: float) -> np.ndarray:
        """Returns second derivative of tendon positions at arc
        length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetCrossSectionArea(self, s: float):
        """Returns the cross sectional area of the robot at arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetSecondMomentsOfArea(self, s: float):
        """Returns the second moments of area about principal axes x and y
        at arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetPStar(self, s: float):
        """Returns the reference position of the backbone at arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetRStar(self, s: float):
        """Returns the reference orientation of the backbone at
        arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetVStar(self, s: float):
        """Returns the reference linear rate of change of the backbone
        at arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetVStarDot(self, s: float):
        """Returns the derivative of v*"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetUStar(self, s: float):
        """Returns the reference angular rate of change of the
        backbone at arc length s"""
        raise NotImplementedError
    
    @abc.abstractmethod
    def GetUStarDot(self, s: float):
        """Returns the derivative of u*"""
        raise NotImplementedError

