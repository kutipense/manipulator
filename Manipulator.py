#!-*- coding=utf-8 -*-
import numpy as np

from random import randint

from math import cos,sin

from numpy.linalg import norm, inv
from scipy.optimize import fsolve
from utils import rotX,rotY,rotZ
from Quaternions import *

class Manipulator():

    def __init__(self,joints, end_effector):
        self.__joints = joints
        self.__end_effector = end_effector
        self.__bounds = [joint.bounds for joint in self.__joints]
        #self.__jacobian = self._getJacobian()

    @property
    def location(self):
        return self.__end_effector.location

    @property
    def orientation(self):
        return self.__end_effector.orientation

    @property
    def angles(self):
        return np.array([joint.angle for joint in self.__joints])


    def random(self):
        for joint in self.__joints:
            min_ang, max_ang = joint.bounds
            joint.angle = randint(min_ang,max_ang)
            joint.rotate(joint.angle)

        return self.location

    def _estimateAngles(self,point3d,orientation):
        q1 = Quaternion.from_euler_angle(orientation)
        target = point3d + q1.rotateit([7.5,0,0])
        x,y,z = target
        print target
        def angles(a):
            a0,a1,a2 = a
            r1 = -(9.5*cos(a1) + 10.5*cos(a1 + a2))*cos(a0) - x
            r2 = -(9.5*cos(a1) + 10.5*cos(a1 + a2))*sin(a0) - y
            r3 = 9.5*sin(a1) + 10.5*sin(a1 + a2) - z
            return np.array([r1,r2,r3])

        return fsolve(angles,(0,pi*3/4,pi/2))


    def goto(self,point,ori):
        #point = np.array(point,dtype=np.float32)

        angles = self._estimateAngles(point, ori)
        ee_angle = 2*pi-angles[1]-angles[2]+ori[1]#-pi/2+angles[1]+angles[2]
        print ee_angle
        angles = np.append(angles,ee_angle)

        for joint,ang in zip(self.__joints,angles):
            joint.rotate(ang*180/pi)

        return self.location

    def _jac_f(self,arr):
        a0,a1,a2,a3 = arr
        jac = np.array(self.__jacobian(a0,a1,a2,a3))
        """#jacInv = inv(jac.T.dot(jac)+np.eye(4)*0.000001).dot(jac.T)
                                locQuat = self.__end_effector._getLocationQuaternion()
                                real = locQuat.real.vector
                                dual = (locQuat * ~locQuat).dual.vector
                                vec = np.concatenate((real,dual))"""
        return jac

    def _getJacobian(self):
        from math_utils import SymQuaternion
        from math_utils import SymDualQuaternion
        import sympy as sym

        q1 = SymQuaternion.from_axis_angle([0,0,1],0)
        q2 = SymQuaternion.from_axis_angle([0,1,0],1)
        q3 = SymQuaternion.from_axis_angle([0,1,0],2)
        q4 = SymQuaternion.from_axis_angle([0,1,0],3)
        da1 = SymDualQuaternion.from_quaternion_vector(q1,[0,0,0])
        da2 = SymDualQuaternion.from_quaternion_vector(q2,[-9.5,0,0])
        da3 = SymDualQuaternion.from_quaternion_vector(q3,[-10.5,0,0])
        da4 = SymDualQuaternion.from_quaternion_vector(q4,[-7.5,0,0])
        locationQuaternion = da1*da2*da3*da4
        real = locationQuaternion.real.vector.jacobian(sym.Matrix(["a0","a1","a2","a3"]))
        dual = locationQuaternion.dual.vector.jacobian(sym.Matrix(["a0","a1","a2","a3"]))
        jac = real.row_insert(4,dual)
        jac_f = sym.lambdify(["a0","a1","a2","a3"],jac)

        del SymQuaternion
        del SymDualQuaternion
        del sym

        return jac_f
