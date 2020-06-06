#!/usr/bin/python
import numpy as np

from math import acos
from math import asin
from math import cos
from math import sin
from math import pi

"""
    !!WARNING!!

      Whole calculations depend on specified rotation matricies.
      Before using this package please change rotation matricies and \
      calculate Euler Angles (orientation function) again. 
      You can use "sympy" for fast and accurate results.

    !!ENDWARNING!!

    Utility tools for rotation, transformation and orientation calculations.

    __author__: Kutay YILMAZ
    __email__ : ktyylmz035@gmail.com
    __github__: kutipense
"""

class Geometry:

    @staticmethod
    def transform(x=0,y=0,z=0):
        arr2 = np.array(
             [
                [1,0,0,x],
                [0,1,0,y],
                [0,0,1,z],
                [0,0,0,1]
            ],
              dtype=np.float)
        return arr2

    @staticmethod
    def rotX(deg):
        c, s = cos(deg), sin(deg)

        arr1 = np.array(
            [
                [1,0,0,0],
                [0,c,-s,0],
                [0,s,c,0],
                [0,0,0,1]
            ],
              dtype=np.float)

        return arr1

    @staticmethod
    def rotY(deg):
        c, s = cos(deg), sin(deg)

        arr1 = np.array(
            [
                [c,0,s,0],
                [0,1,0,0],
                [-s,0,c,0],
                [0,0,0,1] 
            ],
              dtype=np.float)

        return arr1

    @staticmethod
    def rotZ(deg):
        c, s = cos(deg), sin(deg)

        arr1 = np.array(
            [
                [c,-s,0,0],
                [s,c,0,0],
                [0,0,1,0],
                [0,0,0,1]
            ],
              dtype=np.float)

        return arr1

    @staticmethod
    def orientation(rotMat):
        """
            x : roll
            y : pitch
            z : yaw

            RotX = [[1,       0,      0],
                    [0,  cos(x), sin(x)],
                    [0, -sin(x), cos(x)]]

            RotY = [[ cos(y), 0, sin(y)],
                    [      0, 1,      0],
                    [-sin(y), 0, cos(y)]]

            RotZ = [[ sin(z), cos(z), 0],
                    [-cos(z), sin(z), 0],
                    [      0,      0, 1]]

            Euler Angles = dot(RotZ, dot(RotY, RotX)) # dot product

            Rotation matrix:

              [[ sin(z)*cos(y), -sin(x)*sin(y)*sin(z) + cos(x)*cos(z), sin(x)*cos(z) + sin(y)*sin(z)*cos(x)]
               [-cos(y)*cos(z),  sin(x)*sin(y)*cos(z) + sin(z)*cos(x), sin(x)*sin(z) - sin(y)*cos(x)*cos(z)]
               [       -sin(y),                        -sin(x)*cos(y),                        cos(x)*cos(y)]]

            Rotation matrix equals to end-effector's rotation matrix:

              [[a00, a01, a02]
               [a10, a11, a12]
               [a20, a21, a22]]

            a20 = -sin(y)         -> y = asin(-a20)
            a21 = -sin(x)*cos(y)  -> x = asin(-a21/cos(y))
            a10 = -cos(y)*cos(z)  -> z = acos(-a10/cos(y))
        """
        y = asin(round(-rotMat[2][0],3))
        x = asin(round(-rotMat[2][1]/cos(y),3))
        z = acos(round(-rotMat[1][0]/cos(y),3))

        return np.array([x,y,z])
