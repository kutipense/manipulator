import numpy as np

from utils import rotX,rotY,rotZ
from utils import orientation
from Quaternions import *

class Joint():
    def __init__(self, parent, serial, params):
        self.parent = parent

        self.serial = serial

        for i,j in params.items():
            setattr(self, i, j)

    @property
    def rotationQuaternion(self):
        axis = [1 if self.axis == i else 0 for i in range(3)]
        angle = self.angle
        translation = [self.lengthX,self.lengthY,self.lengthZ]

        rotation_quat = Quaternion.from_axis_angle(axis,angle)
        dual_quat = DualQuaternion.from_quaternion_vector(rotation_quat,translation)

        return dual_quat

    @property
    def orientation(self):
        return Quaternion.quaternion_to_euler_angle(self._getLocationQuaternion().real)

    @property
    def location(self):
        quat = self._getLocationQuaternion()
        return quat * ~quat

    def rotate(self, deg, time = 500):
        rot = deg*(self.degree90-self.degree0)/90 + self.degree0
        self.serial.write("#%d P %d T %d \r" %(self.port, rot, time))

        self.angle = to_rad(deg)

    def _getLocationQuaternion(self):
        loc = DualQuaternion.I()

        if self.parent != None:
            loc = self.parent._getLocationQuaternion()
            loc = loc * self.parent.rotationQuaternion

        return loc

