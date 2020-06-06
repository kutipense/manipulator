from utils import *
from Joints import Joint
from Manipulator import Manipulator
import numpy as np
from random import randint
from numpy.linalg import norm
from scipy.optimize import minimize
from math import pi
from math_utils import *
#s = None
s = s_baglan()
s.baudrate = 115200

a = Joint(None,s,joint1)
b = Joint(a,s,joint2)
c = Joint(b,s,joint3)
d = Joint(c,s,joint4)
e = Joint(d,s,joint5)

joints = [a,b,c,d]
end_effector = e

m = Manipulator(joints,end_effector)
m.goto([10.5,0,2],[0,-pi/2,0])
#m._estimateAngles([10.5,0,2],[0,-pi/2,0])
#####################################################
q1 = Quaternion.from_axis_angle([0,0,1],0)
q2 = Quaternion.from_axis_angle([0,1,0],0)
q3 = Quaternion.from_axis_angle([0,1,0],0)
q4 = Quaternion.from_axis_angle([0,1,0],0)
da1 = DualQuaternion.from_quaternion_vector(q1,[0,0,0])
da2 = DualQuaternion.from_quaternion_vector(q2,[-9.5,0,0])
da3 = DualQuaternion.from_quaternion_vector(q3,[-10.5,0,0])
da4 = DualQuaternion.from_quaternion_vector(q4,[-7.5,0,0])
da = da1*da2*da3*da4
db = DualQuaternion.from_translation([0,0,0])
m.location
m._estimateAngles([10.5,0,2],[0,-np.pi/2,0])

ori=Quaternion.from_euler_angle([0,-pi/2,0])
target = DualQuaternion.from_quaternion_vector_rt_first(ori,[10.5,0,2])
target.rotateit(DualQuaternion.I())

"""
from math_utils import SymQuaternion
from math_utils import SymDualQuaternion
import sympy as sym

q1 = SymQuaternion.from_axis_angle([0,0,1],0)
q2 = SymQuaternion.from_axis_angle([0,1,0],1)
q3 = SymQuaternion.from_axis_angle([0,1,0],2)
#q4 = SymQuaternion.from_axis_angle([0,1,0],3)
da1 = SymDualQuaternion.from_quaternion_vector(q1,[0,0,0])
da2 = SymDualQuaternion.from_quaternion_vector(q2,[-9.5,0,0])
da3 = SymDualQuaternion.from_quaternion_vector(q3,[-10.5,0,0])
#da4 = SymDualQuaternion.from_quaternion_vector(q4,[-7.5,0,0])
da = da1*da2*da3#*da4
db = SymDualQuaternion.I()
real = da.real.vector.jacobian(sym.Matrix(["a0","a1","a2","a3"]))
dual = da.dual.vector.jacobian(sym.Matrix(["a0","a1","a2","a3"]))
jac = real.row_insert(4,dual)
jac_f = sym.lambdify(["a0","a1","a2","a3"],jac)
"""

"""

t = [  90.        ,   85.25852666,  110.        ]
p,k=[0],[0]
h,j =0,0
for i in t:
    h += 5*cos(i)
    j += 5*sin(i)
    p.append(h)
    k.append(j)
"""

"""from joints import Joint
from constants import *
#from random import randint
#import numpy as np
#from numpy.linalg import norm
from serial_move import *
#from scipy.optimize import minimize
from time import sleep

s = s_baglan()
s.baudrate = 115200

a = Joint(None,s,joint1)
b = Joint(a,s,joint2)
c = Joint(b,s,joint3)
d = Joint(c,s,joint4)
e = Joint(d,s,joint5)
print e.location

se = [(a,0,180),(b,90,140),(c,0,110),(d,0,110),(e,0,90)]

def rastgele():
    for i in range(50000):
        s = []
        flag = True
        while flag:
            for i,j,k in se:
                i.angle = randint(j,j+k)
            if e.location[2]>0:
                flag = False
        for i,_,_ in se:
            s.append(i.angle)
        s+=list(map(float,e.location.round(3)))[:-1]
        return s

def gd(target):
    options = {"maxiter": 1000}
    bounds = [(j,k) for _,j,k in se]
    initial_value = [i.angle for i,_,_ in se]
    def euclidian_distance(x):
        for i,j in zip(se,x):
            i[0].angle = j
        return norm(e.location[:-1]-target)

    res = minimize(euclidian_distance, initial_value, method='L-BFGS-B', bounds=bounds, options=options)
    return res

def go(x,y,z):
    ang = gd(np.array([x,y,z])).x
    for i,j in zip(se,ang):
        i[0].rotate(j)
    print e.location

def circle():
    i = -7.
    while i<7.0:
        z = (49-i*i)**0.5
        print z
        go(i,-15,8-z)
        i+=0.1
    i = 7
    while i>-7.0:
        z = (49-i*i)**0.5
        go(i,-15,8+z)
        i-=0.1
        
from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import numpy as np
  
def doloop():
    global depth, rgb
    while True:
        (depth,_), (rgb,_) = get_depth(), get_video()
        d3 = np.dstack((depth,depth,depth)).astype(np.uint8)
        da = np.hstack((d3,rgb))
        cv2.imshow('both',np.array(da[::2,::2,::-1]))
        cv2.waitKey(5)
doloop()

"""


