import numpy as np
import math


class ikSolver:
    theta1 = (0,'z')
    theta2 = (0,'y')
    theta3 = (0,'x')

    R30 = np.empty(shape=(3,3))

    def __init__(self, t1 = 0,d1= 'z', t2 = 0, d2 = 'y', t3 = 0, d3 = 'x'):
        self.theta1 = (t1,d1)
        self.theta2 = (t2,d2)
        self.theta3 = (t3,d3)

    def rot(self,theta = (0,'x')):
        R = np.empty(shape=(3,3))
        ctheta = math.cos(theta[0])
        stheta = math.sin(theta[0])
        match theta[1]:
            case 'x':
                R = np.matrix(data=([1,0,0],[0,ctheta,-stheta],[0,stheta,ctheta]))
            case 'y':
                R = np.matrix(data=([ctheta,0,stheta],[0,1,0],[stheta,0,ctheta]))
            case 'z':
                R = np.matrix(data=([ctheta,-stheta,0],[stheta,ctheta,0],[0,0,1]))
        return R
    
    def calcFK_arg(self,t1 = (0,'z'),t2 = (0,'y'),t3 = (0,'x')):
        R30 = self.rot(t1) * self.rot(t2) * self.rot(t3)
        #print(R30)
        return R30
    
    def calcFK(self):
        R30 = self.rot(self.theta1) * self.rot(self.theta2) * self.rot(self.theta3)
        #print(R30)
        return R30

    def calcIK(self, R = np.empty(shape=(3,3))):
        beta_y = math.asin(R[2,0])
        cby = math.cos(beta_y)
        alpha_x = math.atan2(-R[2,1]/cby,R[2,2]/cby)
        gamma_z = math.atan2(-R[1,0]/cby,R[0,0]/cby)
        return (alpha_x,beta_y,gamma_z)

class mouse_calculator(ikSolver):
    delta_rx = 0
    delta_ry = 0
    delta_rz = 0
    
    def __init__(self):
        super().__init__()

    def update_mouse(self, x,y,z):
        self.delta_rx = x
        self.delta_ry = y
        self.delta_rz = z

    def delta_matrix(self):
        deltas = super().calcFK_arg(t1 = (self.delta_rz,'z'),t2 = (self.delta_ry,'y'),t3 = (self.delta_rx,'x'))
        return deltas

def conv2angle(a = (0,0,0)):
    return math.degrees(a[0]),math.degrees(a[1]),math.degrees(a[2])

tool = ikSolver(0,0,0) 
R = tool.calcFK_arg((math.pi/2,'z'),(math.pi/4,'y'),(math.pi/4,'x'))
angles = tool.calcIK(R)
mouse = mouse_calculator()
mouse.update_mouse(0.0872664626,0.0,0.0872664626)
R_delta = mouse.delta_matrix()
print(R*R_delta)
new_angles = tool.calcIK(R)
print(conv2angle(new_angles))
R = R*R_delta
new_angles = tool.calcIK(R)
print(conv2angle(new_angles))
R = R*R_delta
new_angles = tool.calcIK(R)
print(conv2angle(new_angles))
