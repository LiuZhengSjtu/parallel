import numpy as np
from dqrobotics import *
from numpy.lib.function_base import angle
from scipy.spatial.transform import Rotation as R
import math

'''
    give the pose of the platform as a quaternion Q_plf
    the point of G in plf frame is PosG = [0, 0, Gz]
    the point of F in plf frame is PosF = [0, 0, Gz] + [0, -FG*cos(mu), FG*sin(my)]
    so, the PosF in ref frame is Q_plf * PosF * conj(Q_plf)) + translation(plf). its y axial value is 0 , so we can get mu
    its x and z values determine the mr clutch angles
'''
class Parallel_IK_class:
    def __init__(self):
        A_x = 39.105  # distance of A to origin along X axis in base frame
        A_y = 0
        A_z = 67.74
        self.lenAC = 196
        self.lenCE = 166
        self.lenEF = 40
        angBias = 120
        self.G_z = 50  # in mobile platform
        self.lenFG = 264 - 60
        self.baseangles = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)

        self.A1_ref = np.array([-A_x, A_y, A_z])
        self.B1_ref = np.array([ A_x, A_y, A_z])
        self.angBias = angBias / 180 * math.pi
        self.rot_m2 = R.from_euler('y', self.angBias, degrees=False).as_matrix()
        self.rot_m3 = R.from_euler('y', self.angBias * 2, degrees=False).as_matrix()

        self.A_ref = np.vstack((  self.A1_ref, np.dot(self.rot_m2, self.A1_ref) , np.dot( self.rot_m3, self.A1_ref ) ))
        self.B_ref = np.vstack((self.B1_ref, np.dot(self.rot_m2, self.B1_ref), np.dot(self.rot_m3, self.B1_ref)))

        self.rot1_quater = DQ([1, 0, 0, 0])
        self.rot2_quater = DQ([ math.cos(self.angBias/2), 0, math.sin(self.angBias/2), 0 ])
        self.rot3_quater = DQ([math.cos(self.angBias / 2 *2 ), 0, math.sin(self.angBias / 2 *2 ), 0])
        self.rot_quater = [ self.rot1_quater, self.rot2_quater , self.rot3_quater]

        self.mu = np.zeros(3,dtype=np.float32)
        self.F_ref = np.zeros((3,3))
        self.E_ref = np.zeros((3,3))

    def cal_plf_pose(self, axis, theta, shift):
        theta = theta / 2.0
        quat = [math.cos(theta), math.sin(theta) * axis[0], math.sin(theta) * axis[1], math.sin(theta) * axis[2]]
        self.quat_dq = DQ(quat)
        self.shift = shift


    def get_mu_F(self,):

        #   vect O_{plf}G in ref
        G_ref = np.zeros((3,3))

        for i in range(3):
            G_ref[i] = np.array(vec3(  Ad( self.quat_dq * self.rot_quater[i] ,  DQ( [0.0, 0, self.G_z  ] )  )))
            self.mu[i] = math.acos( (self.shift[1] + G_ref[i][1] - self.lenEF) / self.lenFG )

            self.F_ref[i] = np.array(vec3(Ad(self.quat_dq * self.rot_quater[i], DQ( np.array([0.0, 0, self.G_z])   + np.array( [ 0, -self.lenFG * math.cos( self.mu[i] ), self.lenFG * math.sin( self.mu[i] ) ,  ]  ) )))) + np.array(self.shift)
            self.E_ref[i] = np.array(  [ self.F_ref[i][0],  0.0, self.F_ref[i][2]])
        print(f'end of get mu F')


    def get_mr_angles(self, pointa, pointb, pointe):
        #   return the angle at A and B. vertical to side AB is 0. clockwise is positive
        #   get angle <CAB, <DBA in three phases
        pointa = np.array( pointa )
        pointb = np.array( pointb )
        pointe = np.array( pointe )
        ae  = pointe - pointa
        be  = pointe - pointb
        ab =  pointb - pointa
        len_a_e = np.linalg.norm(ae)
        len_b_e = np.linalg.norm(be)

        angle_eab =  math.acos( np.dot(ab,ae)/( np.linalg.norm(ab) * np.linalg.norm(ae) ) )
        angle_eba =  math.acos( np.dot(-ab,be)/( np.linalg.norm(ab) * np.linalg.norm(be) ) )

        angle_cae = math.acos( ( self.lenAC**2 + len_a_e**2 - self.lenCE**2 )  / (2 * self.lenAC * len_a_e) )
        angle_dbe = math.acos((self.lenAC ** 2 + len_b_e ** 2 - self.lenCE**2) / (2 * self.lenAC * len_b_e))

        return math.pi * 0.5 - (angle_eab + angle_cae), (angle_eba + angle_dbe) -math.pi * 0.5

    def getthetas(self,):
        self.A_angles = np.zeros(3)
        self.B_angles = np.zeros(3)
        self.get_mu_F()
        for i in range(3):
            self.A_angles[i], self.B_angles[i] = self.get_mr_angles( pointa=self.A_ref[i], pointb=self.B_ref[i], pointe=self.E_ref[i] )





parallel_ik = Parallel_IK_class()
parallel_ik.cal_plf_pose(axis=[0.0, 1, 0], theta= 6 / 180 * math.pi, shift=[0.0, 190 , 0.0])
parallel_ik.getthetas()
print(f'A angles = {parallel_ik.A_angles}\nB angles = {parallel_ik.B_angles}\nE_ref = {parallel_ik.E_ref}')
print(f'ik')












