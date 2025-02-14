import numpy as np
import math
from scipy.spatial.transform import Rotation as R

G_z = 50 * 0.001
A_x = 39.105 * 0.001	#	distance of A to origin along X axis in base frame
A_y = 0 * 0.001
A_z = 67.74 * 0.001
lenAC = 196 * 0.001

class para_jacobs_class:
	def __init__(self,  Gz = G_z):

		self.angBias = 120 / 180 * math.pi

		self.rot_m2 = R.from_euler('y', self.angBias, degrees=False).as_matrix()
		self.rot_m3 = R.from_euler('y', self.angBias * 2, degrees=False).as_matrix()

		self.rot2 = np.delete(np.delete(self.rot_m2,1, axis=0), 1, axis=1)
		self.rot3 = np.delete(np.delete(self.rot_m3, 1, axis=0), 1, axis=1)

		G1_plf = np.array([0, 0, Gz])
		self.Gs_plf = np.array([G1_plf, np.dot(self.rot_m2, G1_plf), np.dot(self.rot_m3, G1_plf)], dtype=float)	#	position of G in mobile platform frame

		s6_1 = np.array([-1,0,0])
		self.s6s = np.array([s6_1, np.dot(self.rot_m2, s6_1), np.dot(self.rot_m3, s6_1)], dtype=float)

		self.As = np.zeros((3,3))
		self.Bs = np.zeros((3,3))

		self.dcm = np.eye(3)
		self.shift = np.zeros(3)
		self.Cs = np.zeros((3,3))
		self.Ds = np.zeros((3,3))
		self.Es = np.zeros((3,3))
		self.Fs = np.zeros((3,3))


	def jab_base1(self,EA,EB,EC,ED):
		self.EA = EA
		self.EB = EB
		self.EC = EC
		self.ED = ED
		
		#	refer to equation: 4-31
		anti_screw = np.array([[self.EC[0], self.EC[2]],[ self.ED[0], self.ED[2]]])
		right_2nd = np.array([[ - self.EC[0] * self.EA[2] + self.EA[0] * self.EC[2] , 0 ],[ 0 , - self.ED[0] * self.EB[2] + self.EB[0] * self.ED[2]  ]])
		jcb = np.dot(np.linalg.pinv(anti_screw), right_2nd)
		
		return jcb
	
	def base_jacobs(self, EAs, EBs, ECs, EDs ):
		#	the jacob matrix from angles to velocities of 3 F positions in base frame
		M_jacob = np.zeros((6,6),dtype=float)
		M_jacob[0:2, 0:2] = self.jab_base1(EAs[0], EBs[0], ECs[0], EDs[0])
		M_jacob[2:4, 2:4] = np.dot(self.rot2, self.jab_base1(EAs[1], EBs[1], ECs[1], EDs[1]))
		M_jacob[4:6, 4:6] = np.dot(self.rot3, self.jab_base1(EAs[2], EBs[2], ECs[2], EDs[2]))

		return  M_jacob

	def jab_plf1(self,F,G,s6, subchain  = 0):
		'''
		:param F: 	position of point F in 3F-frame, translated from base frame
		:param G: 	position of point G in 3F-frame
		:param s6: 	vector of s6 in base frame
		:return:
		'''
		FG = G - F	#	s5 in equ 4-35
		s2 = np.array([0, 0, 1])
		s1 = np.array([1, 0, 0])
		if subchain == 1:
			s2 = np.dot(self.rot_m2, s2)
			s1 = np.dot(self.rot_m2, s1)
		elif subchain == 2:
			s2 = np.dot(self.rot_m3, s2)
			s1 = np.dot(self.rot_m3, s1)

		n_vir = np.cross(FG/np.linalg.norm(FG) , s6)
		sr11_temp = np.cross(n_vir, s2)
		sr11 = sr11_temp / np.linalg.norm(sr11_temp)
		sr21_temp = np.cross(n_vir, s1)
		sr21 = sr21_temp / np.linalg.norm(sr21_temp)

		#	equ 4-38
		l1 = np.hstack((np.cross(F, sr11), sr11))
		l2 = np.hstack((np.cross(F, sr21), sr21))

		#	left and right sides of equ 4-38
		subM_26 = np.vstack((l1,l2))
		subRatio_22 = np.array([[sr11[0], 0],[0, sr21[2]]])

		return subM_26, subRatio_22


	def plf_jacobs(self,Fs,Gs,s6s):
		sub_1 , subr_1 = self.jab_plf1(Fs[0], Gs[0], s6s[0], subchain=0)
		sub_2 , subr_2 = self.jab_plf1(Fs[1], Gs[1], s6s[1], subchain=1)
		sub_3 , subr_3 = self.jab_plf1(Fs[2], Gs[2], s6s[2], subchain=2)
		M = np.vstack((sub_1,sub_2))
		M = np.vstack((M,sub_3))

		r = np.zeros((6,6),dtype=float)
		r[0:2,0:2] = subr_1
		r[2:4, 2:4] = subr_2
		r[4:6, 4:6] = subr_3

		jacob_m = np.dot(np.linalg.pinv(M), r)

		return jacob_m

	def parallel_jacob(self):
		self.Es = self.Fs * 1.0
		self.Es[:,1] = 0
		EAs = self.As - self.Es
		EBs = self.Bs - self.Es
		ECs = self.Cs - self.Es
		EDs = self.Ds - self.Es

		Gs = np.dot(self.dcm,self.Gs_plf) + self.shift
		s6s = np.dot(self.dcm, self.s6s)


		base_jacob = self.base_jacobs(EAs, EBs, ECs, EDs)

		plf_jacob = self.plf_jacobs(self.Fs, Gs, s6s )

		jacob_marix = np.dot(base_jacob, plf_jacob)

		return jacob_marix

	
	
# jacob_cal = para_jacobs()

