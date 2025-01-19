
import numpy as np
import math
import copy
from scipy.spatial.transform import Rotation as R
import sympy as sp

'''
for parallel robot forward kinematic calculation. Newton-raphason iteration method.
input: angles of MRD-dampers in the base frame
output: pose of the mobile platform frame

first step: damper angles --> unknown angles of the mobile platform, mu1, mu2, mu3. Newton iteration
second step: orthogonal vectors.
'''


A_x = 39.105	#	distance of A to origin along X axis in base frame
A_y = 0
A_z = 67.74
lenAC = 196
lenCE = 166
lenEF = 40
angBias = 120
G_z = 50	#	in mobile platform
lenFG = 264 - 60
baseangles =  np.array([0,0,0,0,0,0],dtype=np.float32)
# baseangles =  np.array([-0.2,0.2,-0.2,0.2,-0.2,0.2],dtype=np.float32) * -1

mu1, mu2, mu3 = sp.symbols('mu1, mu2, mu3')



class p_forward:
	def __init__(self,A_x = A_x, A_y = A_y, A_z = A_z, lenAC = lenAC, lenCE = lenCE, lenEF = lenEF, angBias = angBias, G_z =G_z, lenFG = lenFG):
		self.posA = np.array([-A_x,A_y,A_z])
		self.lenAC = lenAC
		self.lenCE = lenCE
		self.lenEF = lenEF
		self.G_z = G_z
		self.angBias = angBias / 180 * math.pi
		self.lenFG = lenFG
		self.rot_m2 = R.from_euler('y',self.angBias, degrees= False).as_matrix()
		self.rot_m3 = R.from_euler('y', self.angBias*2, degrees=False).as_matrix()
		self.equaitonFF()
		self.baseangles = np.array([0,0,0,0,0,0],dtype=np.float32)
		self.mux =  np.array([0.5,0.5,0.5],dtype=np.float32)
		
		self.cnt = 0
		
		self.dcm = np.zeros((3,3),dtype=np.float32)
		self.shift = np.zeros(3,dtype=np.float32)
		self.euler = np.zeros(3,dtype=np.float32)
		
		self.newton_calmu_done = False



	#	calculate the coordinate of E or F in the sub-chain, based on the angles at A and B.
	#	in the base frame
	#	for angelA and angleB, if the link AC or BD wave to the right , the angle is positive.
	def ab2ef(self,angles, outputE = False):
		#	refer to 4.3.0.2
		angleA = angles[0]
		angleB = angles[1]
		lenE = math.sqrt(4*A_x**2 + 2 * lenAC**2 - 4*A_x * lenAC*(math.sin((angleA)) - math.sin((angleB)))
						 - 2 * lenAC**2 * math.cos((angleA-angleB)))		#	distance of CD
		lenE = np.clip(lenE,10,lenCE*2)
		tanECD = math.sqrt(4 * lenCE**2 / lenE**2 -1)
		
		E_x = 0.5 * lenAC *((math.sin((angleA)) + math.sin((angleB))) + tanECD * (math.cos((angleB)) - math.cos((angleA))) )
		E_z = A_z + 0.5 * lenAC *(math.cos((angleA)) + math.cos((angleB))) - 0.5 * tanECD * (2*A_x + lenAC * ( math.sin((angleB)) - math.sin((angleA)) ))
		
		self.posE = np.array([E_x, 0 , E_z])
		self.posF = copy.deepcopy(self.posE)
		self.posF[1] = self.lenEF
		if outputE:
			return self.posE
		else:
			return self.posF
		
	def lenff(self, angles = baseangles ):
		#	square of distance, F  , in base frame
		posF1 = self.ab2ef(angles[0:2])
		posF2 = np.dot(self.rot_m2 , self.ab2ef(angles[2:4]))
		posF3 = np.dot(self.rot_m3 , self.ab2ef(angles[4:6]))
		
		# print('position of F1: ',posF1)
		
		lenF1F2 = np.sum((posF1 - posF2) ** 2) # np.linalg.norm(posF1 - posF2)
		lenF2F3 = np.sum((posF3 - posF2) ** 2) # np.linalg.norm(posF3 - posF2)
		lenF1F3 = np.sum((posF1 - posF3) ** 2) # np.linalg.norm(posF1 - posF3)
		
		self.posF1 = posF1	#	in base frame
		self.posF2 = posF2
		self.posF3 = posF3
		self.lenFF = np.array([lenF1F2, lenF2F3, lenF1F3])
		# return  self.lenFF
	
	# def subequ(self,mux):
	# 	return np.array([0, -lenFG*sp.cos(math.radians(mux)), -lenFG * sp.sin(math.radians(mux)) + self.G_z])
	#
	# def equations(self,mu):
	# 	#	in mobile platform frame, the positions of F can be derived as:
	# 	mu1,mu2,mu3 = mu
	# 	F1x = 0
	# 	F1y = -lenFG*sp.cos(math.radians(mu1))
	# 	F1z = lenFG * sp.sin(math.radians(mu1)) + self.G_z
	#
	# 	F2x = -sp.sin(self.angBias) * ()
	# 	F2y = -lenFG * sp.cos(math.radians(mu1))
	# 	F2z = -lenFG * sp.sin(math.radians(mu1)) + self.G_z
	#
	#
	# 	F2 = self.rot_m2 * self.subequ(mu2)
	# 	F3 = self.rot_m3 * self.subequ(mu3)
	# 	eq1 = np.sum((F1 - F2) ** 2)
	# 	eq2 = np.sum((F3 - F2) ** 2)
	# 	eq3 = np.sum((F1 - F3) ** 2)
	
	
	def equaitonFF(self):
		#	equation for distance between F points. expressed by the symbol mu
		# mu1, mu2, mu3 = sp.symbols('mu1, mu2, mu3')
		F1x = 0
		F1y = -lenFG * sp.cos(mu1)
		F1z = lenFG * sp.sin(mu1) + self.G_z
		
		F2x = sp.sin(self.angBias) * (lenFG * sp.sin(mu2) + self.G_z)
		F2y = -lenFG * sp.cos(mu2)
		F2z =  sp.cos(self.angBias) * (lenFG * sp.sin(mu2) + self.G_z)
		
		F3x = sp.sin(2*self.angBias) * (lenFG * sp.sin(mu3) + self.G_z)
		F3y = -lenFG * sp.cos(mu3)
		F3z = sp.cos(2*self.angBias) * (lenFG * sp.sin(mu3) + self.G_z)
		
		eq1 = (F1x - F2x) ** 2 + (F1y - F2y) ** 2 + (F1z - F2z) ** 2
		eq2 = (F2x - F3x) ** 2 + (F3y - F2y) ** 2 + (F3z - F2z) ** 2
		eq3 = (F1x - F3x) ** 2 + (F1y - F3y) ** 2 + (F1z - F3z) ** 2
		
		eq1_dmu1 = sp.diff(eq1,mu1)
		eq1_dmu2 = sp.diff(eq1, mu2)
		eq1_dmu3 = sp.diff(eq1, mu3)
		eq2_dmu1 = sp.diff(eq2, mu1)
		eq2_dmu2 = sp.diff(eq2, mu2)
		eq2_dmu3 = sp.diff(eq2, mu3)
		eq3_dmu1 = sp.diff(eq3, mu1)
		eq3_dmu2 = sp.diff(eq3, mu2)
		eq3_dmu3 = sp.diff(eq3, mu3)
		
		self.equ_f = [F1x, F1y, F1z, F2x, F2y, F2z, F3x, F3y, F3z]
		self.equ_diff= [ eq1_dmu1,  eq1_dmu2, eq1_dmu3, eq2_dmu1 , eq2_dmu2 , eq2_dmu3, eq3_dmu1, eq3_dmu2, eq3_dmu3]

		
		eq3_dmu3example = float(self.equ_diff[8].evalf(subs={mu1:0.1, mu2:0.1, mu3:0.1}))
		print(eq3_dmu3example)
		
	def lenff_mu(self,mu):
		#	according to input mu, calculate the equation result, present the xyz of F
		#	in mobile platform frame
		F1x = 0
		F1y = float(self.equ_f[1].evalf(subs={mu1:mu[0]}))
		F1z = float(self.equ_f[2].evalf(subs={mu1: mu[0]}))
		posF1 = np.array([F1x, F1y,F1z])
		
		F2x = float(self.equ_f[3].evalf(subs={mu2: mu[1]}))
		F2y = float(self.equ_f[4].evalf(subs={mu2: mu[1]}))
		F2z = float(self.equ_f[5].evalf(subs={mu2: mu[1]}))
		posF2 = np.array([F2x, F2y, F2z])
		
		F3x = float(self.equ_f[6].evalf(subs={mu3: mu[2]}))
		F3y = float(self.equ_f[7].evalf(subs={mu3: mu[2]}))
		F3z = float(self.equ_f[8].evalf(subs={mu3: mu[2]}))
		posF3 = np.array([F3x, F3y, F3z])
		
		lenF1F2 = np.sum((posF1 - posF2) ** 2)  # np.linalg.norm(posF1 - posF2)
		lenF2F3 = np.sum((posF3 - posF2) ** 2)  # np.linalg.norm(posF3 - posF2)
		lenF1F3 = np.sum((posF1 - posF3) ** 2)  # np.linalg.norm(posF1 - posF3)
		
		#	the positions of F1, F2, F3 have consistent spatial relationship in mobiel platform frame and base frame.
		self.posF1_plf = posF1	#	the coordinate of F1 in mobile platform frame
		self.posF2_plf = posF2
		self.posF3_plf = posF3
		self.lenFF_mu = np.array([lenF1F2, lenF2F3, lenF1F3])
		
	
	def len_gra(self,mu):
		g = np.zeros((3,3),dtype=np.float32)
		g[0][0] =  float(self.equ_diff[0].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[0][1] = float(self.equ_diff[1].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[0][2] = float(self.equ_diff[2].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[1][0] = float(self.equ_diff[3].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[1][1] = float(self.equ_diff[4].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[1][2] = float(self.equ_diff[5].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[2][0] = float(self.equ_diff[6].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[2][1] = float(self.equ_diff[7].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		g[2][2] = float(self.equ_diff[8].evalf(subs={mu1: mu[0], mu2: mu[1], mu3: mu[2]}))
		self.gra = g
	
	def measure_angles(self):
		#	measure the angle of MRF-damper at the base platform, degree
		# baseangles = np.array([0,0,0,0,0,0],dtype=np.float32)
		baseangles = np.array([-1, 1, 0, 0, 0, 0], dtype=np.float32) * 5
		
		#	unit: degree
		return  baseangles
	
	def newton_calmu(self, former_mu = [0,0,0], max_iter = 20, tol = 0.02):
		#	calculate the unknown mu1, mu2, mu3 through newton-raphason iteration
		#	mux unit: rad
		self.newton_calmu_done = False
		self.cnt += 1
		ite_again = False
		
		#	calculate the distance between F based on the base angles.
		self.lenff(angles= self.baseangles)
		# mux = np.array([former_mu[0], former_mu[1], former_mu[2]],dtype=np.float32)
		mux   = self.mux * 1.0
		mux_f = self.mux * 1.0
		for _ in range(max_iter):
			self.lenff_mu(mux)  # based on mu values, calculate the distance square of F
			if np.allclose(self.lenFF_mu, self.lenFF, atol=tol):  # 如果函数值足够接近零，则认为已经找到解
				# print("at {:>5}".format(_), 'tolerance satisfied')
				# if self.cnt % 100 == 0:
				# 	print("at {:>5}".format(_), 'th iteration, tolerance satisfied,  the mux={:<50}'.format(np.array_str(mux)), 'with error=',
				# 		  np.sum(abs(self.lenFF_mu - self.lenFF)), '  self.cnt = ', self.cnt)
				self.mux = mux
				self.newton_calmu_done = True
				return mux
			self.len_gra(mux)  # 计算雅可比矩阵
			try:
				dmux = -np.linalg.inv(self.gra).dot(self.lenFF_mu - self.lenFF)  # 解线性方程组得到下一个估计值的修正量
			except np.linalg.LinAlgError:  # 如果雅可比矩阵不可逆，则无法继续迭代，返回当前估计值
				print("at {:>5}".format(_), 'singular gra')
				return mux
			mux += dmux * 0.75  # 更新估计值
			mux = np.clip(mux,0,1.57)
			
			if  np.sum(np.abs(mux - mux_f))  < 0.00001 and np.sum(np.abs(dmux)) > 100 :
				#	meaningless iteration. no practical update
				if not ite_again:
					ite_again = True
					mux = np.array([0.5,0.5,0.5],dtype=np.float32)
				else:
					self.mux = np.array([0.5, 0.5, 0.5], dtype=np.float32)
					return 0
			
			mux_f = mux * 1
			
		self.mux = np.array([0.5,0.5,0.5],dtype=np.float32)
		return 0
			# print('at ', _,'th iteration, the mux = ', mux, 'with totoal error =', np.sum(abs(self.lenFF_mu-self.lenFF)))
			# print("at {:>5}".format(_), 'th iteration, the mux={:<50}'.format(np.array_str(mux)), 'with error=',
			# 	  np.sum(abs(self.lenFF_mu - self.lenFF)))
		
	def pose_cal(self):
		#	according to the position of F1, F2, and F3 in mobile frame and base frame,
		#	calculate the unit vector, x-F1F2 z-vertical to F1F2F3
		#	in the mobile platform frame
		
		if self.newton_calmu_done:
			vector_x_plf = self.posF2_plf - self.posF1_plf
			vector_z_plf = np.cross(vector_x_plf, self.posF3_plf - self.posF1_plf)
			vector_y_plf = np.cross(vector_z_plf, vector_x_plf)
			uv_x_plf = vector_x_plf / np.linalg.norm(vector_x_plf)
			uv_z_plf = vector_z_plf / np.linalg.norm(vector_z_plf)
			uv_y_plf = vector_y_plf / np.linalg.norm(vector_y_plf)
			uvs_plf = np.vstack((uv_x_plf, uv_y_plf, uv_z_plf))
			
			#	in the base frame
			vector_x = self.posF2 - self.posF1
			vector_z = np.cross(vector_x, self.posF3 - self.posF1)
			vector_y = np.cross(vector_z, vector_x)
			uv_x = vector_x / np.linalg.norm(vector_x)
			uv_z = vector_z / np.linalg.norm(vector_z)
			uv_y = vector_y / np.linalg.norm(vector_y)
			uvs = np.vstack((uv_x, uv_y, uv_z))
			
			#	the dirction consine matrix from mobile platform frame to base frame: axes express in base frame
			M_orx = np.dot(uvs.T, uvs_plf)
			#	the position shift from mobile platform frame origin to base frame origin. expressed in base frame
			o_r_e = self.posF1 - np.dot(M_orx,self.posF1_plf)
			
			# rot = R.from_matrix(M_orx)
			# euler_angles = rot.as_euler('zxy',degrees=True)
			
			self.dcm = M_orx
			self.shift = o_r_e
			# self.euler = euler_angles
			
			# print("the 3*3 matrix is ",M_orx)
			# print('the 3*1 vector is ', o_r_e)
			# print('the euler angles are:',euler_angles)

# forwardcal = p_forward()