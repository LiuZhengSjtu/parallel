import numpy as np
import math




class para_jacobs:
	def __init__(self,EA,EB,EC,ED):
		self.EA = EA
		self.EB = EB
		self.EC = EC
		self.ED = ED
		self.angBias = 120 / 180 * math.pi
		
	def jab_base1(self,EA,EB,EC,ED):
		self.EA = EA
		self.EB = EB
		self.EC = EC
		self.ED = ED
		
		#	refer to equation: 4-31
		anti_screw = np.array([[self.EC[0], self.EC[2]],[ self.ED[0], self.ED[2]]])
		right_2nd = np.array([[ - self.EC[0] * self.EA[2] + self.EA[0] * self.EC[2] , 0 ],[ 0 , - self.ED[0] * self.EB[2] + self.EB[0] * self.ED[2]  ]])
		jcb = np.dot(np.linalg.inv(anti_screw), right_2nd)
		
		return jcb
	
	def base_jacobs(self, EA1, EB1, EC1, ED1, EA2, EB2, EC2, ED2, EA3, EB3, EC3, ED3):
		#	the jacob matrix from angles to velocities of 3 F positions, individual frame
		M_jacob = np.zeros((6,6),dtype=float)
		M_jacob[0:2][0:2] = self.jab_base1(EA1, EB1, EC1, ED1)
		M_jacob[2:4][2:4] = self.jab_base1(EA2, EB2, EC2, ED2)
		M_jacob[4:6][4:6] = self.jab_base1(EA3, EB3, EC3, ED3)
		
	def mobile_jacobs
	
	
	