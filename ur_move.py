
import urx
import numpy as np
from math import pi
import time
from scipy.linalg import logm
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import psutil
import os
import math
import copy
from scipy.spatial.transform import Rotation as R


def dcm_to_rotation_vector(dcm):
	# Compute the skew-symmetric matrix associated with the rotation vector
	skew_sym_matrix = logm(dcm)
	
	# Extract the rotation vector components from the skew-symmetric matrix
	rotation_vector = np.array([skew_sym_matrix[2, 1], skew_sym_matrix[0, 2], skew_sym_matrix[1, 0]])
	
	return rotation_vector


class ur_moveclass():
	def __init__(self):
		# self.robot = urx.Robot("192.168.3.101", use_rt=False)
		# self.robot.set_tcp((0, 0, 0, 0, 0, 0))
		# self.robot.set_payload(0, (0, 0, 0))
		# self.delt_move = np.array([0.005, 0.005, 0.005, 0.005, 0.005, 0.005])
		# self.initj = self.robot.getj()
		# self.init_pose = self.robot.get_pose()
		#
		# print('at the init, robot joints are ',self.initj, ' and init pose is ',self.init_pose)
		#
		self.predefine_j =  [0.0,-1.5708,1.5708,0.0,1.5708,0.0]
		
		self.pose_tcp_urbase = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.pose_tcp_urbase_pre =  np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.delt_pose_tcp_urbase = np.array([0.001, 0.001, 0.001, 0.018, 0.018, 0.018]) * 0.5
		self.actual_pose_tcp_urbase = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.pose_tcp_urbase_0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		# self.delt_pose_tcp_urbase = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		
		#	the master robot base frame and the ur5 ee frame are aligned.
		#	the origianl dcm of the predefiend ur5 ee frame in the ur base frame
		self.R_e_0 =R.from_euler('xz',[90, -90], degrees= True).as_matrix()
		self.pose_hd_urbase_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.pose_hd_urbase = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		# self.delt_pose_hd_urbase = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		
		self.tele_state = False
		
		# Parameters
		self.vel = 0.5
		self.acc = 0.5
		rtde_frequency = 500.0
		self.dt = 1.0 / rtde_frequency  # 2ms
		flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
		ur_cap_port = 50002
		self.robot_ip = "192.168.3.101"
		self.lookahead_time = 0.2
		self.gain = 100
		
		# ur_rtde realtime priorities
		rt_receive_priority = 90
		rt_control_priority = 85
		
		self.rtde_r = RTDEReceive(self.robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
		self.rtde_c = RTDEControl(self.robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
		
		process = psutil.Process(os.getpid())
		process.nice(psutil.REALTIME_PRIORITY_CLASS)
		
		self.init_pose = self.rtde_r.getActualTCPPose()
		print('at the init step, the pose is: ', self.init_pose)
		
		self.init_j = self.rtde_r.getActualQ()
		print('at the init step, the joints are: ', self.init_j)
		
		print('predefine joints are:', self.predefine_j)
		
		self.run_cnt = 0
		
	def move2init(self):
		print(type(self.predefine_j))
		self.rtde_c.servoJ(self.predefine_j, self.acc, self.vel, self.dt, self.lookahead_time, self.gain)
		time.sleep(2)
		print('to predefined j')
		testj = copy.copy(self.predefine_j)
		for i in range(2000):
			t_start = self.rtde_c.initPeriod()
			testj[5] = self.predefine_j[5] + 0.05 * math.sin((2 * math.pi  * i / 2000.0 ))
			self.rtde_c.servoJ(testj, self.acc, self.vel, self.dt, self.lookahead_time, self.gain)
			self.rtde_c.waitPeriod(t_start)
		time.sleep(0.2)
		self.rtde_c.servoStop()
		time.sleep(0.3)
		
		testpose = self.rtde_r.getActualTCPPose()
		testpose2 = 1.0 * testpose[2]
		for i in range(2000):
			t_start = self.rtde_c.initPeriod()
			testpose[2] = testpose2 + 0.05 * math.sin((2 * math.pi  * i / 2000.0 ))
			self.rtde_c.servoL(testpose, self.acc, self.vel, self.dt, self.lookahead_time, self.gain)
			self.rtde_c.waitPeriod(t_start)
		time.sleep(0.2)
		self.rtde_c.servoStop()
		
		
	def moveur(self, R_mr, shift_mr, Enable):
		'''
		:param R_mr: 	direction rotation matrix of the master robot handle to master robot base frames.
		:param Enable: 	flag of the Digital In1, indicates the state of the button.
		:return:
		'''
		
		#	 for the master robot handle, calculate the R in ur base frame (with origin in mr base), convert it to rotation vector.
		R_mr_ur = np.dot(self.R_e_0, R_mr)
		rv_mr_ur = dcm_to_rotation_vector(R_mr_ur)	#	1*3
		shift_mr_ur = np.dot(self.R_e_0, shift_mr)
		self.pose_hd_urbase = np.hstack((shift_mr_ur, rv_mr_ur))
		
		#	 the tcp pose, list[float] 1*6
		self.actual_pose_tcp_urbase = self.rtde_r.getActualTCPPose()
		
		if Enable and self.tele_state == False :
			self.pose_hd_urbase_0 = self.pose_hd_urbase
			self.pose_tcp_urbase_0 = np.array(self.actual_pose_tcp_urbase)
			self.pose_tcp_urbase_pre = self.pose_tcp_urbase_0
			self.tele_state = True
		elif Enable and self.tele_state == True :
			delt_pose = self.pose_hd_urbase - self.pose_hd_urbase_0
			# delt_pose[0:3] = np.array([0.0, 0.0, 0.0])
			self.pose_tcp_urbase = self.pose_tcp_urbase_0 + delt_pose
			self.pose_tcp_urbase = np.clip(self.pose_tcp_urbase,a_min=self.pose_tcp_urbase_pre - self.delt_pose_tcp_urbase, a_max= self.pose_tcp_urbase_pre + self.delt_pose_tcp_urbase)
			self.pose_tcp_urbase_pre = self.pose_tcp_urbase
			
			self.rtde_c.servoL(self.pose_tcp_urbase.tolist(), self.acc, self.vel, self.dt, self.lookahead_time, self.gain)
			# print('at {} ur_move, delt_pose={}'.format(self.run_cnt, delt_pose))
		else:
			self.rtde_c.servoStop()
			self.tele_state = False
			
			pass
		
		self.run_cnt += 1
		
		# if self.run_cnt % 100 == 0:
		# 	print('pose_hd_urbase = {} \n pose_tcp_urbase={} \n actual actual_pose_tcp_urbase = {} '.format(self.pose_hd_urbase, self.pose_tcp_urbase, self.actual_pose_tcp_urbase))
		




