# import cv2
import csv
import os
import numpy as np
from datetime import datetime
from ctypes import *
import keyboard
import nidaqmx
import time
import copy
# from skimage import measure
import matplotlib.pyplot as plt
# import cv2.aruco as aruco
import glob
import math


myEpos = WinDLL('.\EposCmd64.dll')


class share_dict():
	def __init__(self):
		self.angles = np.zeros(6)
		self.dcm = np.zeros((3,3))
		self.origin_bias = np.zeros(3)


class eposclass():
	def __init__(self,id = 1):
		self.MotorHandle = []
		self.pErrorCode = c_uint()
		self.Motor_Device = create_string_buffer(b'EPOS2')
		self.Motor_Protocol = create_string_buffer(b'MAXON SERIAL V2')
		self.Motor_Protocol2 = create_string_buffer(b'CANopen')
		self.Motor_Interface = create_string_buffer(b'USB')
		self.Motor_Port = create_string_buffer(b'USB0')
		self.Motor_NodeID = id
		self.DigOut = c_uint()
		self.DigIn1 = c_long()
		self.AnaIn_mV1 = c_long()
		self.AnaIn_mV2 = c_long()

	def eposconfig(self,hd=[],id=1):
		#	Pin16-AnaIn1, Pin15-AnaIn2, Pin14-AnaGnd, Pin13-DigOut1, Pin8-DigIn1, Pin1-DGND
		if self.Motor_NodeID == id or True:
			self.MotorHandle = myEpos.VCS_OpenDevice(byref(self.Motor_Device), byref(self.Motor_Protocol),
												  byref(self.Motor_Interface),
												  byref(self.Motor_Port), byref(self.pErrorCode))
		else:
			self.MotorHandle = myEpos.VCS_OpenSubDevice(hd,byref(self.Motor_Device),byref(self.Motor_Protocol2),byref(self.pErrorCode))
		
		
		# self.MotorHandle = myEpos.VCS_OpenDeviceDlg(byref(self.pErrorCode))
		
		ret3 = myEpos.VCS_ResetDevice(self.MotorHandle, self.Motor_NodeID, byref(self.pErrorCode))
		
		# myEpos.VCS_AnalogInputConfiguration(self.MotorHandle, self.Motor_NodeID, 2, 15, True, byref(self.pErrorCode))
		print('Motor_NodeID=',self.Motor_NodeID,'  at the init step, the AnaIn is: ', self.AnaIn_mV1.value)
		myEpos.VCS_GetAnalogInput(self.MotorHandle, self.Motor_NodeID, 2, byref(self.AnaIn_mV2), byref(self.pErrorCode))
		myEpos.VCS_GetAnalogInputVoltage(self.MotorHandle, self.Motor_NodeID, 1, byref(self.AnaIn_mV1),
										 byref(self.pErrorCode))
		print('Motor_NodeID=',self.Motor_NodeID,'afeter AnaIn config, before DigOut config, the AnaIn2 is: ', self.AnaIn_mV2.value, '   and AnaIn1: ',
			  self.AnaIn_mV1.value)
		
		myEpos.VCS_DigitalOutputConfiguration(self.MotorHandle, self.Motor_NodeID, 1, 15, True, True, True,
											  byref(self.pErrorCode))
		# myEpos.VCS_DigitalOutputConfiguration(self.MotorHandle, self.Motor_NodeID, 2, 14, True, True, 0,
		# 									  byref(self.pErrorCode))
		myEpos.VCS_GetAllDigitalOutputs(self.MotorHandle, self.Motor_NodeID, byref(self.DigOut), byref(self.pErrorCode))
		print('Motor_NodeID=',self.Motor_NodeID,'at init step, the DigOut state is:{:X} '.format(self.DigOut.value))
		if self.DigOut.value > 0x8000:
			self.DigOut.value = self.DigOut.value & 0x7fff
		else:
			self.DigOut.value = self.DigOut.value | 0x8000
		# self.DigOut.value |= 0x4000
		
		myEpos.VCS_DigitalInputConfiguration(self.MotorHandle, self.Motor_NodeID, 1, 15, True, 0, 0,
											  byref(self.pErrorCode))
		
		print('Motor_NodeID=',self.Motor_NodeID,'then, DigOut stete is turned to : {:X}'.format(self.DigOut.value))
		myEpos.VCS_SetAllDigitalOutputs(self.MotorHandle, self.Motor_NodeID, self.DigOut, byref(self.pErrorCode))
		
		time.sleep(1)
		myEpos.VCS_GetAnalogInputVoltage(self.MotorHandle, self.Motor_NodeID, 2, byref(self.AnaIn_mV2), byref(self.pErrorCode))
		myEpos.VCS_GetAnalogInputVoltage(self.MotorHandle, self.Motor_NodeID, 1, byref(self.AnaIn_mV1),
										 byref(self.pErrorCode))
		myEpos.VCS_GetAllDigitalInputs(self.MotorHandle, self.Motor_NodeID,byref(self.DigIn1),byref(self.pErrorCode))
		
		print('Motor_NodeID=',self.Motor_NodeID,'after setting the DigOut, the AnaIn2 is: ', self.AnaIn_mV2.value, '  and AnaIn_mV1: ', self.AnaIn_mV1.value)


class eposallclass():
	def __init__(self):
		#	ana voltage of encoder in mV. the encoder number is not the same as the axis number
		self.voltages = np.array([0,0,0,0,0,0],dtype=np.float32)
		self.voltages_bias = np.array([2060, 748, 2002, 1032, 1288, 480], dtype=np.float32)
		#	angles of axis.
		self.angles = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
		
		self.digitalinput = 0
		
		self.ids = np.array([4,3,1],dtype=int)
		self.epos1 = eposclass(id=int(self.ids[0]))
		self.epos2 = eposclass(id=int(self.ids[1]))
		self.epos3 = eposclass(id=int(self.ids[2]))
	def configall(self):
		self.epos1.eposconfig(id=self.ids[0])
		self.epos2.eposconfig(hd=self.epos1.MotorHandle,id=self.ids[1])
		self.epos3.eposconfig(hd=self.epos1.MotorHandle,id=self.ids[2])
		
	def rdvoltage(self):
		time1 = time.time()
		#	ID = 4			label-6 . connected to usb
		myEpos.VCS_GetAnalogInputVoltage(self.epos1.MotorHandle, self.epos1.Motor_NodeID, 1, byref(self.epos1.AnaIn_mV1), byref(self.epos1.pErrorCode))
		self.voltages[0] = self.epos1.AnaIn_mV1.value
		myEpos.VCS_GetAnalogInputVoltage(self.epos1.MotorHandle, self.epos1.Motor_NodeID, 2, byref(self.epos1.AnaIn_mV2), byref(self.epos1.pErrorCode))
		self.voltages[5] = self.epos1.AnaIn_mV2.value
		myEpos.VCS_GetAllDigitalInputs(self.epos1.MotorHandle, self.epos1.Motor_NodeID,byref(self.epos1.DigIn1),byref(self.epos1.pErrorCode))
		if self.epos1.DigIn1.value & 0x8000 > 1:
			self.digitalinput = 1
		else:
			self.digitalinput = 0
		
		
		#	ID = 3			label-2
		myEpos.VCS_GetAnalogInputVoltage(self.epos2.MotorHandle, self.epos2.Motor_NodeID, 1, byref(self.epos2.AnaIn_mV1),
								  byref(self.epos2.pErrorCode))
		self.voltages[4] = self.epos2.AnaIn_mV1.value
		myEpos.VCS_GetAnalogInputVoltage(self.epos2.MotorHandle, self.epos2.Motor_NodeID, 2, byref(self.epos2.AnaIn_mV2),
								  byref(self.epos2.pErrorCode))
		self.voltages[3] = self.epos2.AnaIn_mV2.value
		
		#	ID = 1			label-3
		myEpos.VCS_GetAnalogInputVoltage(self.epos3.MotorHandle, self.epos3.Motor_NodeID, 1, byref(self.epos3.AnaIn_mV1),
								  byref(self.epos3.pErrorCode))
		self.voltages[2] = self.epos3.AnaIn_mV1.value
		myEpos.VCS_GetAnalogInputVoltage(self.epos3.MotorHandle, self.epos3.Motor_NodeID, 2, byref(self.epos3.AnaIn_mV2),
								  byref(self.epos3.pErrorCode))
		self.voltages[1] = self.epos3.AnaIn_mV2.value
		time2=time.time()
		# print('delt time = ',time2-time1)
		# print(self.voltages)
		
		#	the encoder number and the axis number are not aligned.
		#	in the master robot, encoder number is labeled by number. the axie numbe is labeled by number plus circle around it.
		
		self.angles = (self.voltages - self.voltages_bias) / 5000 * 360  * math.pi / 180

		
		return self.angles ,self.digitalinput


# eposall = eposallclass()

# def eposinitall():
# 	global eposall
# 	eposall.epos1.eposinit()
	
	
