# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import multiprocessing
import time
import keyboard
import numpy as np
# import urx
import time
import rtde_control
import ur_move
import parallel_robot
import epos_read
import threading
from datetime import datetime
import  os
import csv

EPOS = True
UR5E = False
KINE = True


fileHd = []
csv_writer = []
csv_vect = np.zeros(21,np.float32)

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


def process_epos(indata_epos, share_dict):
    k = 0
    timeinstance = time.time()
    eposall = epos_read.eposallclass()
    eposall.configall()
    
    while True:
        #   time used in each loop: 10 ms
        k += 1
        angles,digital_input = eposall.rdvoltage()
        share_dict['angles'] = angles
        share_dict['digin'] = digital_input
        
        if k % 1000 ==0:
            timeinstance0 = time.time()
            # print('in EPOS read, 1000 times use time {} s'.format(timeinstance0  - timeinstance))
            timeinstance = timeinstance0
        #     print('at {}th reads'.format(k), 'angles (rad) = ',angles, 'the digital input = ', eposall.digitalinput)
        
        if (keyboard.is_pressed('alt')):
            print('break the epos while loop')
            break
        # time.sleep(0.3)
        

def process_parallel_robot(indata_parallel_robot,share_dict):
    k = 0
    finishtime0 = 0.0
    finishtime1 = 0.0

    forw = parallel_robot.p_forward()
    while True:
        #   time used in each loop: 0.5 ~ 0.7 s
        
        k += 1
        #    angles (degree)
        forw.baseangles = share_dict['angles']

        forw.newton_calmu()
        forw.pose_cal()

        if forw.newton_calmu_done:
            share_dict['dcm'] = forw.dcm
            share_dict['shift'] = forw.shift * 0.001 #  turn unit form mm to m.
        # share_dict['euler'] = forw.euler
        else:
            print('newton iteration error')

        if k % 10 == 0:
            finishtime1 = time.time()
            deltt = finishtime1 - finishtime0
            finishtime0 = finishtime1
            # print('in 10 kine calculation, time = {} s'.format(deltt))
            # print("at the {}th kine calcu, handle  position = {} with time = {}, button_flg = {}".format(k,share_dict['shift'],deltt,share_dict['digin']))
            

        if (keyboard.is_pressed('alt')):
            print('break the kine while loop')
            break
        
        
def process_ur_move(indata_parallel_robot,share_dict):
    k=0
    timeinstance = time.time()
    urrobot = ur_move.ur_moveclass()

    urrobot.move2init()
    while True:
        #   time used in each loop: 15 ms
        k += 1
        urrobot.moveur(R_mr=share_dict['dcm'], shift_mr=share_dict['shift'], Enable=share_dict['digin'])
        share_dict['actualtcppose'] = urrobot.actual_pose_tcp_urbase
        share_dict['tcppose'] = urrobot.pose_tcp_urbase
        share_dict['hdpose'] = urrobot.pose_hd_urbase

        if k % 1000 == 0:
            timeinstance0 = time.time()
            # print('in urrobot read, 1000 times use time {} s'.format(timeinstance0 - timeinstance))
            timeinstance = timeinstance0
    
        if (keyboard.is_pressed('alt')):
            print('break the ur move while loop')
            urrobot.rtde_c.servoStop()
            break
    # urrobot.close()
    
def capTime(timeorstr = False):
    dt = datetime.now()
    if not timeorstr:
        data_str = 'data-'+str(dt.month).rjust(2,'0')+'-'+str(dt.day).rjust(2,'0')+'-'+str(dt.hour).rjust(2,'0')+'-'+str(dt.minute).rjust(2,'0')+'-'+str(dt.second).rjust(2,'0')
        return data_str
    else:
        return ((dt.hour * 60 + dt.minute) * 60 + dt.second) * 1000 + dt.microsecond * 0.001

def record(share_dict):
    fileHd = []
    csv_writer = []
    csv_vect = np.zeros(21, np.float32)
    time0 = capTime(True)

    datastr = capTime(False)
    os.mkdir(datastr + '/')

    while True:
        

        csv_vect[0] = share_dict['record_cnt']
        csv_vect[1] = capTime(True) - time0
        csv_vect[2:8] = share_dict['angles']
        csv_vect[8] = share_dict['digin']
        csv_vect[9:15] = share_dict['hdpose']
        csv_vect[15:21] = share_dict['tcppose']
    
    
        if csv_vect[0] % 100000 == 1:
            
    
            if not fileHd == []:
                fileHd.close()
            cntstr = str(int(csv_vect[0] / 100000))
            cntTxtname = datastr  + '/' + cntstr.rjust(2, '0') + '.txt'
            fileHd = open(cntTxtname, 'w+', encoding='utf-8', newline='')
            csv_writer = csv.writer(fileHd, delimiter=" ")
        
            csv_writer.writerow('%_Col-0 cnt')
            csv_writer.writerow('%_Col-1 time (ms)')
            csv_writer.writerow('%_Col-2~8 mr angle degree (rad)')
            csv_writer.writerow('%_Col-8 digin')
            csv_writer.writerow('%_Col-9~15 handle pose')
            csv_writer.writerow('%_Col-15~21 ur pose cmd')

        share_dict['record_cnt'] += 1
        
        csv_writer.writerow(csv_vect)

    # record_timer = threading.Timer(0.01, record, args=(share_dict,))
    # record_timer.start()



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    
    share_dict = multiprocessing.Manager().dict()
    share_dict['angles'] = np.array([0,0,0,0,0,0],dtype=np.float32)
    share_dict['hdpose'] = np.array([0,0,0,0,0,0],dtype=np.float32)
    share_dict['dcm'] = np.zeros((3,3),dtype=np.float32)
    share_dict['shift'] = np.zeros(3,dtype=np.float32)
    # share_dict['euler'] = np.zeros(3,dtype=np.float32)
    share_dict['digin'] = 0
    share_dict['record_cnt'] = 1
    share_dict['tcppose'] = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
    

    
    # #
    indata_epos = 0
    indata_parallel_robot = 0
    indata_urrobot = 0
    # #
    p_epos = multiprocessing.Process(target=process_epos, args=(indata_epos, share_dict))
    p_parallel_robot = multiprocessing.Process(target=process_parallel_robot, args=(indata_parallel_robot,share_dict ))
    p_ur_move = multiprocessing.Process(target=process_ur_move, args=(indata_urrobot, share_dict))

    record_timer = threading.Timer(0.01, record, args=(share_dict, ))
    record_timer.start()
    

    if EPOS:
        p_epos.start()
    if KINE:
        p_parallel_robot.start()
    if UR5E:
        p_ur_move.start()
    
    #-------------------------------------------------
    
    # ur_move.ur5e.move2initj()
    # ur_move.ur5e.close()
    #-------------------------------------------------

    
    


    while (p_epos.is_alive() and EPOS ) or (KINE and p_parallel_robot.is_alive()):
        time.sleep(0.1)
    print('it is the time to kill the epos process')
    
    if EPOS:
        p_epos.kill()
    if KINE:
        p_parallel_robot.kill()
    if UR5E:
        p_ur_move.kill()
    
    print('end')
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
