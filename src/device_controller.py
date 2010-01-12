import subprocess
import re
import time
import math
from data_manager import *
from RILCommonModules import *

# Device status alias
DEVICE_NOT_RESPONDING = 0
DEVICE_AVAILABLE = 1
DEVICE_MOVING = 2
DEVICE_IDLE = 3
# globals
_packet_loss = ''
EXIT_COND = True 
# may be changed later to set as time duration 
SMALL_DELAY = 3

def process_ping_output(output):
    global _packet_loss
    pct = re.findall(r"\d\%", output)
    if(pct):
        #print "%loss: ", pct[0]
        _packet_loss = pct[0]
    if (_packet_loss == '0%'):
        #print "Device Alive"
        return True
    if (_packet_loss == '100%' or  (not _packet_loss)): 
        print "Device Dead"
        return False
    _packet_loss = ''

class DeviceController():
    def __init__(self,  dm,  bdaddr,  task_period= TASK_PERIOD):
        self.datamgr_proxy = dm
        self.bdaddr = bdaddr
        self.task_start = 0
        self.task_period = task_period
        # shared status
        self.l2ping_ok = False
        self.task_selected = False
        self.task_is_rw = False
        self.task_started = Fasle
        self.task_pending = False
        self.task_done = False
        self.task_timedout = False
        self.at_task = False
        self.pose_available = False
        # Device status
        self.status = DEVICE_NOT_RESPONDING

    def L2PingOK(self):
        # [CodeMakeup] Check if l2ping exits!
        cmd = "/usr/bin/l2ping -c " + " 1 " + self.bdaddr
        subproc = subprocess.Popen([cmd, ],\
            stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
        stdout_value = subproc.communicate()
        #print '\t got l2ping stdout_value:', stdout_value[0]
        self.l2ping_ok = process_ping_output(stdout_value[0])
        return self.l2ping_ok 

    def TaskSelected(self):
        taskdict = self.datamgr_proxy.mSelectedTask
        taskid = eval(str(taskdict[SELECTED_TASK_ID]))
        status = str(taskdict[SELECTED_TASK_STATUS])
        print "From TaskDict got %i %s"  %(taskid,  status)
        if status is TASK_SELECTED:
            self.task_selected = True
        if taskid is RANDOM_WALK_TASK_ID:
            self.task_is_rw = True
        return self.task_selected
    
    def RandomWalkTask(self):
        return self.task_is_rw 

    def TaskPending(self):
        if self.task_selected and (not self.task_started):
            self.task_pending = True
            self.task_started = True
            return self.task_pending
        if  (not self.ArrivedAtTask()): # <<FixIt: RandomWalking case>>
            self.task_pending = True
        return self.task_pending

    def TaskDone(self):
        ''' Defn1: If Robot reached at Task and task_period passed
            Defn2: If Task is RW, Robot RW and task_period passed'''
        return self.task_done

    def TaskTimedOut(self):
        now = time.time()
        elaspsed = now - self.task_start
        if elasped > self.task_period:
            self.task_timedout = True
        return self.task_timedout

    def ArrivedAtTask(self):
        robot_x= self.datamgr_proxy.mRobotPose[ROBOT_POSE_X]
        robot_y= self.datamgr_proxy.mRobotPose[ROBOT_POSE_Y]
        st = self.datamgr_proxy.mSelectedTask[SELECTED_TASK_INFO]
        print "Selected TaskInfo"
        print st
        task_x = st[TASK_INFO_X]
        task_y = st[TASK_INFO_Y]
        dx = math.fabs(robot_x - task_x)
        dy = math.fabs(robot_y - task_y)
        print "ArrivedAtTask(): xdist: %f ydist: %f" %(dx, dy)
        pxdist = math.sqrt(dx * dx + dy * dy)
        if (pxdist < TASK_RADIUS) :
            self.at_task = True
            print "ArrivedAtTask(): Arrived within radius %.2f !\n" %pxdist
        return self.at_task


    def PoseAvailable(self):
        if len(self.datamgr_proxy.mRobotPose) > 0: # <<  Test >>
            self.pose_available = True
        return self.pose_available

    def RunDeviceUnavailableLoop(self):
        while self.status is DEVICE_NOT_RESPONDING:
            if self.L2PingOK():
                self.status = DEVICE_AVAILABLE # out loop
                #print "Switching to RunDeviceAvailableLoop()"
                self.RunDeviceAvailableLoop()
                break
            else: 
                print "@ RunDeviceUnavailableLoop"
                self.status = DEVICE_NOT_RESPONDING # stay here in loop
                time.sleep(SMALL_DELAY)

    def RunDeviceAvailableLoop(self):
        while self.status is DEVICE_AVAILABLE:
            if self.TaskSelected():
                self.task_start = time.time()
                self.status = DEVICE_MOVING
                self.RunDeviceMovingLoop() # go out of this loop
                break
            elif (not self.L2PingOK()):
                self.status = DEVICE_NOT_RESPONDING
                self.RunDeviceUnavailableLoop() # go out of this loop
                break
            else:
               self.status = DEVICE_AVAILABLE # stay in-loop
               print "@DEVICE_AVAILABLE loop"
               time.sleep(SMALL_DELAY)
   
    def RunDeviceMovingLoop(self):
        while self.status is DEVICE_MOVING:
            if TaskTimedOut():
                self.task_done = True
                self.status = DEVICE_AVAILABLE # go out of loop
                self.RunDeviceAvailableLoop()
                break 
            elif  self.task_is_rw:
                self.status = DEVICE_MOVING # stay in-loop
                # do random walking ...
            elif self.TaskPending():
                if (not self.PoseAvailable()) or self.ArrivedAtTask():
                    self.status = DEVICE_IDLE # go out of loop
                    self.RunDeviceIdleLoop()
                    break
                else :
                    self.status = DEVICE_MOVING # stay in-loop
                    # go to navigation routines for MoveToTarget
            elif (not self.L2PingOK()):
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                self.RunDeviceUnavailableLoop()
            else:
                print "@RunDeviceMovingLoop: Unexpected situation "

    def RunDeviceIdleLoop(self):
        while self.status is DEVICE_IDLE:
            if not self.L2PingOK():
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                self.RunDeviceUnavailabeLoop()
                break
            elif self.TaskTimedOut():
                self.status = DEVICE_AVAILABLE # go out of loop
                self.RunDeviceAvailabeLoop()
                break
            elif  self.TaskPending() and self.PoseAvailable():
                self.status = DEVICE_MOVING # go out of loop
                self.RunDeviceMovingLoop()
                break
            elif self.TaskPending() and self.ArrivedAtTask() : 
                # stay in loop
                self.status = DEVICE_IDLE
                time.sleep(SMALL_DELAY)
            else:
                print "@RunDeviceIdleLoop: Unexpected situation "

    def RunMainLoop(self):
        while EXIT_COND:
            self.RunDeviceUnavailableLoop()

# CODE++: parse based on robot-id
def get_config(config_file,  config):
    result = ' '
    f = open(config_file, 'r')
    data = f.read()
    lst = re.split(';', data)
    if (config == 'bdaddr'):
        result = lst[1]
    return result

def controller_main(data_mgr,  config_file):
        bdaddr = get_config(config_file,  'bdaddr')
        dc = DeviceController(data_mgr,  bdaddr)
        dc.RunMainLoop()
