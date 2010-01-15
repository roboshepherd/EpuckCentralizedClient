import subprocess
import re
import time
import math
import  logging,  logging.config,  logging.handlers
logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")


from myro import *
from RILCommonModules.RILSetup import *
from RILCommonModules.LiveGraph import *
from RILCommonModules.pose import *
from data_manager import *
from epuck_navigator import *

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
    def __init__(self,  dm,  bdaddr = 0,  task_period = TASK_PERIOD):
        self.datamgr_proxy = dm
        self.robotid = dm.mRobotID
        self.pose = Pose()
        self.bdaddr = bdaddr
        self.task_start = 0
        self.task_period = task_period
        # shared status
        self.l2ping_ok = False
        self.task_selected = False
        self.task_is_rw = False
        self.task_started = False
        self.task_pending = False
        self.task_done = False
        self.task_timedout = False
        self.at_task = False
        self.pose_available = False
        # Device status
        self.status = DEVICE_NOT_RESPONDING
        # myro robot
        self.epuck = None
        self.epuck_ready = False
        # navigator
        self.navigator = EpuckNavigator()
        # data logger
        self.step = 0
        self.pose_writer = None
	
    def InitLogFiles(self):
        # -- Init Stimuli writer --
        name = "RobotPose"
        now = time.strftime("%Y%b%H%M%S", time.gmtime())
        desc = "logged in centralized communication mode from: " + now
        # prepare label
        label = "TimeStamp;StepCounter;X;Y;Theta \n"
        # Data context
        ctx = DataCtx(name, label, desc)
        self.pose_writer = DataWriter("Robot", self.robotid, ctx, now)

    def GetCommonHeader(self):
        ts = time.strftime("%H%M%S", time.gmtime())
        sep = DATA_SEP
        header = str(ts) + sep + str(self.step)
        return header
	
    def AppendPoseLog(self):        
        sep = DATA_SEP
        self.pose.UpdateFromList(self.datamgr_proxy.mRobotPose)
        p = self.pose
        log = self.GetCommonHeader()\
         + sep + str(p.x) + sep + str(p.y) + sep + str(p.theta) + "\n"
        try: 
            self.pose_writer.AppendData(log)
        except:
            print "Pose logging failed"

    def EpuckReady(self):
        try:
            self.epuck = Epuck(self.datamgr_proxy.mRobotID)
            self.epuck_ready = True
        except:
            self.epuck_ready = False
            print "Failed to initialize Myro epuck robot"
        return self.epuck_ready

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
        if status == TASK_SELECTED:
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
        return self.task_done

    def TaskTimedOut(self):
        now = time.time()
        elasped = now - self.task_start
        if elasped > self.task_period:
            self.task_timedout = True
        return self.task_timedout

    def ArrivedAtTask(self):
        robot_x, robot_y, robot_theta = self.GetRobotPose()
        task_x, task_y = self.GetTaskPoseXY()
        dx = math.fabs(robot_x - task_x)
        dy = math.fabs(robot_y - task_y)
        print "ArrivedAtTask(): xdist: %f ydist: %f" %(dx, dy)
        pxdist = math.sqrt(dx * dx + dy * dy)
        if (pxdist < TASK_RADIUS) :
            self.at_task = True
            print "ArrivedAtTask(): Arrived within radius %.2f !\n" %pxdist
        return self.at_task

    def PoseAvailable(self):
        if self.pose.x: # <<  Test >>
            self.pose_available = True
            print "PoseAvailable Indicated"
        return self.pose_available

    def GetRobotPose(self):
        self.pose.UpdateFromList(self.datamgr_proxy.mRobotPose)
        x = self.pose.x
        y = self.pose.y
        theta = self.pose.y
        return x, y, theta

    def GetTaskPoseXY(self):
        st = self.datamgr_proxy.mSelectedTask[SELECTED_TASK_INFO]
        x = st[TASK_INFO_X]
        y= st[TASK_INFO_Y]
        return x, y

    def RunDeviceUnavailableLoop(self):
        while self.status is DEVICE_NOT_RESPONDING:
            if self.EpuckReady():
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
            elif (not self.EpuckReady()):
                self.status = DEVICE_NOT_RESPONDING
                self.RunDeviceUnavailableLoop() # go out of this loop
                break
            else:
               self.status = DEVICE_AVAILABLE # stay in-loop
               print "@DEVICE_AVAILABLE loop"
               time.sleep(SMALL_DELAY)
   
    def RunDeviceMovingLoop(self):
        while self.status is DEVICE_MOVING:
            logger.debug ("@DEVICE_MOVING loop")
            if self.TaskTimedOut():
                self.task_done = True
                self.status = DEVICE_AVAILABLE # go out of loop
                self.RunDeviceAvailableLoop()
                logger.debug ("Timedout")
                break 
            elif  self.task_is_rw:
                self.status = DEVICE_MOVING # stay in-loop
                # do random walking ...
                maxtime = time.time() + self.task_period
                try:
                    self.navigator.GoForward(self.epuck, maxtime)
                except:
                    print "Random walk failed"
            elif self.TaskPending():
                #if (not self.PoseAvailable()) or self.ArrivedAtTask():
                    #self.status = DEVICE_IDLE # go out of loop
                    #self.RunDeviceIdleLoop()
                    #logger.debug ("TaskPending")
                    #break
                #else :
                self.status = DEVICE_MOVING # stay in-loop
                # go to navigation routines for MoveToTarget
                self.AppendPoseLog()
                rx, ry, rtheta = self.GetRobotPose()
                task_x, task_y = self.GetTaskPoseXY()
                maxtime = time.time() + self.task_period
                try:
                    logger.debug ("--> Naviagtor")
                    self.navigator.GoTowardsTarget(self.epuck,\
                     rx, ry, rtheta, task_x, task_y, maxtime)
                except:
                    print "Going towards target failed"
            elif (not self.EpuckReady()):
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                self.RunDeviceUnavailableLoop()
            else:
                print "@RunDeviceMovingLoop: Unexpected situation "

    def RunDeviceIdleLoop(self):
        while self.status is DEVICE_IDLE:
            logger.debug ("@DEVICE_MOVING loop")
            if not self.EpuckReady():
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                self.RunDeviceUnavailabeLoop()
                break
            elif self.TaskTimedOut():
                self.status = DEVICE_AVAILABLE # go out of loop
                self.RunDeviceAvailableLoop()
                logger.debug ("Timedout")
                break
            elif  self.TaskPending() and self.PoseAvailable():
                self.status = DEVICE_MOVING # go out of loop
                self.RunDeviceMovingLoop()
                logger.debug ("TaskPending and PoseAvailable")
                break
            elif self.TaskPending() and self.ArrivedAtTask() : 
                # stay in loop
                self.status = DEVICE_IDLE
                time.sleep(SMALL_DELAY)
                logger.debug ("TaskPending and ArrivedAtTask")
            else:
                print "@RunDeviceIdleLoop: Unexpected situation "

    def RunMainLoop(self):          
        while EXIT_COND:
            try:
                self.RunDeviceUnavailableLoop()                
                time.sleep(2)
            except (KeyboardInterrupt, SystemExit):
                print "User requested exit... shutting down now"                
                sys.exit(0)

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
	dc.InitLogFiles()
        dc.RunMainLoop()
