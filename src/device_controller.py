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
SMALL_DELAY = 1

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
        #self.l2ping_ok = False
        self.task_selected = False
        self.task_is_rw = False
        self.task_started = False
        self.task_pending = False
        self.task_done = False
        self.task_timedout = False
        self.at_task = False
        #self.pose_available = False
        self.last_pose_ts = 0 # last time stamp
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

    #def L2PingOK(self):
        ## [CodeMakeup] Check if l2ping exits!
        #cmd = "/usr/bin/l2ping -c " + " 1 " + self.bdaddr
        #subproc = subprocess.Popen([cmd, ],\
            #stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
        #stdout_value = subproc.communicate()
        ##print '\t got l2ping stdout_value:', stdout_value[0]
        #self.l2ping_ok = process_ping_output(stdout_value[0])
        #return self.l2ping_ok 

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

    #def TaskPending(self):
        #if self.task_selected and (not self.task_started):
            #self.task_pending = True
            #self.task_started = True
            #return self.task_pending
        #if  (not self.ArrivedAtTask()): # <<FixIt: RandomWalking case>>
            #self.task_pending = True
        #return self.task_pending

    #def TaskDone(self):
        #return self.task_done

    def TaskTimedOut(self):
        now = time.time()
        elasped = now - self.task_start
        if elasped > self.task_period:
            self.task_timedout = True
        return self.task_timedout

    def ArrivedAtTask(self):
        robot_x, robot_y, robot_theta = self.GetUpdatedRobotPose()
        task_x, task_y = self.GetTaskPoseXY()
        dx = math.fabs(robot_x - task_x)
        dy = math.fabs(robot_y - task_y)
        print "ArrivedAtTask(): xdist: %f ydist: %f" %(dx, dy)
        pxdist = math.sqrt(dx * dx + dy * dy)
        if (pxdist < TASK_RADIUS) :
            self.at_task = True
            print "ArrivedAtTask(): Arrived within radius %.2f !\n" %pxdist
        return self.at_task

    #def PoseAvailable(self):
        #if self.pose.x: # <<  Test >>
            #self.pose_available = True
            #print "PoseAvailable Indicated"
        #return self.pose_available
    
    def PoseUpdated(self):
        updated = False
        self.pose.UpdateFromList(self.datamgr_proxy.mRobotPose)
        if int(self.last_pose_ts) < int(self.pose.ts):
            updated = True
        return updated

    def GetUpdatedRobotPose(self):
        self.pose.UpdateFromList(self.datamgr_proxy.mRobotPose)
        x = self.pose.x
        y = self.pose.y
        theta = self.pose.y
        ts = self.pose.ts
        self.last_pose_ts = ts # used later, see PoseUpdated()
        return x, y, theta

    def GetTaskPoseXY(self):
        st = self.datamgr_proxy.mSelectedTask[SELECTED_TASK_INFO]
        x = st[TASK_INFO_X]
        y= st[TASK_INFO_Y]
        return x, y

    def RunDeviceUnavailableLoop(self):
        logger.debug ("Entering DEVICE_NOT_RESPONDING loop--->")
        while self.status is DEVICE_NOT_RESPONDING:
            if self.EpuckReady():
                self.status = DEVICE_AVAILABLE # out loop
                #self.RunDeviceAvailableLoop()
                break
            else: 
                self.status = DEVICE_NOT_RESPONDING # stay here in loop
                #time.sleep(SMALL_DELAY)

    def RunDeviceAvailableLoop(self):
        logger.debug ("Entering DEVICE_AVAILABLE loop--->")
        while self.status is DEVICE_AVAILABLE:
            if self.TaskSelected():
                self.task_start = time.time()
                self.status = DEVICE_MOVING
                #self.RunDeviceMovingLoop() # go out of this loop
                break
            elif (not self.EpuckReady()):
                self.status = DEVICE_NOT_RESPONDING
                #self.RunDeviceUnavailableLoop() # go out of this loop
                break
            else:
               self.status = DEVICE_AVAILABLE # stay in-loop
               print "@DEVICE_AVAILABLE loop"
               #time.sleep(SMALL_DELAY)
   
    def RunDeviceMovingLoop(self):
        logger.debug ("Entering DEVICE_MOVING loop--->")
        while self.status is DEVICE_MOVING:
            if (not self.EpuckReady()):
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                #self.RunDeviceUnavailableLoop()
                logger.debug ("\tDEVICE_NOT_RESPONDING")
                break
            elif self.TaskTimedOut():
                self.task_done = True
                self.status = DEVICE_AVAILABLE # go out of loop
                #self.RunDeviceAvailableLoop()
                logger.debug ("\tTimedout")
                break 
            # doing task 
            if  self.task_is_rw:
                self.status = DEVICE_MOVING # stay in-loop
                # do random walking ...
                maxtime = time.time() + self.task_period
                try:
                    self.navigator.GoForward(self.epuck, maxtime)
                    logger.debug ("\tRandom walking")
                    time.sleep(self.task_period)
                except:
                    logger.debug ("\tRandom walk failed")
            elif self.PoseUpdated(): # task is move to target
                if self.ArrivedAtTask():
                    logger.debug ("\tAtTask")
                    self.status = DEVICE_IDLE
                    #self.RunDeviceIdleLoop()
                    break
                else:
                    logger.debug ("\tMoving to Task")
                    self.status = DEVICE_MOVING # stay in-loop
                    # go to navigation routines for MoveToTarget
                    self.AppendPoseLog()
                    rx, ry, rtheta = self.GetUpdatedRobotPose()
                    task_x, task_y = self.GetTaskPoseXY()
                    maxtime = time.time() + self.task_period
                    try:
                        logger.debug ("\t--> Naviagtor")
                        self.navigator.GoTowardsTarget(self.epuck,\
                         rx, ry, rtheta, task_x, task_y, maxtime)
                        time.sleep(self.task_period)
                    except:
                        print "Going towards target failed"
            else:
                self.status = DEVICE_MOVING # stay in-loop
            
        logger.debug ("--->Exiting DEVICE_MOVING loop.")    
        
    def RunDeviceIdleLoop(self):
        logger.debug ("Entering DEVICE_IDLE loop--->")
        while self.status is DEVICE_IDLE:
            if not self.EpuckReady():
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                #self.RunDeviceUnavailabeLoop()
                logger.debug ("\tDEVICE_NOT_RESPONDING")
                break
            elif self.TaskTimedOut():
                self.status = DEVICE_AVAILABLE # go out of loop
                #self.RunDeviceAvailableLoop()
                logger.debug ("\tTimedout")
                break
            while (not self.TaskTimedOut()):
                if self.PoseUpdated():
                    if self.ArrivedAtTask():
                        logger.debug ("\tAtTask")
                        # stay in loop
                        self.status = DEVICE_IDLE
                    else:
                        self.status = DEVICE_MOVING # go out of loop
                        logger.debug ("\tgo out to DEVICE_MOVING")
                        #self.RunDeviceMovingLoop()
                        break
                else:
                    logger.debug ("\tPose not Updated")
                    # stay in loop
                    self.status = DEVICE_IDLE 
        logger.debug ("--->Exiting DEVICE_IDLE loop.")              


    def RunMainLoop(self):          
        while EXIT_COND:
            try:
                self.RunDeviceUnavailableLoop()
                self.RunDeviceAvailableLoop()
                self.RunDeviceMovingLoop()
                self.RunDeviceIdleLoop()                
                time.sleep(1)
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
