import time
from multiprocessing import *
from multiprocessing.managers import BaseManager

class DataManager(object):
    def __init__(self,  id=-1):
        self.mRobotID = id
        self.mgr = Manager()
        self.mRobotPose = self.mgr.dict() # to retrieve last  observed pose
       # key: x, y, theta, time_stamp(ts) val: <values>
        self.mRobotPoseAvailable = self.mgr.Event() # set by dbus client
        self.mTaskInfo = self.mgr.dict() 
        # key: taskid, value: list of attrib (t.s., x, y,  phi)
        self.mTaskInfoAvailable = self.mgr.Event() # set by dbus client
        self.mSelectedTask = self.mgr.dict()  
        # Set/Unset by TaskSelector, DeviceController
        # key: SEE RILsetup, val: Values
        
        self.mSelectedTaskAvailable = self.mgr.Event() 
        # Set/Unset by TaskSelector
        
        self.mSelectedTaskStarted = self.mgr.Event()
        #set by Device controller, used/clear by dbus_emmitter
        
        # DeviceController Signals
        self.mTaskTimedOut = self.mgr.Event()  # Set/Unset by DeviceController
        #self.mDeviceNotResponding =
    
    # accessors
    def GetRobotID(self):
        return self.mRobotID        
    def GetRobotPose(self):
        val = self.mRobotPose.copy()
        return str(val)
    def IsRobotPoseAvailable(self):
        val = False
        if self.mRobotPoseAvailable:
            val = True
        return val
    def GetTaskInfo(self):
        val = self.mTaskInfo
        return str(val)
    def IsTaskInfoAvailable(self):
        val = self.mTaskInfoAvailable
        return str(val)
    def GetSelectedTask(self):
        val = self.mSelectedTask.copy()
        return str(val) 
    def IsSelectedTaskAvailable(self):
        val = False
        if self.mSelectedTaskAvailable:
            val = True
        return val
    def IsSelectedTaskStarted(self):
        val = False
        if self.mSelectedTaskStarted:
            val = True
        return val
    def IsTaskTimedOut(self):
        val = False
        if self.mTaskTimedOut:
            val = True
        return val

    # mutators
    def SetRobotID(self, i):
        self.mRobotID = i

    # robot pose 
    def SetRobotPose(self, pose):
        for k, v in pose.iteritems():
            self.mRobotPose[k] = v
    def SetRobotPoseAvailable(self):
        self.mRobotPoseAvailable.set()
    def ClearRobotPoseAvailable(self):
        self.mRobotPoseAvailable.clear()
    
    # task info
    def SetTaskInfo(self, ti):
        for k, v in ti.iteritems():
            self.mTaskInfo[k] = v
    def SetTaskInfoAvailable(self):
        self.mTaskInfoAvailable.set()
    def ClearTaskInfoAvailable(self):
        self.mTaskInfoAvailable.clear()

    # selected task
    def SetSelectedTask(self, task):
        for k, v in task.iteritems():
            self.mSelectedTask[k] = v
    def SetSelectedTaskAvailable(self):
        self.mSelectedTaskAvailable.set()
    def ClearSelectedTaskAvailable(self):
        self.mSelectedTaskAvailable.clear()
    
    def SetSelectedTaskStarted(self):
        self.mSelectedTaskStarted.set()
    def ClearSelectedTaskStarted(self):
        self.mSelectedTaskStarted.clear()
    def SetTaskTimedOut(self):
        self.mTaskTimedOut.set()
    def ClearTaskTimedOut(self):
        self.mTaskTimedOut.clear()

class RemoteManager(BaseManager):
  pass
  
def datamgr_main(dm):
    tgt = dm
    RemoteManager.register('get_target', callable=lambda:tgt)
    mgr = RemoteManager(address=('localhost', 50000), authkey="123")
    srv = mgr.get_server()
    srv.serve_forever()

