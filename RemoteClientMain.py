#!/usr/bin/python
from multiprocessing.managers import BaseManager
from EpuckCentralizedClient.data_manager import *

class RemoteManager(BaseManager):
  pass

RemoteManager.register('get_target')

if __name__ == '__main__':
    mgr = RemoteManager(address=('localhost', 50000), authkey="123")
    mgr.connect()
    tgt = mgr.get_target()
    print tgt
    print tgt.GetRobotID()
    print tgt.GetRobotPose()
    print tgt.GetTaskInfo()
    #print tgt.IsRobotPoseAvailable()
    #tgt.SetSelectedTaskStarted()
    print tgt.IsSelectedTaskAvailable()
    print tgt.GetSelectedTask()
    
