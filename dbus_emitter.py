#!/usr/bin/env python
import time, os, sys, sched, subprocess, re, signal, traceback
import gobject
import dbus, dbus.service, dbus.mainloop.glib 
import multiprocessing,  logging

from RILCommonModules.RILSetup import *
from RILCommonModules.LiveGraph import *
from EpuckCentralizedClient.data_manager import *
from EpuckCentralizedClient.utils import *

schedule = sched.scheduler(time.time, time.sleep)
loop = None
# ------------ Log signal emission
class CommLogger():
    def __init__(self, dm):
        self.datamgr_proxy = dm
        self.robotid = dm.GetRobotID()
        self.log_writer1 = None  # for logging emitted robot status signal      
        self.step = 0

    def InitLogFiles(self):
        name = "DBusEmitter"
        now = time.strftime("%Y%b%d-%H%M%S", time.gmtime())
        desc = "logged in centralized communication mode from: " + now
        # prepare label
        label = "TimeStamp;HH:MM:SS;StepCounter;TaskID\n"
        # Data context
        ctx = DataCtx(name, label, desc)
        # Signal Logger
        self.log_writer1 = DataWriter("Robot", ctx, now, str(self.robotid))

    def _GetCommonHeader(self):
        sep = DATA_SEP
        ts = str(time.time()) + sep + time.strftime("%H:%M:%S", time.gmtime())
        self.step = self.step + 1
        header = ts + sep + str(self.step)
        return header
    
    def AppendCommLog(self, taskid):        
        sep = DATA_SEP        
        log = self._GetCommonHeader()\
         + sep + str(taskid) + sep + "\n"
        try: 
            self.log_writer1.AppendData(log)
        except:
            print "RobotStatus signal logging failed"

#------------------ Signal Despatch ---------------------------------
class TaskStatusSignal(dbus.service.Object):
    def __init__(self, object_path):
        dbus.service.Object.__init__(self, dbus.SessionBus(), object_path)
    @dbus.service.signal(dbus_interface= DBUS_IFACE_EPUCK, signature='sii')
    def  TaskStatus(self,  sig,  robotid,  taskid):
        #logger.info("Emitted %s : Robot %d now %s ",  sig,  taskid,  status)
        #print "Emitted %s : Robot selected task %d now %s \
        #"  %(sig,  taskid,  status)
        pass
    def Exit(self):
        global loop
        loop.quit()

    
def emit_task_signal(delay,  sig1):
    global task_signal,  datamgr_proxy, comm_logger
    schedule.enter(delay, 0, emit_task_signal, (delay, sig1  ) )
    try:
        datamgr_proxy.mSelectedTaskStarted.wait()
        robotid = datamgr_proxy.mRobotID
        taskdict = datamgr_proxy.mSelectedTask
        datamgr_proxy.mSelectedTaskStarted.clear()
        taskid =  eval(str(taskdict[SELECTED_TASK_ID])) 
        status = str(taskdict[SELECTED_TASK_STATUS]) 
#        for k, v in taskdict.items():
#            taskid = eval(str(k))
#            status = str(v)
        print "From TaskDict got %i %s"  %(taskid,  status)
        task_signal.TaskStatus(sig1,  robotid,  taskid)
        comm_logger.AppendCommLog(taskid)
    except:
        print "Emitting TaskStatus signal failed"
   


def emitter_main(dm,  dbus_iface= DBUS_IFACE_EPUCK,  dbus_path =\
    DBUS_PATH_BASE, sig1 = "TaskStatus",   delay = 3):
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    session_bus = dbus.SessionBus()
    global task_signal,  datamgr_proxy, comm_logger
    datamgr_proxy = dm
    # setup logging status emission
    comm_logger = CommLogger(dm)
    comm_logger.InitLogFiles()
    try:
        name = dbus.service.BusName(dbus_iface, session_bus)
        task_signal = TaskStatusSignal(dbus_path)
        loop = gobject.MainLoop()
        print "Running Outbound TaskStatus service."
    except dbus.DBusException:
        traceback.print_exc()
        sys.exit(1)
    try:
            e = schedule.enter(0, 0, emit_task_signal, (delay,  sig1,  ))
            schedule.run()
            loop.run()
    except (KeyboardInterrupt, dbus.DBusException, SystemExit):
            print "User requested exit... shutting down now"
            task_signal.Exit()
            pass
            sys.exit(0)
