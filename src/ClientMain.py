#!/usr/bin/python
import multiprocessing,  logging,  logging.config,  logging.handlers,  time

logging.config.fileConfig("logging.conf")
#create logger
logger = logging.getLogger("EpcLogger")

from multiprocessing import *
from RILCommonModules import *
from data_manager import *
from ril_robot import *
import dbus_server
import dbus_listener 
import dbus_emitter
import task_selector
multiprocessing.log_to_stderr(logging.DEBUG)


def main():
        logging.debug("--- Start EPC---")
        #dbus_server.start()
        dbus_listener.start()
        #taskselector.start()
        #dbus_emitter.start()
        # Ending....
        time.sleep(10)
        #dbus_server.join()
        dbus_listener.join()
        #taskselector.join()
        #dbus_emitter.join()
        logging.debug("--- End EPC---")


if __name__ == '__main__':
     robotid = '12'
     dbus_shared_path = DBUS_PATH_BASE + robotid
     dm = DataManager(id=12)
     robot = RILRobot(id=1)
     robot.InitTaskRecords(MAX_SHOPTASK)
     sig1 = SIG_ROBOT_POSE 
     sig2 = SIG_TASK_INFO
     sig3 = SIG_TASK_STATUS
     delay = 3 # interval between signals
     dbus_server = multiprocessing.Process(target=dbus_server.server_main,\
                            name="SwisTrackProxy",  args=(DBUS_IFACE_TRACKER, \
                            dbus_shared_path,  sig1,  delay,  ))
     dbus_listener = multiprocessing.Process(target=dbus_listener.listener_main,\
                            name="DBusListener",  args=(dm,  DBUS_IFACE_TRACKER,\
                            dbus_shared_path, DBUS_IFACE_TASK_SERVER, \
                            DBUS_PATH_TASK_SERVER,  sig1,  sig2,  delay ))
     taskselector = multiprocessing.Process(target=task_selector.selector_main,\
                            name="TaskSelector",  args=(dm,  robot ))
     dbus_emitter = multiprocessing.Process(target=dbus_emitter.emitter_main,\
                            name="DBusEmitter",  args=(dm,  DBUS_IFACE_EPUCK,\
                            dbus_shared_path,  sig3,   delay,  ))
                                                                     
     main()
      


