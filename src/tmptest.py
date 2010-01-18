#!/usr/bin/env python

#from device_controller import *

from epuck_navigator import *

navigator = EpuckNavigator()


#dc = DeviceController(1,0)
#print "TCA:", dc.navigator.mTaskConeAngle

navigator.GoTowardsTarget(1,  1750, 900, 4.9, 1500, 1300, 3)
