#!/usr/bin/env python

from device_controller import *

dc = DeviceController(1,0)
#print "TCA:", dc.navigator.mTaskConeAngle
dc.navigator.GoTowardsTarget(1, 200, 2300, 1.5, 500, 300, 3)