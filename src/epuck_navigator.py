import  time
import sys
from math import fabs, atan2, atan

from myro import *
from RILCommonModules.RILSetup import *
from RILCommonModules.pose import *
from utils import *
import  logging,  logging.config,  logging.handlers
logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")

PROXIMITY_THRESHOLD = 200
TRANSLATE_SPEED = 0.5
ROTATE_SPEED = 0.644 # 1.0 == 3.14 rev/s, 0.644 == 2.04 rev/s
STEP_DIST = 40 # pixel
TRANSLATE_CONST = 40 # dividing desired px-dist by this gives time(translate) 
ROTATE_CONST = 0.637/2 # mutiplying this with desired angle gives time(rotate)
# typical value: 0.637  max val = 0.411
FORWARD_STEP_TIME = 1
# Obstacle Avoidance params
BACKWARD_SPEED1 = -0.8 # orig: -0.4
BACKWARD_TURN = 0.5 # orig: 0 , no turn
AVOID_WAIT = 0.1 # orig: 0.05

class NavFunc :
    NOTSET = -1
    MINZ = 1,  # minimize axis value
    FIXD = 50 # keep axis value const
    MAXZ = 100  #maximize axis value

class  Quad:
    Q0 = 0 #unknown
    Q1 = 1 #top left
    Q2 = 2 #top right
    Q3 = 3 #bottom right
    Q4 = 4 #bottom left
    Q12 = 12
    Q23 = 23
    Q34 = 34
    Q41 = 41
class EpuckNavigator:
    """Move epuck to a target task boundary using gps like robot pose data """
    def __init__(self):
        self.mRobotPose = Pose()
        self.mTaskPose = Pose()
        self.mTaskRadius = TASK_RADIUS
        self.mTaskConeAngle =  TASK_CONE_ANGLE
        self.mXFunc = NavFunc.NOTSET
        self.mYFunc = NavFunc.NOTSET
        self.mCurrentQuad = Quad.Q0
        self.mThetaDesired = 0.0
        self.mThetaLocal = 0.0
        # epuck
        self.epuck = None

    
    def GoForward(self, e, timeout = FORWARD_STEP_TIME):
        self.epuck = e
        maxtime = time.time() + timeout
        threshold = PROXIMITY_THRESHOLD
        while int(time.time()) < int(maxtime):
            sensors = e.getProximity()
            #print sensors
            maxval = max(sensors)
            if maxval > threshold:
                mostActive = sensors.index(maxval)
                allActive = [i for i in range(8) if sensors[i] > PROXIMITY_THRESHOLD]
                print 'sensor %d activated' % mostActive
                #print 'i feel sensors', allActive
                # flash the closest LED
                led = e.getLEDnumber(mostActive)
                e.flashLED(led, 0.25)
                # response
                if allActive == [0, 7]:
                    trans = e.getTranslate()
                    rotate = e.getRotate()
                    e.move(0, 0.7)
                    wait(1)
                    e.move(trans, rotate)
                elif mostActive in (0, 7):
                    e.move(BACKWARD_SPEED1 , BACKWARD_TURN)
                elif mostActive == 1:
                    e.move(-0.3, 0.1)
                elif mostActive == 6:
                    e.move(-0.3, -0.1)
                elif mostActive == 2:
                    e.move(0.5, 0.2)
                elif mostActive == 5:
                    e.move(0.5, -0.2)
                elif mostActive == 3:
                    e.move(0.4, 0.2)
                elif mostActive == 4:
                    e.move(0.4, -0.2)
            else:
                # nothing detected
                e.forward(TRANSLATE_SPEED, FORWARD_STEP_TIME)
                
            wait(AVOID_WAIT)
        e.move(0, 0) # stop

    def GoTowardsTarget(self, epuck, rx, ry, rtheta, task_x, task_y, maxtime):
        self.epuck = epuck
        self.UpdateCurrentPose(rx, ry, rtheta)
        self.SetupTaskLoc(task_x,  task_y)
        self.UpdateTargetTaskAngle()
        # update objective function
        self.UpdateNavFunc()
        logger.debug("GoTowardsTarget(): START: %s", time.ctime())
        logger.debug("TaskPoseOrientation:%2f Cone:%2f",\
            self.mTaskPose.theta, self. mTaskConeAngle)
        # Do Rotation by Quad 
        GoActionSwitch = {"12" : self.GoQ12, "23": self.GoQ23,\
            "34": self.GoQ34,  "41":  self.GoQ41,"1": self.GoQ1,\
             "2": self.GoQ2, "3": self.GoQ3, "4": self.GoQ4 }
        GoActionSwitch.get(str(self.mCurrentQuad),  self.GoErrHandler)()
        # Trasnslation is fixed
        time.sleep(FORWARD_STEP_TIME)
        self.Translate(epuck)
        #self.epuck.backward(TRANSLATE_SPEED, 5)

    def SetupTaskLoc(self, x, y, r=TASK_RADIUS, ca=TASK_CONE_ANGLE):
        self.mTaskPose.x = x
        self.mTaskPose.y= y
        self.mTaskRadius = r
        self.mTaskConeAngle = ca
        logger.info("Updated TaskLoc: x= %f y= %f", x, y)

    def UpdateCurrentPose(self,  x,  y,  theta):
        self.mRobotPose.x = x
        self.mRobotPose.y = y
        self.mRobotPose.theta= theta
        logger.info( "Updated RobotPose: x= %f y= %f  theta:%f", x, y, theta)

    def UpdateTargetTaskAngle(self):
        dx = fabs(self.mRobotPose.x - self.mTaskPose.x)
        dy = fabs(self.mRobotPose.y - self.mTaskPose.y)
        self.mTaskPose.theta= atan(dy/dx)

    def UpdateNavFunc(self):
        rx = self.mRobotPose.x
        ry = self.mRobotPose.y
        tx1 = self.mTaskPose.x - self.mTaskRadius
        ty1 = self.mTaskPose.y - self.mTaskRadius
        tx2 = self.mTaskPose.x + self.mTaskRadius
        ty2 = self.mTaskPose.y + self.mTaskRadius
        # reset previous values
        self.mXFunc = NavFunc.NOTSET
        self.mYFunc = NavFunc.NOTSET
        self.mCurrentQuad = Quad.Q0
        # for four vertical axes
        if ( (rx < tx1)  and  (ry < ty2 ) and  (ry > ty1) ):  # xleft
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.FIXD
            logger.debug("NavFunc set @xleft")
            self.mCurrentQuad = Quad.Q41
        elif ( (rx > tx2)  and  (ry < ty2 )  and (ry > ty1) ) :      #xright
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.FIXD
            logger.debug("NavFunc set @xright ")
            self.mCurrentQuad = Quad.Q23
        elif ( (rx < tx2)  and  (rx > tx1 ) and  (ry < ty1) ):  #yup
            self.mXFunc = NavFunc.FIXD
            self.mYFunc = NavFunc.MAXZ
            logger.debug("NavFunc set @yup ")
            self.mCurrentQuad = Quad.Q12
        elif ( (rx < tx2)  and  (rx > tx1 ) and  (ry > ty2) ):   #udown
            self.mXFunc = NavFunc.FIXD
            self.mYFunc = NavFunc.MINZ
            printf("NavFunc set @ydown ")
            self.mCurrentQuad = Quad.Q34
        elif ( (rx < tx1) and  (ry < ty1) ) :  # @Q1 : topleft
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.MAXZ
            self.mCurrentQuad = Quad.Q1
            logger.debug("NavFunc set @Q1 ")        
        elif ( (rx > tx2)  and (ry < ty1) ) :
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.MAXZ
            self.mCurrentQuad = Quad.Q2
            logger.debug("NavFunc set @Q2 ")
        elif ( (rx > tx2)  and  (ry > ty2) ) :
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.MINZ
            self.mCurrentQuad = Quad.Q3
            logger.debug("NavFunc set @Q3 ")
        elif ( (rx < tx1)  and  (ry > ty2) ) :
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.MINZ
            self.mCurrentQuad = Quad.Q4
            logger.debug("NavFunc set @Q4 ")
        else:
            logger.warn("Unknown Quad: Make Robot pose > task radius")
    
    
    def  GoQ12(self):
        self.mThetaDesired = ANGLE90
        self.TurnReverse()
    
    def GoQ23(self):
        self.mThetaDesired = ANGLE180
        self.TurnReverse()
    
    def GoQ34(self):
        self.mThetaDesired = ANGLE270
        self.TurnReverse()

    def GoQ41(self):
        self.mThetaDesired = 0.0
        self.TurnReverse()
    
    def GoQ1(self):
        self.mThetaDesired = self.mTaskPose.theta
        logger.info("GoQ1(): Theta desired %f",  self.mThetaDesired)
        min_angle =  self.mThetaDesired
        max_angle = ANGLE180 + self.mThetaDesired
        if(min_angle < self.mRobotPose.theta < max_angle):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            self.RotateLeft(rotate)
            logger.debug("\t 1st turn choice %f", rotate)
        elif(max_angle < self.mRobotPose.theta < ANGLE360):
            rotate = (ANGLE360 - self.mRobotPose.theta) + self.mThetaDesired
            self.RotateRight(rotate)
            logger.debug("\t 2nd turn choice %f", rotate)
        else:
            rotate = self.mThetaDesired - self.mRobotPose.theta 
            self.RotateRight(rotate)
            logger.debug("\t 3rd turn choice %f", rotate)

    def GoQ2(self):
        self.mThetaDesired = ANGLE180 -  self.mTaskPose.theta
        logger.info("GoQ2(): Theta desired %f", self.mThetaDesired)
        min_angle = self.mThetaDesired
        max_angle = ANGLE360 - self.mTaskPose.theta
        if(min_angle <= self.mRobotPose.theta <= max_angle):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            self.RotateLeft(rotate)
        elif(max_angle <= self.mRobotPose.theta <= ANGLE360):
            rotate = (ANGLE360 - self.mRobotPose.theta) + self.mThetaDesired
            self.RotateRight(rotate)
        else:
            rotate = self.mThetaDesired - self.mRobotPose.theta 
            self.RotateRight(rotate)
        
    def GoQ3(self):
        self.mThetaDesired = ANGLE180  + self.mTaskPose.theta
        logger.info("GoQ3(): Theta desired %f", self.mThetaDesired)
        min_angle = self.mTaskPose.theta
        max_angle = self.mThetaDesired
        if(min_angle <= self.mRobotPose.theta <= max_angle):
            rotate = self.mThetaDesired - self.mRobotPose.theta
            self.RotateRight(rotate)
        elif(ANGLE0 <= self.mRobotPose.theta <= min_angle):
            rotate = self.mRobotPose.theta + ANGLE360 -  self.mThetaDesired
            self.RotateLeft(rotate)
        else:
            rotate = self.mRobotPose.theta -  self.mThetaDesired
            self.RotateLeft(rotate)

    def GoQ4(self):
        self.mThetaDesired = ANGLE360 -  self.mTaskPose.theta
        logger.info("GoQ4(): Theta desired %f", self.mThetaDesired) 
        min_angle = ANGLE180 - self.mTaskPose.theta
        max_angle = self.mThetaDesired
        logger.info("min_angle %f max_angle %f", min_angle, max_angle)
        if(min_angle <= self.mRobotPose.theta <= max_angle):
            rotate = self.mThetaDesired - self.mRobotPose.theta
            #print "robot pose falls w/i min-max"
            self.RotateRight(rotate)
        elif(max_angle <= self.mRobotPose.theta <= ANGLE360):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            #print "robot pose falls w/i max - 360"
            self.RotateLeft(rotate)
        else:
            rotate = self.mRobotPose.theta + self.mTaskPose.theta
            #print "robot pose falls inside third block"
            self.RotateLeft(rotate)

    def GoErrHandler(self): 
        logger.warn("Unknown Quad, can't go ")
    
    def TurnReverse(self):
        logger.debug("\t Turning reverse")
        tm = ROTATE_CONST * ANGLE180
        self.TurnLeft(tm)

    def RotateLeft(self, angle):
        logger.debug("\t Rotating left for %f rad" %angle)
        tm = ROTATE_CONST * angle
        self.TurnLeft(tm)
    
    def RotateRight(self, angle):
        logger.debug("\t Rotating right for %f rad" %angle)
        tm = ROTATE_CONST * angle
        self.TurnRight(tm)
    
    def Translate(self, epuck):
        timeout = STEP_DIST / TRANSLATE_CONST
        logger.debug("Now Doing Translation for %f sec" %timeout)
        try:
            #self.epuck.forward(TRANSLATE_SPEED, timeout)
            self.GoForward(epuck, timeout)
            time.sleep(timeout)
        except Exception,e:
            print e
            print sys.exc_info()[0]
            logger.error("Translatation failed for %s", sys.exc_info()[0])
    
    def TurnLeft(self, timeout):
        logger.debug("Now Doing Left Rotation for %f sec" %timeout)
        try:
            self.epuck.turnLeft(ROTATE_SPEED, timeout)
        except:
            logger.debug("TurnLeft failed")
    
    def TurnRight(self, timeout):
        logger.debug("Now Doing Right Rotation for %f sec" %timeout)
        try:
            self.epuck.turnRight(ROTATE_SPEED, timeout)
        except:
            logger.debug("TurnRight failed")

