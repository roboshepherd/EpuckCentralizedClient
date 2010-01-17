import  time
from math import fabs, atan2

#from myro import *
from RILCommonModules.RILSetup import *
from RILCommonModules.pose import *
from utils import *

PROXIMITY_THRESHOLD = 200
TRASNLATE_SPEED = 0.466
ROTATE_SPEED = 0.644 
STEP_DIST = 100 # pixel
TRANSLATE_CONST = 40 # dividing desired px-dist by this gives time(translate) 
ROTATE_CONST = 0.67 # mutiplying this with desired angle gives time(rotate)
FORWARD_STEP_TIME = 2

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

    
    def GoForward(self, timeout):
        e = self.epuck
        maxtime = time.time() + timeout
        threshold = PROXIMITY_THRESHOLD
        while int(time.time()) < int(maxtime):
            sensors = e.getProximity()
            #print sensors
            maxval = max(sensors)
            if maxval > threshold:
                mostActive = sensors.index(maxval)
                allActive = [i for i in range(8) if sensors[i] > 200]
                #print 'sensor %d activated' % mostActive
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
                    e.move(-0.4, 0)
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
                e.forward(0.3, FORWARD_STEP_TIME)
                
            wait(0.05)

    def GoTowardsTarget(self, epuck, rx, ry, rtheta, task_x, task_y, maxtime):
        self.epuck = epuck
        self.UpdateCurrentPose(rx, ry, rtheta)
        self.SetupTaskLoc(task_x,  task_y)
        self.UpdateTargetTaskAngle()
        # update objective function
        self.UpdateNavFunc()
        print "GoTowardsTarget(): START:", time.ctime()
        print "TaskPoseOrientation:%2f Cone:%2f\n"\
            %(self.mTaskPose.theta, self. mTaskConeAngle)
        GoActionSwitch = {"12" : self.GoQ12, "23": self.GoQ23,\
            "34": self.GoQ34,  "41":  self.GoQ41,"1": self.GoQ1,\
             "2": self.GoQ2, "3": self.GoQ3, "4": self.GoQ4 }
        GoActionSwitch.get(str(self.mCurrentQuad),  self.GoErrHandler)()
        # Roatation Calculator //TODO Fix long reverse
        thetadiff = self.mRobotPose.theta - self.mThetaDesired 
        # in tracker unit 0 to 6.28 rad
        localangle = fabs(thetadiff)
        if(localangle < self.mTaskConeAngle):
          print "GoTowardsTarget(): [No Turning] localangle:%.2f\
          < TaskConeAngle:%.2f" %(localangle, self.mTaskConeAngle)
          self.GoForward(epuck, maxtime)
        elif (thetadiff < 0):  # Right Turn
            print "GoToTaskLoc(): ThetaDesired: %.2f Localangle %.2f"\
            %(self.mThetaDesired, localangle)
            print "=> TurnRight selected" 



    def SetupTaskLoc(self, x, y, r=TASK_RADIUS, ca=TASK_CONE_ANGLE):
        self.mTaskPose.x = x
        self.mTaskPose.y= y
        self.mTaskRadius = r
        self.mTaskConeAngle = ca
        print "Updated TaskLoc: x= %f y= %f" %(x, y)

    def UpdateCurrentPose(self,  x,  y,  theta):
        self.mRobotPose.x = x
        self.mRobotPose.y = y
        self.mRobotPose.theta= theta
        print "Updated RobotPose: x= %f y= %f  theta:%f" %(x, y, theta)

    def UpdateTargetTaskAngle(self):
        dx = fabs(self.mRobotPose.x - self.mTaskPose.x)
        dy = fabs(self.mRobotPose.y - self.mTaskPose.y)
        self.mTaskPose.theta= atan2(dy,  dx)

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
            print "NavFunc set @xleft \n"
            self.mCurrentQuad = Quad.Q41
        elif ( (rx > tx2)  and  (ry < ty2 )  and (ry > ty1) ) :      #xright
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.FIXD
            print "NavFunc set @xright \n"
            self.mCurrentQuad = Quad.Q23
        elif ( (rx < tx2)  and  (rx > tx1 ) and  (ry < ty1) ):  #yup
            self.mXFunc = NavFunc.FIXD
            self.mYFunc = NavFunc.MAXZ
            print "NavFunc set @yup \n"
            self.mCurrentQuad = Quad.Q12
        elif ( (rx < tx2)  and  (rx > tx1 ) and  (ry > ty2) ):   #udown
            self.mXFunc = NavFunc.FIXD
            self.mYFunc = NavFunc.MINZ
            printf("NavFunc set @ydown \n");
            self.mCurrentQuad = Quad.Q34
        elif ( (rx < tx1) and  (ry < ty1) ) :  # @Q1 : topleft
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.MAXZ
            self.mCurrentQuad = Quad.Q1
            print "NavFunc set @Q1 \n"        
        elif ( (rx > tx2)  and (ry < ty1) ) :
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.MAXZ
            self.mCurrentQuad = Quad.Q2
            print "NavFunc set @Q2 \n"
        elif ( (rx > tx2)  and  (ry > ty2) ) :
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.MINZ
            self.mCurrentQuad = Quad.Q3
            print "NavFunc set @Q3 \n"
        elif ( (rx < tx1)  and  (ry > ty2) ) :
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.MINZ
            self.mCurrentQuad = Quad.Q4
            print "NavFunc set @Q4 \n"
        else:
            print "Unknown Quad \n"
    
    def MoveReverse(self):
        turn_angle = ANGLE180
        t1 = ROTATE_CONST * ANGLE180
        self.TurnLeft(t1)
        t2 = STEP_DIST / TRANSLATE_CONST
        Translate(t2)

    def  GoQ12(self):
        self.mThetaDesired = ANGLE90
        self.MoveReverse()
    
    def GoQ23(self):
        self.mThetaDesired = ANGLE180
        self.MoveReverse()
    
    def GoQ34(self):
        self.mThetaDesired = ANGLE270
        self.MoveReverse()

    def GoQ41(self):
        self.mThetaDesired = 0.0
        self.MoveReverse()
    
    def GoQ1(self):
        self.mThetaDesired = self.mTaskPose.theta
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad

    def GoQ2(self):
        self.mThetaDesired = ANGLE180 -  self.mTaskPose.theta
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad

    def GoQ3(self):
        self.mThetaDesired = ANGLE180  + self.mTaskPose.theta
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad
    
    def GoQ4(self):
        self.mThetaDesired = ANGLE360 -  self.mTaskPose.theta
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad
        
    def GoErrHandler(self):
        print "Unknown Quad, can't go \n"
    
    def Translate(self):
        print "Now Doing Translation\n"
        try:
            self.epuck.forward(TRANSLATE_SPEED, timeout)
        except:
            print "Translate failed"
    
    def TurnLeft(self, timeout):
        print "Now Doing Left Rotation\n"
        try:
            self.epuck.turnLeft(ROTATE_SPEED, timeout)
        except:
            print "TurnLeft failed"
    
    def TurnRight(self, timeout):
        print "Now Doing Right Rotation\n"
        try:
            self.epuck.turnRight(ROTATE_SPEED, timeout)
        except:
            print "TurnRight failed"

    def ArrivedAtTaskLoc(self):
        ret = False
        dx = fabs(self.mRobotPose.x - self.mTaskPose.x)
        dy = fabs(self.mRobotPose.y - self.mTaskPose.y)
        print "ArrivedAtTaskLoc(): xdist: %f ydist: %f" %(dx, dy)
        pxdist = sqrt(dx * dx + dy * dy)
        print "********ArrivedAtTaskLoc(): pxdist: %.2f taskradi: %.2f\n" %(pxdist, mTaskRadius)
        if (pxdist < mTaskRadius) :
            ret = True
            print "ArrivedAtTaskLoc(): Arrived within radius %.2f !\n" %self.mTaskRadius
        return ret



   
