import  time
from math import fabs, atan2

#from myro import *
from RILCommonModules.RILSetup import *
from RILCommonModules.pose import *
from utils import *

PROXIMITY_THRESHOLD = 200
FORWARD_SPEED = 0.5
FORWARD_STEP_TIME = 1

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

    
    def GoForward(self, epuck, timeout):
        e = epuck
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
            if (localangle >= ANGLE180):
                print "GoToTaskLoc(): Multi step turning needed \n"
                localangle1 = ANGLE180
                localangle2 = localangle - localangle1
                print "GoToTaskLoc(): Local angle %.2f + %.2f " %(localangle1, localangle2)
                # first turn
                # TurnReverse(pc, p2d, FIX_VA);
                # second turn
                #if(localangle2 > mTaskConeAngle) #TurnRight(pc, p2d, localangle2, FIX_VA);
                #else: #TurnRight(pc, p2d, localangle, FIX_VA);
            elif (thetadiff > 0): #// Left Turn
                  print "GoToTaskLoc(): ThetaDesired: %.2f Localangle %.2f"\
                    %(self.mThetaDesired, localangle)
                  print "=> TurnLeft selected"
                  if (localangle >= ANGLE180) :
                    print "GoToTaskLoc(): Multi step turning needed \n" 
                    localangle1 = ANGLE180
                    localangle2 = localangle - localangle1
                    print "GoToTaskLoc(): Local angle %.2f + %.2f \n"\
                     %(localangle1, localangle2)
                    # first turn
                    # TurnReverse(pc, p2d, FIX_VA);
                    # second turn
                    #if(localangle2 > mTaskConeAngle): TurnLeft(pc, p2d, localangle2, FIX_VA)
                    #else: TurnLeft(pc, p2d, localangle, FIX_VA);



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

    def  GoQ12(self):
        self.mThetaDesired = ANGLE90
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad
        if((self.mRobotPose.theta > (ANGLE270 - DELTA_ANGLE0))  and\
        (self.mRobotPose.theta < (ANGLE270 + DELTA_ANGLE0))):
            printf("with TurnReverse \n");
            #TurnReverse(pc, p2d, FIX_VA);
            Translate()
        elif ((self.mRobotPose.theta > (ANGLE270 - DELTA_ANGLE0))  and\
        (self.mRobotPose.theta < (ANGLE270 + DELTA_ANGLE0))):
             Translate()
        else:
            print "with rotation \n"
    
    def GoQ23(self):
        self.mThetaDesired = ANGLE180
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad
        if((self.mRobotPose.theta > (ANGLE360 - DELTA_ANGLE0))  or\
        (self.mRobotPose.theta < (DELTA_ANGLE0))):
            printf("with TurnReverse \n");
            #TurnReverse(pc, p2d, FIX_VA);
            Translate()
        elif ((self.mRobotPose.theta > (ANGLE180 - DELTA_ANGLE0))  and\
           (self.mRobotPose.theta < (ANGLE180 + DELTA_ANGLE0))):
             Translate()
        else:
            print "with rotation \n"
    
    def GoQ34(self):
        self.mThetaDesired = ANGLE270
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad
        if((self.mRobotPose.theta > (ANGLE90 - DELTA_ANGLE0))  and\
           (self.mRobotPose.theta < (ANGLE90 + DELTA_ANGLE0))):
            printf("with TurnReverse \n");
            #TurnReverse(pc, p2d, FIX_VA);
            Translate()
        elif ((self.mRobotPose.theta > (ANGLE90 - DELTA_ANGLE0))  and\
           (self.mRobotPose.theta < (ANGLE90 + DELTA_ANGLE0))):
             Translate()
        else:
            print "with rotation \n"

    def GoQ41(self):
        self.mThetaDesired = 0.0
        print "GoToTaskLoc(): **Quad %d**, called "  %self.mCurrentQuad
        if((self.mRobotPose.theta > (ANGLE180 - DELTA_ANGLE0))  and\
        (self.mRobotPose.theta < (ANGLE180 + DELTA_ANGLE0))):
            printf("with TurnReverse \n");
            #TurnReverse(pc, p2d, FIX_VA);
            Translate()
        elif ((self.mRobotPose.theta > (ANGLE360 - DELTA_ANGLE0))  and\
        (self.mRobotPose.theta < (DELTA_ANGLE0))):
             Translate()
        else:
            print "with rotation \n"
    
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
        pass
        #        if(!ArrivedAtTaskLoc()) {
        #        #//p2d->SetSpeed(0.4,-0.26);
        #       double gone = GoAhead(pc, p2d, ir, BASIC_VX);
        #          Sleep(100);
        #        } else {
        #          Stop(pc, p2d);
        #        }
    
    
    def Rotate(self):
        print "Now Doing Rotattion\n"
        pass

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



   
