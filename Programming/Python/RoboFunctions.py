import math
import sys

class RoboCar:
    steeringLeft=0
    steeringRight=0
    rearLeft=0
    rearRight=0
    frontLeft=0
    frontRight=0
    desiredSteeringAngle=0
    desiredWheelRotSpeed=0
    leftFront_enc=0
    rightFront_enc=0
    leftRear_enc=0
    rightRear_enc=0
    leftSteering_enc=0
    rightSteering_enc=0
    cam=0
    client=0
    simTime=0
    cam_image=[]
    disconnect=False
    stopDisconnect=True
    d=0.0604 # 2*d=distance between left and right wheels
    l=0.2062 # l=distance between front and read wheels
    __lF_old=0
    __rF_old=0
    __lR_old=0
    __rR_old=0
    __lS_old=0
    __rS_old=0
    __lF_rot=0
    __rF_rot=0
    __lR_rot=0
    __rR_rot=0
    __lS_rot=0
    __rS_rot=0
    __simState=0
    __oldSimState=0
    
    def __init__(self, cl):
        self.client=cl
        errHand, self.steeringLeft=self.client.simxGetObjectHandle('SteeringLeft', self.client.simxServiceCall() )
        errHand, self.steeringRight=self.client.simxGetObjectHandle('SteeringRight', self.client.simxServiceCall() )
        errHand, self.rearLeft=self.client.simxGetObjectHandle('RearAxisLeft', self.client.simxServiceCall() )
        errHand, self.rearRight=self.client.simxGetObjectHandle('RearAxisRight', self.client.simxServiceCall() )
        errHand, self.frontLeft=self.client.simxGetObjectHandle('Front_motorLeft', self.client.simxServiceCall() )
        errHand, self.frontRight=self.client.simxGetObjectHandle('Front_motorRight', self.client.simxServiceCall() )
        errHand, self.cam=self.client.simxGetObjectHandle('RoboCamera', self.client.simxServiceCall() )
        self.client.simxGetJointPosition(self.frontLeft, self.client.simxDefaultSubscriber(self.l_F_enc) )
        self.client.simxGetJointPosition(self.frontRight, self.client.simxDefaultSubscriber(self.r_F_enc) )
        self.client.simxGetJointPosition(self.rearLeft, self.client.simxDefaultSubscriber(self.l_R_enc) )
        self.client.simxGetJointPosition(self.rearRight, self.client.simxDefaultSubscriber(self.r_R_enc) )
        self.client.simxGetJointPosition(self.steeringLeft, self.client.simxDefaultSubscriber(self.l_S_enc) )
        self.client.simxGetJointPosition(self.steeringRight, self.client.simxDefaultSubscriber(self.r_S_enc) )
        self.client.simxGetSimulationTime( self.client.simxDefaultSubscriber(self.getSimTime) )
        self.client.simxGetSimulationState( self.client.simxDefaultSubscriber(self.getSimState) )
        self.client.simxGetVisionSensorImage(self.cam, False, self.client.simxDefaultSubscriber(self.getCameraImage,1) )
    
    def setSteeringAngleDeg(self, pos=0):
        p=pos
        if p>45:
            p=45
        if p<-45:
            p=-45
        desiredSteeringAngle=p*math.pi/180
        steeringAngleLeft=math.atan( self.l/ ( -self.d+self.l/math.tan ( desiredSteeringAngle ) ) )
        steeringAngleRight=math.atan(self.l/(self.d+self.l/math.tan(desiredSteeringAngle)))
        errPos1=self.client.simxSetJointTargetPosition(self.steeringLeft, steeringAngleLeft, self.client.simxDefaultPublisher() )
        errPos2=self.client.simxSetJointTargetPosition(self.steeringRight, steeringAngleRight, self.client.simxDefaultPublisher() )
        #return errPos1+errPos2
    
    def setSteeringAngleRad(self, pos=0):
        p=pos
        if p>45*math.pi/180:
            p=45*math.pi/180
        if p<-45*math.pi/180:
            p=-45*math.pi/180
        desiredSteeringAngle=p
        steeringAngleLeft=math.atan(self.l/(-self.d+self.l/math.tan(desiredSteeringAngle)))
        steeringAngleRight=math.atan(self.l/(self.d+self.l/math.tan(desiredSteeringAngle)))
        errPos1=self.client.simxSetJointTargetPosition(self.steeringLeft, steeringAngleLeft, self.client.simxDefaultPublisher() )
        errPos2=self.client.simxSetJointTargetPosition(self.steeringRight, steeringAngleRight, self.client.simxDefaultPublisher() )
        #return errPos1+errPos2
    
    def setSpeedDeg(self, spd=0):
        p=spd
        if p>720:
            p=720
        if p<-720:
            p=-720
        desiredWheelRotSpeed=p*math.pi/180
        errPos1=self.client.simxSetJointTargetVelocity(self.rearLeft, desiredWheelRotSpeed, self.client.simxDefaultPublisher() )
        errPos2=self.client.simxSetJointTargetVelocity(self.rearRight, desiredWheelRotSpeed, self.client.simxDefaultPublisher() )
        #return errPos1 and errPos2
    
    def setSpeedRad(self, spd=0):
        p=spd
        if p>720*math.pi/180:
            p=720*math.pi/180
        if p<-720*math.pi/180:
            p=-720*math.pi/180
        desiredWheelRotSpeed=p
        errPos1=self.client.simxSetJointTargetVelocity(self.rearLeft, desiredWheelRotSpeed, self.client.simxDefaultPublisher() )
        errPos2=self.client.simxSetJointTargetVelocity(self.rearRight, desiredWheelRotSpeed, self.client.simxDefaultPublisher() )
        #return errPos1 and errPos2
    
    def setSteeringAndSpeedDeg(self, pos=0, spd=0):
        self.setSteeringAngleDeg(pos)
        self.setSpeedDeg(spd)
        #return self.setPosX(posX) and self.setPosY(posY)
    
    def setSteeringAndSpeedRad(self, pos=0, spd=0):
        self.setSteeringAngleRad(pos)
        self.setSpeedRad(spd)
        #return self.setPosX(posX) and self.setPosY(posY)
    
    def getCameraImage(self, msg):
        if msg[0] and self.__simState>0:
            self.cam_image=msg[2]
    
    def setCameraResolution(self, x, y):
        resX=x
        resY=y
        if resX<1: resX=1
        if resX>1024: resX=1024
        if resY<1: resY=1
        if resY>1024: resY=1024
        errParX=self.client.simxSetObjectIntParameter(self.cam, 1002, resX, self.client.simxDefaultPublisher() )
        errParY=self.client.simxSetObjectIntParameter(self.cam, 1003, resY, self.client.simxDefaultPublisher() )

    def l_F_enc(self,msg):
        enc=0
        if msg[0] and self.__simState>0:
            enc=msg[1]*180.0/math.pi
            if enc - self.__lF_old > 180: self.__lF_rot=self.__lF_rot-1
            if enc - self.__lF_old <-180: self.__lF_rot=self.__lF_rot+1
            self.__lF_old = enc
            self.leftFront_enc = self.__lF_rot * 360 + enc

    def r_F_enc(self,msg):
        enc=0
        if msg[0] and self.__simState>0:
            enc=msg[1]*180.0/math.pi
            if enc - self.__rF_old > 180: self.__rF_rot=self.__rF_rot-1
            if enc - self.__rF_old <-180: self.__rF_rot=self.__rF_rot+1
            self.__rF_old = enc
            self.rightFront_enc = self.__rF_rot * 360 + enc

    def l_R_enc(self,msg):
        enc=0
        if msg[0] and self.__simState>0:
            enc=msg[1]*180.0/math.pi
            if enc - self.__lR_old > 180: self.__lR_rot=self.__lR_rot-1
            if enc - self.__lR_old <-180: self.__lR_rot=self.__lR_rot+1
            self.__lR_old = enc
            self.leftRear_enc = self.__lR_rot * 360 + enc

    def r_R_enc(self,msg):
        enc=0
        if msg[0] and self.__simState>0:
            enc=msg[1]*180.0/math.pi
            if enc - self.__rR_old > 180: self.__rR_rot=self.__rR_rot-1
            if enc - self.__rR_old <-180: self.__rR_rot=self.__rR_rot+1
            self.__rR_old = enc
            self.rightRear_enc = self.__rR_rot * 360 + enc

    def l_S_enc(self,msg):
        if msg[0] and self.__simState>0:
            self.leftSteering_enc=msg[1]*180.0/math.pi

    def r_S_enc(self,msg):
        if msg[0] and self.__simState>0:
            self.rightSteering_enc=msg[1]*180.0/math.pi

    
    def getSimTime(self, msg):
        if msg[0] and self.__simState>0:
            self.simTime=msg[1]
    
    def getSimState(self, msg):
        if msg[0]:
            self.__oldSimState=self.__simState
            self.__simState=msg[1]
        else:
            self.__simState=0
        if self.__simState==0 and self.__oldSimState>0:
            self.disconnect=True
            if self.stopDisconnect:
                sys.exit()
