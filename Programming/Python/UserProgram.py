import os
os.add_dll_directory(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu')

import b0RemoteApi
from RoboFunctions import RoboCar
import time
import math


with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi_FE') as client:
    doNextStep=True
    robot=RoboCar(client)
    step=0 #counter steps of simulation
    
    def init():
	#Example of using standart functions of robot:
        
        simTime=robot.simTime # get time of simulation
        #simTime - float number in seconds, e.g. simPime=0.563 [s]
        
        #Robot have a camera (resolution 512 x 512 pixeles):
        #img=robot.cam_image #get full camera image in RGB mode, resolution 512 x 512 pixeles
        #Return number array with lenght= 512*512*3 = 786 432 values. Array struct:
        #{pix_0x0_red, pix_0x0_green, pix_0x0_blue, pix_0x1_red, pix_0x1_green, pix_0x1_blue...}
        #Each value in range 0..1, e.g.
        #{0.80000001192093, 0.7843137383461, 0.76470589637756...}
        
        #resX=512 #frame width, range 1..1024 pixels
        #resY=512 #frame width, range 1..1024 pixels
        #robot.setCameraResolution(resX, resY) #set resolution of camera

        #encoders, values in degrees
        #robot.leftFront_enc
        #robot.rightFront_enc
        #robot.leftRear_enc
        #robot.rightRear_enc
        #robot.leftSteering_enc
        #robot.rightSteering_enc
        
        #Move robot:
        #angleDeg=15 # degrees, range -45..45
        #speedDeg=720 # degrees/sec, range -720..720
        #robot.setSteeringAngleDeg(angleDeg) # set steering angle to position
        #robot.setSpeedDeg(speedDeg) # set speed of robot
        #robot.setSteeringAndSpeedDeg(angleDeg, speedDeg)
	
        #angleRad=angleDeg*math.pi/180 # radian, range -0.7854..0.7854
        #speedRad=speedDeg*math.pi/180 # radian/sec, range -12.5664..12.5664
        #robot.setSteeringAngleRad(angleRad) # set steering angle to position
        #robot.setSpeedRad(speedRad) # set speed of robot
        #robot.setSteeringAndSpeedRad(angleRad, speedRad)
        
        #robot.stopDisconnect=True #disconnect from simulator stoped the programm
        time.time()
        
	#See also:
	#ReadMe for information about robot and simulator
	#https://coppeliarobotics.com/helpFiles/en/b0RemoteApi-python.htm - list of all Python B0 remote API function
    
    def simulationStepStarted(msg):
        #inactivity
        time.time()
    
    def simulationStepDone(msg):
        #inactivity
        global doNextStep #code section to
        doNextStep=True   #sinch with main thread (main loop)
    
    def cleanup():
	#inactivity
        time.time()
    
    client.simxSynchronous(True)
    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))	
    res=client.simxStartSimulation(client.simxDefaultPublisher())
    init()
    #Put your main action here:
    
    startTime=time.time()
    startStep=step
    while (not robot.disconnect) and robot.simTime<120: #repeat loop 120 sec (time from simulator)
        #other options:
        #time.time()<startTime+5 #repeat loop 5 sec (real time)
        #step-startStep<100 #repeat loop 100 steps
        if doNextStep:
            doNextStep=False
            #put you synchronously executing code here:
            
            step=step+1
            client.simxSynchronousTrigger()
        client.simxSpinOnce()
    
    #put you asynchronously executing code here
    #for example:
    #time.sleep(long_time)
    
    
    
    #End of simulation:
    cleanup()
    if not robot.disconnect:
        client.simxStopSimulation(client.simxDefaultPublisher())
    else:
        print('Simulation was stoped and client was disconnected!')
