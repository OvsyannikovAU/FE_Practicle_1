function sysCall_threadmain()
    --require "Programming/Lua/RobotFunctions"
    --roboInit()
    -- Put some initialization code here
    
    
    -- Put your main action here:
	
	
	--[[Example of using standart functions of robot:
    
    simTime=sim.getSimulationTime() -- get time of simulation
    --simTime - float number in seconds, e.g. simPime=0.563 [s]
    
    --Robot have a camera (resolution 512 x 512 pixeles):
    img=getCameraImage() --get full camera image in RGB mode, resolution 512 x 512 pixeles
    --Return number array with lenght= 512*512*3 = 786 432 values. Array struct:
    --{pix_0x0_red, pix_0x0_green, pix_0x0_blue, pix_0x1_red, pix_0x1_green, pix_0x1_blue...}
    -- Each value in range 0..1, e.g.
    --{0.80000001192093, 0.7843137383461, 0.76470589637756...}
    
    x=0
    y=0
    pixel=getCameraPixel(x,y) --get RGB pixel values from camera
    --Return number array with lenght 3. Array struct:
    --{pixel_red, pixel_green, pixel_blue}
    -- Each value in range 0..1, e.g.
    --{0.80000001192093, 0.7843137383461, 0.76470589637756}
    --Pixel numering in range 0..512 (camera resolution 512 x 512)
    
    x=0
    x_size=10
    y=0
    y_size=10
    imgField=getCameraField(x, x_size,y, y_size) --get  part of camera image in RGB mode
    --Must be x+x_size<512 and y+y_size<512
    --Pixel numering in range 0..512 (camera resolution 512 x 512)
    --Return number array with lenght x_size*y_size*3. Array struct:
    --{pix_X_Y_red, pix_X_Y_green, pix_X_Y_blue, pix_X_Y+1_red, pix_X_Y+1_green, pix_X_Y+1_blue...}
    -- Each value in range 0..1, e.g.
    --{0.80000001192093, 0.7843137383461, 0.76470589637756...}
    
    
    --Move robot:
    angleDeg=15 -- degrees, range -45..45
    speedDeg=60 -- degrees/sec, range -720..720
    setSteeringAngleDeg(angleDeg) --set steering angle to position
    setSpeedDeg(speedDeg) --set speed of robot
	setSteeringAndSpeedDeg(angleDeg, speedDeg)
	
	angleRad=angleDeg*math.pi/180 --radian, range -0.7854..0.7854
	speedRad=speedDeg*math.pi/180 --radian/sec, range -12.5664..12.5664
	setSteeringAngleRad(angleRad) --set steering angle to position
    setSpeedRad(speedRad) --set speed of robot
	setSteeringAndSpeedRad(angleRad, speedRad)
	    
    --See also:
    --ReadMe for information about robot and simulator
    --https://coppeliarobotics.com/helpFiles/en/apiFunctionListCategory.htm - list of all Lua regular API function
    ]]--
    
    -- sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches, wait in current step
    -- Do some one-step actions
    -- sim.switchThread() -- resume in next simulation step
end

function sysCall_cleanup()

end