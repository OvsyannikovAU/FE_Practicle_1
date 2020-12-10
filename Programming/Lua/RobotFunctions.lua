desiredSteeringAngle=0
desiredWheelRotSpeed=0
d=0.0604 -- 2*d=distance between left and right wheels
l=0.2062 -- l=distance between front and read wheels

function roboInit()
	steeringLeft=sim.getObjectHandle('SteeringLeft')
    steeringRight=sim.getObjectHandle('SteeringRight')
	rearLeft=sim.getObjectHandle('RearAxisLeft')
	rearRight=sim.getObjectHandle('RearAxisRight')
    cam=sim.getObjectHandle('RoboCamera')
	desiredSteeringAngle=0
	desiredWheelRotSpeed=0
	d=0.0604 -- 2*d=distance between left and right wheels
	l=0.2062 -- l=distance between front and read wheels
end

function setSteeringAngleDeg(pos)
    local p=pos
    if p>45 then p=45 end
    if p<-45 then p=-45 end
    desiredSteeringAngle=p*math.pi/180	
	steeringAngleLeft=math.atan(l/(-d+l/math.tan(desiredSteeringAngle)))
    steeringAngleRight=math.atan(l/(d+l/math.tan(desiredSteeringAngle)))
    sim.setJointTargetPosition(steeringLeft,steeringAngleLeft)
    sim.setJointTargetPosition(steeringRight,steeringAngleRight)
end

function setSteeringAngleRad(pos)
    local p=pos
    if p>45*math.pi/180 then p=45*math.pi/180 end
    if p<-45*math.pi/180 then p=-45*math.pi/180 end
    desiredSteeringAngle=p
	steeringAngleLeft=math.atan(l/(-d+l/math.tan(desiredSteeringAngle)))
    steeringAngleRight=math.atan(l/(d+l/math.tan(desiredSteeringAngle)))
    sim.setJointTargetPosition(steeringLeft,steeringAngleLeft)
    sim.setJointTargetPosition(steeringRight,steeringAngleRight)
end

function setSpeedRad(spd)
    local p=spd
    if p>720*math.pi/180 then p=720*math.pi/180 end
    if p<-720*math.pi/180 then p=-720*math.pi/180 end
    desiredWheelRotSpeed=p
	sim.setJointTargetVelocity(rearLeft,desiredWheelRotSpeed)
    sim.setJointTargetVelocity(rearRight,desiredWheelRotSpeed)
end

function setSpeedDeg(spd)
    local p=spd
    if p>720 then p=720 end
    if p<-720 then p=-720 end
    desiredWheelRotSpeed=p*math.pi/180
	sim.setJointTargetVelocity(rearLeft,desiredWheelRotSpeed)
    sim.setJointTargetVelocity(rearRight,desiredWheelRotSpeed)
end

function setSteeringAndSpeedDeg(pos, spd)
	setSteeringAngleDeg(pos)
	setSpeedDeg(spd)
end

function setSteeringAndSpeedRad(pos, spd)
	setSteeringAngleRad(pos)
	setSpeedRad(spd)
end

function getCameraImage()
    local img=sim.getVisionSensorImage(cam, 0,0,0,0,0)
    return img
end

function getCameraPixel(x, y)
    local img=sim.getVisionSensorImage(cam, x,y,1,1,0)
    return img
end

function getCameraField(x, x_size, y, y_size)
    local img=sim.getVisionSensorImage(cam, x,y,x_size,y_size,0)
    return img
end

function setCameraResolution(x, y)
	local resX=x
	local resY=y
	if resX<1 then resX=1 end
	if resX>1024 then resY=1024 end
	if resY<1 then resY=1 end
	if resY>1024 then resY=1024 end
	local resultX = sim.setObjectInt32Parameter(cam,sim.visionintparam_resolution_x, resX)
	local resultY = sim.setObjectInt32Parameter(cam,sim.visionintparam_resolution_y, resY)
	return resultX and resultY
end