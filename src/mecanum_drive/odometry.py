from __future__ import division
from math import pi, sin, cos
from mecanum_drive.encoder import Encoder
from mecanum_drive.pose import Pose

class Odometry:
    """Keeps track of the current position and velocity of a
    robot using differential drive.
    """

    def __init__(self):
        self.frontLeftEncoder = Encoder()
        self.frontRightEncoder = Encoder()
        self.rearLeftEncoder = Encoder()
        self.rearRightEncoder = Encoder()
        self.pose = Pose()
        self.lastTime = 0

    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation

    def setWheelSeparationLength(self, separation):
        self.wheelSeparationLength = separation

    def setTicksPerMeter(self, ticks):
        self.ticksPerMeter = ticks
        
    def setEncoderRange(self, low, high):
        self.frontLeftEncoder.setRange(low, high)
        self.frontRightEncoder.setRange(low, high)
        self.rearLeftEncoder.setRange(low, high)
        self.rearRightEncoder.setRange(low, high)

    def setTime(self, newTime):
        self.lastTime = newTime
        
    def updateWheels(self, fl, fr, rl, rr):
        self.frontLeftEncoder.update(fl)
        self.frontRightEncoder.update(fr)        
        self.rearLeftEncoder.update(rl)        
        self.rearRightEncoder.update(rr)        

    def updatePose(self, newTime):
        """Updates the pose based on the accumulated encoder ticks
        of the four mecanum wheels.
        """
        frontLeftTravel = self.frontLeftEncoder.getDelta() / self.ticksPerMeter
        frontRightTravel = self.frontRightEncoder.getDelta() / self.ticksPerMeter
        rearLeftTravel = self.rearLeftEncoder.getDelta() / self.ticksPerMeter
        rearRightTravel = self.rearRightEncoder.getDelta() / self.ticksPerMeter
        deltaTime = newTime - self.lastTime

        deltaXTravel = (frontLeftTravel + frontRightTravel + rearLeftTravel + rearRightTravel) / 4.0
        deltaYTravel = (-frontLeftTravel + frontRightTravel + rearLeftTravel - rearRightTravel) / 4.0
        deltaTheta = (-frontLeftTravel + frontRightTravel - rearLeftTravel + rearRightTravel) / (2 * (self.wheelSeparation + self.wheelSeparationLength))

        self.pose.x += deltaXTravel*cos(self.pose.theta) - deltaYTravel*sin(self.pose.theta)
        self.pose.y += deltaYTravel*cos(self.pose.theta) + deltaXTravel*sin(self.pose.theta)
        self.pose.theta = (self.pose.theta + deltaTheta) % (2*pi)
        self.pose.xVel = deltaXTravel / deltaTime if deltaTime > 0 else 0.
        self.pose.yVel = deltaYTravel / deltaTime if deltaTime > 0 else 0.
        self.pose.thetaVel = deltaTheta / deltaTime if deltaTime > 0 else 0.

        self.lastTime = newTime

    def getPose(self):
        return self.pose;

    def setPose(self, newPose):
        self.pose = newPose
