from __future__ import division

class MotorCommand:
    """Holds motor control commands for a differential-drive robot.
    """

    def __init__(self):
        self.frontLeft = 0
        self.frontRight = 0
        self.rearLeft = 0
        self.rearRight = 0
        

class Controller:
    """Determines motor speeds to accomplish a desired motion.
    """

    def __init__(self):
        # Set the max motor speed to a very large value so that it
        # is, essentially, unbound.
        self.maxMotorSpeed = 10000 # ticks/s

    def getSpeeds(self, linearXSpeed, linearYSpeed, angularSpeed):
    
    
        # print(linearXSpeed,linearYSpeed,angularSpeed)
    
        WHEEL_SEPARATION_WIDTH = 0.1
        WHEEL_SEPARATION_LENGTH = 0.13
        
        speeds = MotorCommand()
        
        speeds.frontLeft = self.ticksPerMeter * (linearXSpeed - linearYSpeed - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        speeds.frontRight = self.ticksPerMeter * (linearXSpeed + linearYSpeed + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        speeds.rearLeft = self.ticksPerMeter * (linearXSpeed + linearYSpeed - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        speeds.rearRight = self.ticksPerMeter * (linearXSpeed - linearYSpeed + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        
        # print(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight)
        

        # Adjust speeds if they exceed the maximum.
        if max(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight) > self.maxMotorSpeed:
            factor = self.maxMotorSpeed / max(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight)
            speeds.frontLeft *= factor
            speeds.frontRight *= factor
            speeds.rearLeft *= factor
            speeds.rearRight *= factor

        speeds.frontLeft = int(speeds.frontLeft)
        speeds.frontRight = int(speeds.frontRight)
        speeds.rearLeft = int(speeds.rearLeft)
        speeds.rearRight = int(speeds.rearRight)
        return speeds

    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation

    def setMaxMotorSpeed(self, limit):
        self.maxMotorSpeed = limit

    def setTicksPerMeter(self, ticks):
        self.ticksPerMeter = ticks
