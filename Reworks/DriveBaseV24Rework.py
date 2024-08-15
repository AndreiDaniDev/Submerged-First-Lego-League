import runloop, hub, motor, motor_pair, color, math, time
from hub import motion_sensor as gs

'''
    # ---> Defining the hub's motors & ports <---
    leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
    leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
    leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
'''

# ---> The class that has all acceleration methods (can be modified) <---
class SpeedMethods(object):
    @staticmethod
    def liniar(x: float) -> float:
        return x
    @staticmethod
    def easeOutIn(x: float) -> float:
        return 1 - math.pow(abs(2 * x - 1), 2)
    @staticmethod
    def constant(x: float) -> float:
        return 0


# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase(object):

    def __init__(self, leftMotor: int, rightMotor: int, centimeterToDegrees: float, brakeMethod) -> None:
        # ---> Defining the pair that is made out of [leftMotor, rightMotor] <---
        motor_pair.pair(motor_pair.PAIR_1, leftMotor, rightMotor)
        self.oneCentimeterToDegrees: float = abs(centimeterToDegrees)
        self.leftMotor = leftMotor; self.rightMotor = rightMotor
        self.pair: int = motor_pair.PAIR_1; self.brake = brakeMethod
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 100
        self.usualMaxSpeed: int = 1000; self.usualMinSpeed: int = 200
        motor.reset_relative_position(self.leftMotor, 0)
        motor.reset_relative_position(self.rightMotor, 0)
        gs.reset_yaw(0); # self.initRun()
        # ---> Already used when starting run 1 <---

        self.speed: int
        self.startSpeed: int
        self.endSpeed: int
        self.diffSpeed: int
        self.leftSpeed: int
        self.rightSpeed: int

        self.approximationScale: int
        self.currentApproximationValue: float
        self.previousApproximationValue: float
        self.outputValue: int

        self.currentReachDistance: int
        self.previousReachDistance: int
        self.reachDistance: int
        self.reachAngle: int

        self.position: int
        self.previousPosition: int

        self.iterator: int
        self.previousFunction: int
        self.wasStuck: bool

        self.kp: float
        self.ki: float
        self.kd: float
        self.dt: int
        self.kLeft: float
        self.kRight: float

        self.angle: int
        self.error: float
        self.previousError: float
        self.proportional: float
        self.integral: float
        self.derivative: float
        self.controllerOutput: float

        return None

    def initRun(self) -> None:
        # ---> Limiting and applying speeds <---
        self.speed: int = 0
        self.startSpeed: int = 0
        self.endSpeed: int = 0
        self.diffSpeed: int = 0
        self.leftSpeed: int = 0
        self.rightSpeed: int = 0

        self.approximationScale: int = 0
        self.currentApproximationValue: float = 0
        self.previousApproximationValue: float = 0
        self.outputValue: int = 0

        self.currentReachDistance: int = 0
        self.previousReachDistance: int = 0
        self.reachDistance: int = 0
        self.reachAngle: int = 0

        self.position: int = 0
        self.previousPosition: int = 0

        self.iterator: int = 0
        self.previousFunction: int = 0
        self.wasStuck: bool = False

        self.kp: float = 0
        self.ki: float = 0
        self.kd: float = 0
        self.dt: int = 0
        self.kLeft: float = 0
        self.kRight: float = 0

        self.angle: int = 0
        self.error: float = 0
        self.previousError: float = 0
        self.proportional: float = 0
        self.integral: float = 0
        self.derivative: float = 0
        self.controllerOutput: float = 0

        motor.reset_relative_position(self.leftMotor, 0)
        motor.reset_relative_position(self.rightMotor, 0)
        gs.reset_yaw(0)

        return None

    def limit(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [ valueMin, valueMax ] and return it <---
        return min(max(value, valueMin), valueMax)

    def limitAbs(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [ valueMin, valueMax ] based on its sign and return it <---
        return (min(max(value, valueMin), valueMax) * (value / abs(value)) if (value != 0) else 0)

    def getTimeDistance(self) -> float: # Return how far the robot has travelled based on the given distance
        return (self.position - self.previousReachDistance) / self.currentReachDistance

    def getTimeAngle(self) -> float: # return how far the robot has rotated based on the given angle
        return (self.angle) / (self.reachAngle)

    def getApproximation(self, value: float, approximationScale: int) -> float:
        # Return the approximated value based on a simple formula and the given scale
        return math.floor(value * approximationScale) / approximationScale

    def getSpeed(self, method, addition: int, divizor: int) -> int:
        if(self.currentApproximationValue != self.previousApproximationValue):
            self.outputValue = self.diffSpeed * method((self.currentApproximationValue + addition) / divizor)
            self.previousApproximationValue = self.currentApproximationValue
        return self.outputValue

    def PID_Controller(self, error: float, integralLimit: int, sign: int) -> None:

        self.error = (error)
        self.proportional = (self.error)
        self.integral += (self.error * self.dt)
        self.derivative = (self.error - self.previousError) / self.dt
        self.previousError = (self.error)

        self.integral = self.limit(self.integral, integralLimit, -integralLimit)

        self.controllerOutput = (
            self.proportional * self.kp +
            self.integral * self.ki +
            self.derivative * self.kd
        )

        self.controllerOutput *= sign

        return None

    async def gyroForwards(self, distance: float, startSpeed: int, endSpeed: int, *, kp: float = 0.5, ki: float = 0.2, kd: float = 0.75, dt: int = 1, integralLimit: int = 250, stallDetectionIterator: int = 250, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.liniar, stop: bool = True) -> None:

        if(self.wasStuck == True and self.previousFunction == 1):
            print("Canceled function"); return None

        self.kp = abs(kp); self.ki = abs(ki); self.kd = abs(kd); self.dt = abs(1 if (dt == 0) else dt)
        # stallDetectionIterator = abs(1 if (stallDetectionIterator == 0) else stallDetectionIterator)
        # accelerationScale = abs(1 if (accelerationScale == 0) else accelerationScale)
        # integralLimit = abs(integralLimit); self.wasStuck = False

        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle); self.reachAngle = 0
        self.angle = gs.tilt_angles()[0]
        self.approximationScale = abs(accelerationScale)
        self.currentReachDistance = int(abs(distance * self.oneCentimeterToDegrees))
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed
        self.outputValue = 0; self.iterator = 1
        self.previousApproximationValue = 0
        self.wasStuck = False

        if(self.startSpeed > self.pairMaxSpeed): self.startSpeed = self.usualMaxSpeed
        if(self.startSpeed < self.pairMinSpeed): self.startSpeed = self.usualMinSpeed
        if(self.endSpeed > self.pairMaxSpeed): self.endSpeed = self.usualMaxSpeed
        if(self.endSpeed < self.pairMinSpeed): self.endSpeed = self.usualMinSpeed

        if(self.previousFunction == 1):
            self.previousReachDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
        else:
            motor.reset_relative_position(self.leftMotor, 0)
            motor.reset_relative_position(self.rightMotor, 0)
            self.reachDistance = self.currentReachDistance
            self.previousReachDistance = 0
            self.previousPosition = 0
            self.position = 0

            self.proportional = 0
            self.integral = 0
            self.derivative = 0
            self.error = 0
            self.previousError = 0
            self.controllerOutput = 0

        await runloop.sleep_ms(25)

        while(self.position < self.reachDistance):
            self.angle = gs.tilt_angles()[0] # We only read once and store the value
            self.PID_Controller(self.reachAngle - self.angle, integralLimit, 1)
            # self.controllerOutput - The correction that needs to be applied

            self.currentApproximationValue = self.getApproximation(self.getTimeDistance(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, 0, 1))
            self.leftSpeed = int(self.limit(int(self.speed - self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))
            self.rightSpeed = int(self.limit(int(self.speed + self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))

            # Object Stall Detection
            self.position = (abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor)))
            if(self.iterator % stallDetectionIterator == 0):
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(dt); self.iterator += 1

        if(stop == True or self.wasStuck == True):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 1

        print("Gyro Forwards Complete")

        return None

    async def gyroBackwards(self, distance: float, startSpeed: int, endSpeed: int, *, kp: float = 0.5, ki: float = 0.2, kd: float = 0.75, dt: int = 1, integralLimit: int = 250, stallDetectionIterator: int = 250, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.liniar, stop: bool = True) -> None:

        if(self.wasStuck == True and self.previousFunction == 2):
            print("Canceled function"); return None

        self.kp = abs(kp); self.ki = abs(ki); self.kd = abs(kd); self.dt = abs(1 if (dt == 0) else dt)
        # stallDetectionIterator = abs(1 if (stallDetectionIterator == 0) else stallDetectionIterator)
        # accelerationScale = abs(1 if (accelerationScale == 0) else accelerationScale)
        # integralLimit = abs(integralLimit); self.wasStuck = False

        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle); self.reachAngle = 0
        self.angle = gs.tilt_angles()[0]
        self.approximationScale = abs(accelerationScale)
        self.currentReachDistance = int(abs(distance * self.oneCentimeterToDegrees))
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed
        self.outputValue = 0; self.iterator = 1
        self.previousApproximationValue = 0
        self.wasStuck = False

        if(self.startSpeed > self.pairMaxSpeed): self.startSpeed = self.usualMaxSpeed
        if(self.startSpeed < self.pairMinSpeed): self.startSpeed = self.usualMinSpeed
        if(self.endSpeed > self.pairMaxSpeed): self.endSpeed = self.usualMaxSpeed
        if(self.endSpeed < self.pairMinSpeed): self.endSpeed = self.usualMinSpeed

        if(self.previousFunction == 2):
            self.previousReachDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
        else:
            motor.reset_relative_position(self.leftMotor, 0)
            motor.reset_relative_position(self.rightMotor, 0)
            self.reachDistance = self.currentReachDistance
            self.previousReachDistance = 0
            self.previousPosition = 0
            self.position = 0

            self.proportional = 0
            self.integral = 0
            self.derivative = 0
            self.error = 0
            self.previousError = 0
            self.controllerOutput = 0

        while(self.position < self.reachDistance):
            self.angle = gs.tilt_angles()[0] # We only read once and store the value
            self.PID_Controller(self.reachAngle - self.angle, integralLimit, -1)
            # self.controllerOutput - The correction that needs to be applied

            self.currentApproximationValue = self.getApproximation(self.getTimeDistance(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, 0, 1))
            self.leftSpeed = -int(self.limit(int(self.speed - self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))
            self.rightSpeed = -int(self.limit(int(self.speed + self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))

            # Object Stall Detection
            self.position = (abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor)))
            if(self.iterator % stallDetectionIterator == 0):
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(dt); self.iterator += 1

        if(stop == True or self.wasStuck == True):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 2

        print("Gyro Backwards Complete")

        return None

    async def arcTurnForwards(self) -> None:
        # https://www.physicsforums.com/threads/turning-radius-and-wheel-speed-difference.249401/
        return None

    async def turnLeft(self, angle: float, startSpeed: int, endSpeed: int, *, kLeft: float = -1, kRight: float = 1, stallDetectionIterator: int = 250, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.liniar, addition: int = 0, divizor: int = 1, stop: bool = True):

        if(self.wasStuck and kLeft == self.kLeft and kRight == self.kRight):
            print("Canceled function"); return None

        if(((kLeft > 0) and (kRight > 0)) or ((kLeft < 0) and (kRight < 0)) or (kLeft == kRight)):
            print("Error - kLeft and kRight are of the same sign"); return None

        angle = min(abs(angle), 165)

        if((kLeft > 0 and kRight == 0) or (kLeft > 0 and kRight < 0) or (kLeft == 0 and kRight < 0)):
            # ---> When the robot is going on the outside (large path) <--- + ---> I have no idea on why it isn't working <---
            print("Turn on the longer path -> 3 new turns")
            await self.turnRight((360 - angle) / 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.turnRight((360 - angle) / 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.turnRight((360 - angle) / 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)

        self.reachAngle = math.floor(angle * 10)
        self.kLeft = kLeft; self.kRight = kRight

        self.startSpeed = abs(startSpeed)
        self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed

        self.approximationScale = accelerationScale
        self.currentApproximationValue = 0
        self.previousApproximationValue = 0
        self.outputValue = 0

        self.position = 0; self.previousPosition = 0
        self.iterator = 1; self.wasStuck = False

        motor.reset_relative_position(self.leftMotor, 0)
        motor.reset_relative_position(self.rightMotor, 0)

        await runloop.sleep_ms(25)

        self.angle = gs.tilt_angles()[0]

        while(self.angle < self.reachAngle - 5):
            self.angle = gs.tilt_angles()[0]

            self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

            self.leftSpeed = int(self.speed * self.kLeft)
            self.rightSpeed = int(self.speed * self.kRight)

            # Object Stall Detection
            self.position = (abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor)))
            if(self.iterator % stallDetectionIterator == 0):
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(1); self.iterator += 1

        if(stop == True or self.wasStuck == True):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 3

        print("Turn Left Completed", addition)

        return None

    async def turnRight(self, angle: float, startSpeed: int, endSpeed: int, *, kLeft: float = 1, kRight: float = -1, stallDetectionIterator: int = 250, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.liniar, addition: int = 0, divizor: int = 1, stop: bool = True):

        if(self.wasStuck and kLeft == self.kLeft and kRight == self.kRight):
            print("Canceled function"); return None

        if(((kLeft > 0) and (kRight > 0)) or ((kLeft < 0) and (kRight < 0)) or (kLeft == kRight)):
            print("Error - kLeft and kRight are of the same sign"); return None

        angle = min(abs(angle), 165)

        if((kLeft < 0 and kRight == 0) or (kLeft < 0 and kRight > 0) or (kLeft == 0 and kRight > 0)):
            # ---> When the robot is going on the outside (large path) <--- + ---> I have no idea on why it isn't working <---
            print("Turn on the longer path -> 3 new turns")
            await self.turnLeft((360 - angle) / 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.turnLeft((360 - angle) / 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.turnLeft((360 - angle) / 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)

        self.reachAngle = -math.floor(angle * 10)
        self.kLeft = kLeft; self.kRight = kRight

        self.startSpeed = abs(startSpeed)
        self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed

        self.approximationScale = accelerationScale
        self.currentApproximationValue = 0
        self.previousApproximationValue = 0
        self.outputValue = 0

        self.position = 0; self.previousPosition = 0
        self.iterator = 1; self.wasStuck = False

        motor.reset_relative_position(self.leftMotor, 0)
        motor.reset_relative_position(self.rightMotor, 0)

        await runloop.sleep_ms(25)

        self.angle = gs.tilt_angles()[0]

        while(self.angle > self.reachAngle + 5):
            self.angle = gs.tilt_angles()[0]

            self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

            self.leftSpeed = int(self.speed * self.kLeft)
            self.rightSpeed = int(self.speed * self.kRight)

            # Object Stall Detection
            self.position = (abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor)))
            if(self.iterator % stallDetectionIterator == 0):
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(1); self.iterator += 1

        if(stop == True or self.wasStuck == True):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 4

        print("Turn Right Completed", addition)

        return None

# ---> The main program <---
async def main():
    driveBase = DriveBase(hub.port.B, hub.port.D, 20.45, motor.SMART_BRAKE); driveBase.initRun()

    ''' # ---> Standard Test for moving forwards & backwards <---
    await driveBase.gyroForwards(10, 200, 500, accelerationMethod = SpeedMethods.easeOutIn)
    await driveBase.gyroBackwards(10, 200, 500, accelerationMethod = SpeedMethods.easeOutIn)
    '''

    ''' 
    # ---> Standard Test for turns on the shortest path <---
    await driveBase.turnLeft(90, 200, 500, kLeft = -1, kRight = 1, accelerationMethod = SpeedMethods.easeOutIn)
    await runloop.sleep_ms(1000)
    await driveBase.turnRight(90, 200, 500, kLeft = 1, kRight = -1, accelerationMethod = SpeedMethods.easeOutIn)

    await runloop.sleep_ms(5000)

    await driveBase.turnLeft(90, 200, 500, kLeft = 0, kRight = 1, accelerationMethod = SpeedMethods.easeOutIn)
    await runloop.sleep_ms(1000)
    await driveBase.turnRight(90, 200, 500, kLeft = 1, kRight = 0, accelerationMethod = SpeedMethods.easeOutIn)

    await runloop.sleep_ms(5000)

    await driveBase.turnLeft(90, 200, 500, kLeft = -1, kRight = 0, accelerationMethod = SpeedMethods.easeOutIn)
    await runloop.sleep_ms(1000)
    await driveBase.turnRight(90, 200, 500, kLeft = 0, kRight = -1, accelerationMethod = SpeedMethods.easeOutIn)
    '''

    '''
    # ---> Standard Test for turns on the longer path <---
    await driveBase.turnLeft(90, 200, 500, kLeft = 1, kRight = 0, accelerationMethod = SpeedMethods.easeOutIn)
    await runloop.sleep_ms(1000)
    await driveBase.turnRight(90, 200, 500, kLeft = 0, kRight = 1, accelerationMethod = SpeedMethods.easeOutIn)

    await runloop.sleep_ms(5000)

    await driveBase.turnLeft(90, 200, 500, kLeft = 1, kRight = -1, accelerationMethod = SpeedMethods.easeOutIn)
    await runloop.sleep_ms(1000)
    await driveBase.turnRight(90, 200, 500, kLeft = -1, kRight = 1, accelerationMethod = SpeedMethods.easeOutIn)

    await runloop.sleep_ms(5000)

    await driveBase.turnLeft(90, 200, 500, kLeft = 0, kRight = -1, accelerationMethod = SpeedMethods.easeOutIn)
    await runloop.sleep_ms(1000)
    await driveBase.turnRight(90, 200, 500, kLeft = -1, kRight = 0, accelerationMethod = SpeedMethods.easeOutIn)
    '''
    
    ''' 
    # ---> Standard Test to see if the robot can make a square correctly <---
    for x in range(0, 4):
        await driveBase.gyroForwards(50, 200, 500, kp = 0.5, ki = 0.2, kd = 2, accelerationMethod = SpeedMethods.easeOutIn)
        await runloop.sleep_ms(100)
        await driveBase.turnLeft(90, 200, 500, kLeft = 1, kRight = 0, accelerationMethod = SpeedMethods.easeOutIn)
        await runloop.sleep_ms(100)
    await driveBase.gyroForwards(25, 200, 500, kp = 0.5, ki = 0.2, kd = 2, accelerationMethod = SpeedMethods.easeOutIn)
    '''
    
runloop.run(main())
