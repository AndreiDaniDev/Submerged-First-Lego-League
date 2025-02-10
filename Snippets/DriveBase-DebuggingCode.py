import runloop, hub, motor, motor_pair, math, os, color
from hub import motion_sensor as gs
os.chdir('/flash') # Used for creating files

# ---> Class used for writing and reading data to files <---
class fileHandeler(object):
    def __init__(self, name: str) -> None:
        self.fileName = name

    def readFromFile(self):
        with open(self.fileName, 'r') as dataFile:
            value = dataFile.read()
        return value

    def writeToFile(self, value, nullchar: str = ' ') -> None:
        with open(self.fileName, 'w') as dataFile:
            dataFile.write(str(value) + nullchar)

# ---> The class that has all acceleration methods (can be modified) <---
class SpeedMethods(object):

    # ---> Slope & General Functions <---
    @staticmethod
    def slope(x: float, a: int, b: int): return a * x + b
    @staticmethod
    def easeBack(x: float, c: float): return (c + 1) * math.pow(x, 3) - c * math.pow(x, 2)
    @staticmethod
    def easeOutIn(x: float, c: int): return 1 - math.pow(abs(1 - 2 * x), c)
    @staticmethod
    def linear(x: float): return x
    @staticmethod # Combine for simple functions (that take only x as a parameter)
    def combine(x: float, a, b): return a(2 * x) / 2 if x <= 0.5 else (1 + b(2 * x - 1)) / 2

    @staticmethod
    def easeOutInLinear(x: float): return SpeedMethods.easeOutIn(x, 1)

    @staticmethod
    def easeOutInQuad(x: float): return SpeedMethods.easeOutIn(x, 2)

    @staticmethod
    def easeInQuad(x: float): return math.pow(x, 2)

    @staticmethod
    def easeOutQuad(x: float): return 1 - SpeedMethods.easeInQuad(1 - x)

    @staticmethod
    def easeInOutQuad(x: float): return SpeedMethods.combine(x, SpeedMethods.easeInQuad, SpeedMethods.easeOutQuad)
    # Constant1 = x, Constant2 = (x + 1)
    @staticmethod
    def easeInBack(x: float): return SpeedMethods.easeBack(x, 1.2)

    @staticmethod
    def easeOutBack(x: float): return 1 - SpeedMethods.easeInBack(1 - x)

    @staticmethod
    def easeInOutBack(x: float): return SpeedMethods.combine(x, SpeedMethods.easeInBack, SpeedMethods.easeOutBack)

    @staticmethod
    def easeOutInCubic(x: float): return SpeedMethods.easeOutIn(x, 3)

    @staticmethod
    def easeOutInQuart(x: float): return SpeedMethods.easeOutIn(x, 4)

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase(object):

    def __init__(self, leftMotorDB: int, rightMotorDB: int, leftMotorSYS: int, rightMotorSYS: int, oneUnit: int, scale: int) -> None:

        '''
        ---> Parameters Initialization Class <---> DB - driveBase, SYS - systems <---
        leftMotorDB: the port of the left motor from the driveBase
        rightMotorDB: the port of the right motor from the driveBase

        leftMotorSYS: the port of the left motor from the systems mounting system
        rightMotorSYS: the port of the right motor from the systems mounting system

        oneUnit: the circumference written as a fraction (oneUnit / scale = circumference)
        scale: the circumference written as a fraction (oneUnit / scale = circumference)
        brakeMethod: how we want the robot to break for all functions
        '''

        # ---> Defining the constant values that won't change in any run <---
        motor_pair.pair(motor_pair.PAIR_1, leftMotorDB, rightMotorDB)
        self.oneUnit: int = abs(oneUnit); self.scaleUnit: int = scale
        self.leftMotorDB = leftMotorDB; self.rightMotorDB = rightMotorDB
        self.leftMotorSYS = leftMotorSYS; self.rightMotorSYS = rightMotorSYS
        self.pair: int = motor_pair.PAIR_1
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 100
        self.usualMaxSpeed: int = 1000; self.usualMinSpeed: int = 200
        self.maxAngle = 3350; self.maxOneTurnAngle = 1650

        # ---> Defining the (global) values that will be used in the functions <---
        self.speed: int = 0; self.lowestSpeed: int = 0; self.highestSpeed: int = 0
        self.diffSpeed: int = 0; self.leftSpeed: int = 0; self.rightSpeed: int = 0

        self.approximationScale: int = 0; self.currentApproximationValue: float = 0
        self.previousApproximationValue: float = 0; self.outputValue: int = 0

        self.currentReachDistance: int = 0; self.previousReachDistance: int = 0
        self.reachDistance: int = 0; self.reachAngle: int = 0; self.targetedAngle: int = 0

        self.position: int = 0; self.previousPosition: int = 0

        self.iterator: int = 0; self.previousFunction: int = 0

        self.kp: int = 0; self.ki: int = 0; self.kd: int = 0
        self.dt: int = 0; self.kLeft: int = 0; self.kRight: int = 0

        self.angle: int = 0; self.error: int = 0; self.previousError: int = 0
        self.proportional: int = 0; self.integral: int = 0
        self.derivative: int = 0; self.controllerOutput: int = 0
        self.leftColorError: int = 0; self.rightColorError: int = 0

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)
        gs.reset_yaw(0); self.wasStuck: bool = False

        self.debuggingDATA = []
        self.debugfreq = 20
        self.debugframe = 0

        return None

    def initRun(self) -> None:
        # ---> This needs to be called at the start of every program / run <---
        # ---> In order for all variables to be reset <---

        self.speed = 0; self.lowestSpeed = 0; self.highestSpeed = 0
        self.diffSpeed = 0; self.leftSpeed = 0; self.rightSpeed = 0

        self.approximationScale = 0; self.currentApproximationValue = 0
        self.previousApproximationValue = 0; self.outputValue = 0

        self.currentReachDistance = 0; self.previousReachDistance = 0
        self.reachDistance = 0; self.reachAngle = 0

        self.position = 0; self.previousPosition = 0

        self.iterator = 0; self.previousFunction = 0

        self.kp = 0; self.ki = 0; self.kd = 0
        self.dt = 0; self.kLeft = 0; self.kRight = 0

        self.angle = 0; self.error = 0; self.previousError = 0
        self.proportional = 0; self.integral = 0
        self.derivative = 0; self.controllerOutput = 0

        self.leftColorError = 0; self.rightColorError = 0

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)
        gs.reset_yaw(0); self.wasStuck: bool = False

        return None

    def limit(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [valueMin, valueMax] and return it <---
        return min(max(value, valueMin), valueMax)

    def limitAbs(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [valueMin, valueMax] based on its sign and return it <---
        return (min(max(value, valueMin), valueMax) * (value / abs(value)) if (value != 0) else 0)

    def transformDistanceMM(self, distanceMM: int) -> int:
        return 2 * (abs(distanceMM) * self.oneUnit) // self.scaleUnit

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

    def PID_Controller(self, error: int, integralLimit: int, sign: int, scaleConstants: int) -> None:

        '''
        ---> Parameters for PID_Controller (Proportional - Integral - Derivative Controller) <---
        error: The error that was read by the sensor (in our case the gyroscopic sensor)
        integralLimit: A value that acts like a limit so that the integral won't overshoot from regular boundaries
        sign: A value that signifies the sign of the controller's output
        scaleConstants: kp, kd and ki are written as (x / scaleConstants, where x is either kp, kd or ki (integers))
        '''

        self.error = (error)
        self.proportional = (self.error)
        self.integral += (self.error * self.dt)
        self.integral = int(self.limit(self.integral, integralLimit, -integralLimit))
        self.derivative = (self.error - self.previousError)
        self.previousError = (self.error)

        self.controllerOutput = sign * (
            self.proportional * self.kp +
            self.integral * self.ki +
            self.derivative * self.kd // self.dt
        ) // scaleConstants

        return None

    async def gyroForwards(self, distance: int, lowestSpeed: int, highestSpeed: int, *, kp: int = 50, ki: int = 20, kd: int = 75, dt: int = 1, constantsScale: int = 100, integralLimit: int = 25000, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, easingMethod = SpeedMethods.easeOutInQuad, stop: bool = True, brakeMethod = motor.SMART_BRAKE) -> None:

        '''
        ---> Parameters Gyro Forwards / Backwards <---
        distance: How much we want the robot to travel (in milimeters)
        lowestSpeed: The lowest speed that the robot will reach (accelerating function - f(0))
        highestSpeed: The highest speed that the robot will reach

        kp, ki, kd: Positive coefficients that will be used in the Proportional - Integral - Derivative Controller (to calculate the correction that will be applied)
        dt: The time between the readings of two consecutive errors (acts like a delay -> slower response time, but sometimes this can help)
        constantScale: All of our constants are written as fractions (a / b = c) so that the PID Controller will have a faster response time.
        integralLimit: A value that acts like a limit so that the integral won't overshoot from regular boundaries
        stallDetectionIterator: This represents how frequently the controller checks if the robot is stuck or not (be aware that it's affected by dt)
        stallDetectionThreshold: If the difference between the current position and the last position is smaller that this value it means that the robot is stuck
        accelerationScale: It reprezents how many speed changes we want to have while travelling the given distance
        easingMethod: The graph (function) that will influence what the speed is based on the travelled distance until that moment
        stop: If we want to stop the robot after the funcion this will be true, otherwise false
        '''

        if(self.wasStuck == True and self.previousFunction == 1):
            print("Canceled function"); return None

        self.kp = abs(kp); self.ki = abs(ki); self.kd = abs(kd); self.dt = abs(dt)
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle); self.reachAngle = 0
        self.lowestSpeed = abs(lowestSpeed); self.highestSpeed = abs(highestSpeed)
        self.currentReachDistance = self.transformDistanceMM(distance)
        self.approximationScale = abs(accelerationScale)
        self.diffSpeed = self.highestSpeed - self.lowestSpeed
        self.outputValue = 0; self.iterator = 1
        self.previousApproximationValue = 0
        self.previousReachDistance = 0
        self.wasStuck = False

        # Make lowestSpeed & highestSpeed be in the range of [self.pairMinSpeed, self.pairMaxSpeed]
        if(self.lowestSpeed >= self.pairMaxSpeed): self.lowestSpeed = self.usualMaxSpeed
        if(self.lowestSpeed <= self.pairMinSpeed): self.lowestSpeed = self.usualMinSpeed
        if(self.highestSpeed >= self.pairMaxSpeed): self.highestSpeed = self.usualMaxSpeed
        if(self.highestSpeed <= self.pairMinSpeed): self.highestSpeed = self.usualMinSpeed

        # Precalculating the necessary values
        if(self.previousFunction == 1):
            self.previousReachDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
        else:
            motor.reset_relative_position(self.leftMotorDB, 0)
            motor.reset_relative_position(self.rightMotorDB, 0)
            self.reachDistance = self.currentReachDistance
            self.previousPosition = 0; self.position = 0

            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.error = 0
            self.previousError = 0; self.controllerOutput = 0

        await runloop.sleep_ms(25)

        while(self.position < self.reachDistance):
            self.angle = gs.tilt_angles()[0] # We only read once and store the value
            self.PID_Controller(self.reachAngle - self.angle, integralLimit, 1, constantsScale)
            # self.controllerOutput - The correction that needs to be applied

            # ---> Calculating the necessary speeds to fix the error and to accelerate <---
            self.currentApproximationValue = self.getApproximation(self.getTimeDistance(), self.approximationScale)
            self.speed = int(self.lowestSpeed + self.getSpeed(easingMethod, 0, 1))

            if(self.debugframe >= self.debugfreq):
                self.debuggingDATA.append([self.error, self.integral, self.speed])
                self.debugframe = 0
            self.debugframe += 1

            self.leftSpeed = int(self.limit(int(self.speed - self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))
            self.rightSpeed = int(self.limit(int(self.speed + self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))

            # ---> Object Stall Detection <---
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - self.previousPosition < stallDetectionThreshold):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(dt); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = brakeMethod)
        self.previousFunction = 1

        for x in self.debuggingDATA:
            print(x, end =  " ") # Error

        return None

driveBase = DriveBase(hub.port.A, hub.port.C, hub.port.D, hub.port.B, 2045, 1000)

# ---> The main program <---
async def main() -> None:
    await driveBase.gyroForwards(900, 250, 750, kp = 2250, ki = 12, kd = 5000, integralLimit = 1000000000, constantsScale = 10000)
    driveBase.debuggingDATA.clear()

runloop.run(main())
