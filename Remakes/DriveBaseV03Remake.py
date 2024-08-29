import runloop, hub, motor, motor_pair, color, math, time
from hub import motion_sensor as gs

# ---> The class that has all acceleration methods (can be modified) <---
class SpeedMethods(object):
    @staticmethod
    def liniar(x: float) -> float:
        return x
    @staticmethod
    def easeOutInQuad(x: float) -> float:
        return 1 - math.pow(abs(2 * x - 1), 2)
    @staticmethod
    def easeOutInCubic(x: float) -> float:
        return 1 - math.pow(abs(2 * x - 1), 3)

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase(object):

    def __init__(self, leftMotorDB: int, rightMotorDB: int, leftMotorSYS: int, rightMotorSYS: int, oneUnit: int, scale: int, brakeMethod) -> None:
        # ---> Defining the constant values that won't change in any run <---
        motor_pair.pair(motor_pair.PAIR_1, leftMotorDB, rightMotorDB)
        self.oneUnit: int = abs(oneUnit); self.scaleUnit: int = scale
        self.leftMotorDB = leftMotorDB; self.rightMotorDB = rightMotorDB
        self.leftMotorSYS = leftMotorSYS; self.rightMotorSYS = rightMotorSYS
        self.pair: int = motor_pair.PAIR_1; self.brake = brakeMethod
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 100
        self.usualMaxSpeed: int = 1000; self.usualMinSpeed: int = 200

        # ---> Defining the (global) values that will be used in the functions <---
        self.speed: int = 0; self.startSpeed: int = 0; self.endSpeed: int = 0
        self.diffSpeed: int = 0; self.leftSpeed: int = 0; self.rightSpeed: int = 0

        self.approximationScale: int = 0; self.currentApproximationValue: float = 0
        self.previousApproximationValue: float = 0; self.outputValue: int = 0

        self.currentReachDistance: int = 0; self.previousReachDistance: int = 0
        self.reachDistance: int = 0; self.reachAngle: int = 0

        self.position: int = 0; self.previousPosition: int = 0

        self.iterator: int = 0; self.previousFunction: int = 0

        self.kp: float = 0; self.ki: float = 0; self.kd: float = 0
        self.dt: int = 0; self.kLeft: float = 0; self.kRight: float = 0

        self.angle: int = 0; self.error: float = 0; self.previousError: float = 0
        self.proportional: float = 0; self.integral: float = 0
        self.derivative: float = 0; self.controllerOutput: float = 0

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)
        gs.reset_yaw(0); self.wasStuck: bool = False
        return None

    def initRun(self) -> None:
        # ---> This needs to be called at the start of every program / run <---

        self.speed = 0; self.startSpeed = 0; self.endSpeed = 0
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

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)
        gs.reset_yaw(0); self.wasStuck: bool = False

        return None

    def limit(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [ valueMin, valueMax ] and return it <---
        return min(max(value, valueMin), valueMax)

    def limitAbs(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [ valueMin, valueMax ] based on its sign and return it <---
        return (min(max(value, valueMin), valueMax) * (value / abs(value)) if (value != 0) else 0)

    def transformDistanceMM(self, distanceMM: int) -> int:
        return (abs(distanceMM) * self.oneUnit) // self.scaleUnit

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
        self.integral = self.limit(self.integral, integralLimit, -integralLimit)
        self.derivative = (self.error - self.previousError) / self.dt
        self.previousError = (self.error)

        self.controllerOutput = sign * (
            self.proportional * self.kp +
            self.integral * self.ki +
            self.derivative * self.kd
        )
        return None

    async def gyroForwards(self, distance: int, startSpeed: int, endSpeed: int, *, kp: float = 0.5, ki: float = 0.2, kd: float = 0.75, dt: int = 1, integralLimit: int = 250, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, stop: bool = True) -> None:

        if(self.wasStuck == True and self.previousFunction == 1):
            print("Canceled function"); return None

        self.kp = abs(kp); self.ki = abs(ki); self.kd = abs(kd); self.dt = abs(1 if (dt == 0) else dt)

        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle); self.reachAngle = 0
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.currentReachDistance = self.transformDistanceMM(distance)
        self.approximationScale = abs(accelerationScale)
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
            motor.reset_relative_position(self.leftMotorDB, 0)
            motor.reset_relative_position(self.rightMotorDB, 0)
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
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(dt); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 1

        print("Gyro Forwards Complete")

        return None

    async def gyroBackwards(self, distance: int, startSpeed: int, endSpeed: int, *, kp: float = 0.5, ki: float = 0.2, kd: float = 0.75, dt: int = 1, integralLimit: int = 250, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, stop: bool = True) -> None:

        if(self.wasStuck == True and self.previousFunction == 2):
            print("Canceled function"); return None

        self.kp = abs(kp); self.ki = abs(ki); self.kd = abs(kd); self.dt = abs(1 if (dt == 0) else dt)

        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle); self.reachAngle = 0
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.currentReachDistance = self.transformDistanceMM(distance)
        self.approximationScale = abs(accelerationScale)
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
            motor.reset_relative_position(self.leftMotorDB, 0)
            motor.reset_relative_position(self.rightMotorDB, 0)
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
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(dt); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 2

        print("Gyro Backwards Complete")

        return None

    async def turnLeft(self, angle: int, startSpeed: int, endSpeed: int, *, kLeft: int = -1, kRight: int = 1, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, addition: int = 0, divizor: int = 1, stop: bool = True):

        if(self.wasStuck and kLeft == self.kLeft and kRight == self.kRight):
            print("Canceled function"); return None

        if(((kLeft > 0) and (kRight > 0)) or ((kLeft < 0) and (kRight < 0)) or (kLeft == kRight)):
            print("Error - kLeft and kRight are of the same sign"); return None

        angle = min(abs(angle), 1650)

        if((kLeft > 0 and kRight == 0) or (kLeft > 0 and kRight < 0) or (kLeft == 0 and kRight < 0)):
            # ---> When the robot is going on the outside (large path) <--- + ---> I have no idea on why it isn't working <---
            print("Turn on the longer path -> 3 new turns")
            await self.turnRight((3600 - angle) // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.turnRight((3600 - angle) // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.turnRight((3600 - angle) // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)
        self.reachAngle = angle

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

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)

        await runloop.sleep_ms(25)

        self.angle = gs.tilt_angles()[0]

        while(self.angle < self.reachAngle - 5):
            self.angle = gs.tilt_angles()[0]

            self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

            self.leftSpeed = int(self.speed * self.kLeft)
            self.rightSpeed = int(self.speed * self.kRight)

            # Object Stall Detection
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(1); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 3

        print("Turn Left Completed", addition)

        return None

    async def turnRight(self, angle: int, startSpeed: int, endSpeed: int, *, kLeft: int = 1, kRight: int = -1, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, addition: int = 0, divizor: int = 1, stop: bool = True):

        if(self.wasStuck and kLeft == self.kLeft and kRight == self.kRight):
            print("Canceled function"); return None

        if(((kLeft > 0) and (kRight > 0)) or ((kLeft < 0) and (kRight < 0)) or (kLeft == kRight)):
            print("Error - kLeft and kRight are of the same sign"); return None

        angle = min(abs(angle), 1650)

        if((kLeft < 0 and kRight == 0) or (kLeft < 0 and kRight > 0) or (kLeft == 0 and kRight > 0)):
            # ---> When the robot is going on the outside (large path) <--- + ---> I have no idea on why it isn't working <---
            print("Turn on the longer path -> 3 new turns")
            await self.turnLeft((3600 - angle) // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.turnLeft((3600 - angle) // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.turnLeft((3600 - angle) // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)
        self.reachAngle = -(angle)

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

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)

        await runloop.sleep_ms(25)

        self.angle = gs.tilt_angles()[0]

        while(self.angle > self.reachAngle + 5):
            self.angle = gs.tilt_angles()[0]

            self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

            self.leftSpeed = int(self.speed * self.kLeft)
            self.rightSpeed = int(self.speed * self.kRight)

            # Object Stall Detection
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - stallDetectionThreshold < self.previousPosition):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(1); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 4

        print("Turn Right Completed", addition)

        return None

    async def ArcTurnForwards(self, radius: int, outerWheel: int, startSpeed: int, endSpeed: int) -> None:

        if(self.wasStuck and self.previousFunction == 5):
            print("Canceled Arc Turn Forwards"); return None

        if(outerWheel != self.leftMotorDB and outerWheel != self.rightMotorDB):
            print("Error - Outer Wheel is not a motor from the driveBase"); return None

        if(radius < 50): print("Radius is too small"); return None

        # if(startSpeed * (1000 * (radius - 40) // (radius + 40)) // 1000 > self.pairMaxSpeed):
        #    print("Error - startSpeed is too high"); return None

        # if(startSpeed * (1000 * (radius - 40) // (radius + 40)) // 1000 < self.pairMinSpeed):
        #    print("Error - startSpeed is too low"); return None

        # if(endSpeed * (1000 * (radius - 40) // (radius + 40)) // 1000 > self.pairMaxSpeed):
        #    print("Error - endSpeed is too high"); return None

        # if(endSpeed * (1000 * (radius - 40) // (radius + 40)) // 1000 > self.pairMinSpeed):
        #    print("Error - endSpeed is too low"); return None

        self.startSpeed = startSpeed; self.endSpeed = endSpeed
        self.diffSpeed = self.endSpeed - self.startSpeed

        if(outerWheel == self.leftMotorDB):
            self.kLeft = 1000
            self.kRight = (1000 * (radius - 40) // (radius + 40))
        else:
            self.kLeft = (1000 * (radius - 40) // (radius + 40))
            self.kRight = 1000

        print(self.kLeft, self.kRight)

        return None

driveBase = DriveBase(hub.port.B, hub.port.D, hub.port.A, hub.port.C, 2045, 1000, motor.SMART_BRAKE)

# ---> Approach 1 to implementing runs <---
class Programs(object):
    def __init__(self) -> None: pass

    async def Run1(self) -> None:
        driveBase.initRun()
        await driveBase.gyroForwards(100, 200, 500, kp = 0.4, ki = 0.2, kd = 0.2)
        await driveBase.gyroBackwards(100, 200, 500, kp = 0.4, ki = 0.2, kd = 0.2)

    async def Run2(self) -> None:
        driveBase.initRun()
        # ---> Standard Test for turns on the shortest path <---
        await driveBase.turnLeft(900, 200, 500, kLeft = -1, kRight = 1, accelerationMethod = SpeedMethods.easeOutInQuad)
        await runloop.sleep_ms(1000)
        await driveBase.turnRight(900, 200, 500, kLeft = 1, kRight = -1, accelerationMethod = SpeedMethods.easeOutInQuad)

        await runloop.sleep_ms(5000)

        await driveBase.turnLeft(900, 200, 500, kLeft = 0, kRight = 1, accelerationMethod = SpeedMethods.easeOutInQuad)
        await runloop.sleep_ms(1000)
        await driveBase.turnRight(900, 200, 500, kLeft = 1, kRight = 0, accelerationMethod = SpeedMethods.easeOutInQuad)

        await runloop.sleep_ms(5000)

        await driveBase.turnLeft(900, 200, 500, kLeft = -1, kRight = 0, accelerationMethod = SpeedMethods.easeOutInQuad)
        await runloop.sleep_ms(1000)
        await driveBase.turnRight(900, 200, 500, kLeft = 0, kRight = -1, accelerationMethod = SpeedMethods.easeOutInQuad)

    async def Run3(self) -> None:
        driveBase.initRun()
        # ---> Standard Test for turns on the longer path <---
        await driveBase.turnLeft(900, 200, 500, kLeft = 1, kRight = 0, accelerationMethod = SpeedMethods.easeOutInQuad)
        await runloop.sleep_ms(1000)
        await driveBase.turnRight(900, 200, 500, kLeft = 0, kRight = 1, accelerationMethod = SpeedMethods.easeOutInQuad)

        await runloop.sleep_ms(5000)

        await driveBase.turnLeft(900, 200, 500, kLeft = 1, kRight = -1, accelerationMethod = SpeedMethods.easeOutInQuad)
        await runloop.sleep_ms(1000)
        await driveBase.turnRight(900, 200, 500, kLeft = -1, kRight = 1, accelerationMethod = SpeedMethods.easeOutInQuad)

        await runloop.sleep_ms(5000)

        await driveBase.turnLeft(900, 200, 500, kLeft = 0, kRight = -1, accelerationMethod = SpeedMethods.easeOutInQuad)
        await runloop.sleep_ms(1000)
        await driveBase.turnRight(900, 200, 500, kLeft = -1, kRight = 0, accelerationMethod = SpeedMethods.easeOutInQuad)

    async def Run4(self) -> None:
        await driveBase.ArcTurnForwards(100, driveBase.leftMotorDB, 500, 700)

# ---> The main program <---
async def main() -> None:
    # ---> Approach 2 to implementing runs (SEPERATE PROGRAMS) (WORST HAND SCENARIO) <---
    programs = Programs()
    await programs.Run4()
    return None

runloop.run(main())
