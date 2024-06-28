import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

'''
    leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
    leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
    leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
'''

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase():
    def __init__(self, leftMotor, rightMotor) -> None:
        motor_pair.pair(motor_pair.PAIR_1, leftMotor, rightMotor)
        self.leftMotor = leftMotor; self.rightMotor = rightMotor
        self.pair: int = motor_pair.PAIR_1

        # ---> Parameters for bounding specific values & fixing errors <---
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 125
        self.reachDistance: int = 0; self.reachTarget: int = 0
        self.position: int = 0; self.lastPosition: int = 0
        self.maxCorrection: int = 0; self.minCorrection: int = 0

        # ---> Parameters for the Proportional - Integral - Derivative Controller <---
        self.error: float = 0; self.lastError: float = 0
        self.porportional: float = 0; self.integral: float = 0
        self.derivative: float = 0; self.correction: float = 0

        self.iterator: int = 0
        self.lastFunction: int = 0

        return None

    def initRun(self) -> None:
        #---> Called after switching and running another run <---
        self.reachDistance = 0; self.reachTarget = 0
        self.position = 0; self.lastPosition = 0
        self.lastFunction = 0; self.iterator = 0
        self.error = 0; self.lastError = 0
        self.proportional = 0; self.integral = 0
        self.derivative = 0; self.correction = 0
        self.maxCorrection = 0; self.minCorrection = 0

        return None

    def limitAbs(self, value: float | int, valueMax: float | int, valueMin: float | int) -> float | int:
        if(abs(value) > valueMax): value = valueMax * (value / abs(value))
        if(abs(value) < valueMin): value = valueMin * (value / abs(value))
        return value

    def limit(self, value: float | int, valueMax: float | int, valueMin: float | int) -> float | int:
        if(value > valueMax): value = valueMax
        if(value < valueMin): value = valueMin
        return value

    async def gyroForward(self, distance: float, speed: int, *, kp: float, ki: float, kd: float, dt: int, tresholdIntegral: int, tresholdStallDetection = 10, iteratorStallDetection: int = 5, stop: bool = True) -> None:
        # ---> Needed to know <------> Documentation <---

        '''
            ---> Parameters <---

            distance - the distance (in centimeters) that the robot will travel with a constant speed \n
            speed - the speed that is applied on the robots wheels (recommended to be in the range of [175, 925]) \n
            kp, ki, kd, dt - constants for the Proportional - Integral - Derivative Controller \n
            tresholdIntegral - constant that keeps the integral bound to this so that it avoids overshooting \n
            tresholdStallDetection - the value that is used to tell if the robot is stuck or not \n
            iteratorStallDetection - how frequent we check if the robot is stuck (affected by dt) \n
            stop - tells the robot if we want to stop it after it has travelled the given distasnce (slowing down / speeding up) \n

            ---> For more information about how to use these parameters and functions please take a look in our technical notebook <---
        '''

        # ---> Initialization <---
        self.iterator = 0; tresholdIntegral = abs(tresholdIntegral); iteratorStallDetection = abs(iteratorStallDetection); speed = abs(speed)
        if(iteratorStallDetection == 0): print ("Error - Iterator Stall Detection = 0"); return None
        if(speed > self.pairMaxSpeed or speed < self.pairMinSpeed): print("Error - Speed out of bounds"); return None

        # ---> Backend for automatization <---
        if(self.lastFunction == 1): # ---> Gyro Forwards <---
            self.reachDistance += int(distance * 2 * 20.45)
        elif(self.lastFunction == 2 or self.lastFunction == 3 or self.lastFunction == 4):
            # ---> Gyro Backwards / Turn Left / Turn Right <---
            self.reachDistance = int(distance * 2 * 20.45)
            self.position = 0; self.lastPosition = 0
            self.error = 0; self.lastError = 0
            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.correction = 0

        # ---> Calculate min & max correction so that the speed applied is always bounded <---
        self.maxCorrection: int = max(self.pairMaxSpeed - abs(speed), abs(speed) - self.pairMinSpeed)
        self.minCorrection: int = min(self.pairMaxSpeed - abs(speed), abs(speed) - self.pairMinSpeed)

        # ---> The PID Controller with Object Stall Detection <---
        while(self.position < self.reachDistance):

            # ---> The Proportional - Integral - Derivative Controller <---
            self.error = (self.reachTarget - gs.tilt_angles()[0])
            self.proportional = (self.error)
            self.integral += (self.error * dt)
            self.derivative = (self.error - self.lastError) / dt
            self.lastError = (self.error)

            # ---> Calculating the correction & limiting it & the integral if they're out of bounds <---
            self.integral = DriveBase.limit(self, self.integral, tresholdIntegral, -tresholdIntegral)
            self.correction = ((self.proportional * kp) + (self.integral * ki) + (self.derivative * kd))
            self.correction = DriveBase.limitAbs(self, self.correction, self.maxCorrection, self.minCorrection)

            # ---> Applying the speeds <---
            motor_pair.move_tank(self.pair, int(speed - self.correction), int(speed + self.correction))

            # ---> Object Stall Detection <---
            await runloop.sleep_ms(dt); self.iterator += 1
            if(self.iterator % iteratorStallDetection == 0):
                if(self.position - tresholdStallDetection < self.lastPosition):
                    break; # ---> The robot is stuck <---
                self.lastPosition = self.position
            self.position = abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor))

        # ---> Stopping (if necessary) & exiting <---
        if(stop): motor_pair.stop(self.pair, stop = motor.SMART_BRAKE)
        self.lastFunction = 1; return None

    async def gyroBackwards(self, distance: float, speed: int, *, kp: float, ki: float, kd: float, dt: int, tresholdIntegral: int, tresholdStallDetection = 10, iteratorStallDetection: int = 5, stop: bool = True) -> None:
        # ---> Needed to know <------> Documentation <---

        '''
            ---> Parameters <---

            distance - the distance (in centimeters) that the robot will travel with a constant speed \n
            speed - the speed that is applied on the robots wheels (recommended to be in the range of [175, 925]) \n
            kp, ki, kd, dt - constants for the Proportional - Integral - Derivative Controller \n
            tresholdIntegral - constant that keeps the integral bound to this so that it avoids overshooting \n
            tresholdStallDetection - the value that is used to tell if the robot is stuck or not \n
            iteratorStallDetection - how frequent we check if the robot is stuck (affected by dt) \n
            stop - tells the robot if we want to stop it after it has travelled the given distasnce (slowing down / speeding up) \n

            ---> For more information about how to use these parameters and functions please take a look in our technical notebook <---
        '''

        # ---> Initialization <---
        self.iterator = 0; tresholdIntegral = abs(tresholdIntegral); iteratorStallDetection = abs(iteratorStallDetection); speed = abs(speed)
        if(iteratorStallDetection == 0): print ("Error - Iterator Stall Detection = 0"); return None
        if(speed > self.pairMaxSpeed or speed < self.pairMinSpeed): print("Error - Speed out of bounds"); return None

        # ---> Backend for automatization <---
        if(self.lastFunction == 2): # ---> Gyro Backwards <---
            self.reachDistance += int(distance * 2 * 20.45)
        elif(self.lastFunction == 1 or self.lastFunction == 3 or self.lastFunction == 4):
            # ---> Gyro Forwards / Turn Left / Turn Right <---
            self.reachDistance = int(distance * 2 * 20.45)
            self.position = 0; self.lastPosition = 0
            self.error = 0; self.lastError = 0
            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.correction = 0

        # ---> Calculate min & max correction so that the speed applied is always bounded <---
        self.maxCorrection: int = max(self.pairMaxSpeed - abs(speed), abs(speed) - self.pairMinSpeed)
        self.minCorrection: int = min(self.pairMaxSpeed - abs(speed), abs(speed) - self.pairMinSpeed)

        # ---> The PID Controller with Object Stall Detection <---
        while(self.position < self.reachDistance):

            # ---> The Proportional - Integral - Derivative Controller <---
            self.error = (self.reachTarget - gs.tilt_angles()[0])
            self.proportional = (self.error)
            self.integral += (self.error * dt)
            self.derivative = (self.error - self.lastError) / dt
            self.lastError = (self.error)

            # ---> Calculating the correction & limiting it & the integral if they're out of bounds <---
            self.integral = DriveBase.limit(self, self.integral, tresholdIntegral, -tresholdIntegral)
            self.correction = -((self.proportional * kp) + (self.integral * ki) + (self.derivative * kd))
            self.correction = DriveBase.limitAbs(self, self.correction, self.maxCorrection, self.minCorrection)

            # ---> Applying the speeds <---
            motor_pair.move_tank(self.pair, int(-speed - self.correction), int(-speed + self.correction))

            # ---> Object Stall Detection <---
            await runloop.sleep_ms(dt); self.iterator += 1
            if(self.iterator % iteratorStallDetection == 0):
                if(self.position - tresholdStallDetection < self.lastPosition):
                    break; # ---> The robot is stuck <---
                self.lastPosition = self.position
            self.position = abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor))

        # ---> Stopping (if necessary) & exiting <---
        if(stop): motor_pair.stop(self.pair, stop = motor.SMART_BRAKE)
        self.lastFunction = 2; return None

    async def turnLeft(self, degrees: int, speedMax: int, speedMin: int, *, kp: float = 0.25, dt: int = 2, kLeft: float = -1, kRight: float = 1, tresholdError: int = 5, tresholdStallDetection = 10, iteratorStallDetection: int = 5, stop: bool = True):
        return None

# ---> The class for the upper part of the robot that makes the systems move <---
class Systems():
    def __init__(self, leftMotor: int, rightMotor: int) -> None: self.leftMotor = leftMotor; self.rightMotor = rightMotor; return None
    async def lowerLeftArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.leftMotor, -degrees, speed); return None
    async def raiseLeftArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.leftMotor, degrees, speed); return None
    async def lowerRightArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.rightMotor, -degrees, speed); return None
    async def raiseRightArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.rightMotor, degrees, speed); return None

async def main():
    driveBase = DriveBase(hub.port.B, hub.port.D); systems = Systems(hub.port.A, hub.port.C)

runloop.run(main())
