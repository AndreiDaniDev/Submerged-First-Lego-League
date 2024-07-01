import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

'''
    # ---> Defining the hub's motors & ports <---
    leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
    leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
    leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
'''

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase():
    def __init__(self, leftMotor, rightMotor) -> None:

        # ---> Defining the pair that is made out of [leftMotor, rightMotor] <---
        motor_pair.pair(motor_pair.PAIR_1, leftMotor, rightMotor)
        self.leftMotor = leftMotor; self.rightMotor = rightMotor
        self.pair: int = motor_pair.PAIR_1; gs.reset_yaw(0)

        # ---> Parameters for bounding specific values & fixing errors <---
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 125
        self.reachDistance: int = 0; self.reachTarget: int = 0
        self.position: int = 0; self.lastPosition: int = 0
        self.plusCorrection: int = 0; self.minusCorrection: int = 0
        self.thresholdCorrection: int = 0

        # ---> Parameters for the Proportional - Integral - Derivative Controller <---
        self.error: float = 0; self.lastError: float = 0
        self.proportional: float = 0; self.integral: float = 0
        self.derivative: float = 0; self.correction: float = 0
        self.speedLeft: int = 0; self.speedRight: int = 0

        # ---> Parameters & Commands for Object Stall Detection <---
        self.iterator: int = 0; self.lastFunction: int = 0

        motor.reset_relative_position(self.leftMotor, 0)
        motor.reset_relative_position(self.rightMotor, 0)

        return None

    def initRun(self) -> None:

        # ---> Parameters for bounding & error handling <---
        self.reachDistance = 0; self.reachTarget = 0
        self.position = 0; self.lastPosition = 0
        self.lastFunction = 0; self.iterator = 0

        # ---> Parameters for the PID Controller <---
        self.error = 0; self.lastError = 0
        self.proportional = 0; self.integral = 0
        self.derivative = 0; self.correction = 0
        self.plusCorrection = 0; self.minusCorrection = 0
        self.thresholdCorrection = 0
        self.speedLeft = 0; self.speedRight = 0

        # ---> Commands for error handling <---
        # ---> and object stall detection <---
        motor.reset_relative_position(self.leftMotor, 0)
        motor.reset_relative_position(self.rightMotor, 0)
        gs.reset_yaw(self.reachTarget)

        return None

    def limitAbs(self, value: float | int, valueMax: float | int, valueMin: float | int) -> float | int:
        # ---> Limit a value so that it is in the range of [valueMin, valueMax] without changing its sign <---
        if(abs(value) > valueMax): value = valueMax * (value / abs(value))
        if(abs(value) < valueMin): value = valueMin * (value / abs(value))
        return value

    def limit(self, value: float | int, valueMax: float | int, valueMin: float | int) -> float | int:
        # ---> Limit a value so that it is in the range of [valueMin, valueMax] (valueMin can be negative) <---
        if(value > valueMax): value = valueMax
        if(value < valueMin): value = valueMin
        return value

    async def gyroForward(self, distance: float, speed: int, *, kp: float, ki: float, kd: float, dt: int, thresholdIntegral: float, thresholdStallDetection = 10, iteratorStallDetection: int = 5, stop: bool = True) -> None:
        # ---> Needed to know <------> Documentation <---

        '''
            ---> Parameters <---

            distance - the distance (in centimeters) that the robot will travel with a constant speed \n
            speed - the speed that is applied on the robots wheels (recommended to be in the range of [175, 1000]) \n
            kp, ki, kd, dt - constants for the Proportional - Integral - Derivative Controller \n
            thresholdIntegral - constant that keeps the integral bound to this so that it avoids overshooting \n
            thresholdStallDetection - the value that is used to tell if the robot is stuck or not \n
            iteratorStallDetection - how frequent we check if the robot is stuck (affected by dt) \n
            stop - tells the robot if we want to stop it after it has travelled the given distance \n

            ---> For more information about how to use these parameters and functions please take a look in our technical notebook <---
        '''

        # ---> Initialization <---
        self.iterator = 0; thresholdIntegral = abs(thresholdIntegral); iteratorStallDetection = abs(iteratorStallDetection)
        kp = abs(kp); ki = abs(ki); kd = abs(kd); dt = abs(dt); distance = abs(distance); speed = abs(speed)
        
        # ---> Error Checking <---
        if(iteratorStallDetection == 0): print ("Error - Iterator Stall Detection = 0"); return None
        if(speed > self.pairMaxSpeed): print("Error - Speed out of bounds"); return None
        if(speed < self.pairMinSpeed): print("Error - Speed out of bounds"); return None
        if(dt == 0): print("Error - dt = 0"); return None

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
            motor.reset_relative_position(self.leftMotor, 0)
            motor.reset_relative_position(self.rightMotor, 0)

        # ---> Calculate min & max correction so that the speed applied is always bounded <---
        self.plusCorrection = (self.pairMaxSpeed - abs(speed)); self.minusCorrection = (abs(speed) - self.pairMinSpeed)
        self.thresholdCorrection = min(self.plusCorrection, self.minusCorrection)

        # ---> The PID Controller with Object Stall Detection <---
        while(self.position < self.reachDistance):

            # ---> The Proportional - Integral - Derivative Controller <---
            self.error = (self.reachTarget - gs.tilt_angles()[0])
            self.proportional = (self.error)
            self.integral += (self.error * dt)
            self.derivative = (self.error - self.lastError) / dt
            self.lastError = (self.error)

            # ---> Calculating the correction & limiting it + the integral if they're out of bounds <---
            self.integral = DriveBase.limit(self, self.integral, thresholdIntegral, -thresholdIntegral)
            self.correction = ((self.proportional * kp) + (self.integral * ki) + (self.derivative * kd))
            self.correction = DriveBase.limit(self, self.correction, self.thresholdCorrection, -self.thresholdCorrection)

            # ---> Calculating the speeds and applying them <---
            self.speedLeft = int(speed - self.correction); self.speedRight = int(speed + self.correction)
            motor_pair.move_tank(self.pair, self.speedLeft, self.speedRight)

            # ---> Object Stall Detection every iteratorStallDetection updates <---
            await runloop.sleep_ms(dt); self.iterator += 1
            if(self.iterator % iteratorStallDetection == 0):
                if(self.position - thresholdStallDetection < self.lastPosition):
                    break; # ---> The robot is stuck and so the function finishes <---
                self.lastPosition = self.position; self.iterator = 0
            self.position = abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor))

        # ---> Stopping (if said so) & exiting <---
        if(stop): motor_pair.stop(self.pair, stop = motor.SMART_BRAKE)
        self.lastFunction = 1; return None

    async def turnLeft(self, degrees: float, speedMax: int, speedMin: int, *, kp: float, ki: float, kd: float, dt: int, thresholdIntegral: float, kLeft: float = -1, kRight: float = 1, thresholdError: int = 5, thresholdStallDetection = 10, iteratorStallDetection: int = 5, stop: bool = True) -> None:
        # ---> Needed to know <------> Documentation <---

        '''
            ---> Parameters <---

            degrees - the target that the robot needs to reach (in degrees with one decimal point precision) \n
            speedMax, speedMin - two pointers that make the speed applied always be bounded between these two \n
            kp, ki, kd, dt - constants used for the Proportional - Integral - Derivative Controller \n
            kLeft, kRight - constants that are applied to the correction before applying the speeds \n
            kLeft, kRight - should be set to one of these combinations: [[-1, 1], [-1, 0], [0, 1]] \n
            thresholdError - the max error that is accepted when turning (must be >= 5) \n
            thresholdStallDetection - the value that is used to tell if the robot is stuck or not \n
            iteratorStallDetection - how frequent we check if the robot is stuck (affected by dt) \n
            stop - tells the robot if we want to stop it after it has reached the given target \n

            ---> For more information about how to use these parameters and functions please take a look in our technical notebook <---
        '''

        # ---> Initialization <---
        self.iterator = 0; gs.reset_yaw(gs.tilt_angles()[0] - self.reachTarget); self.reachTarget = -int(10 * abs(degrees))
        kp = abs(kp); ki = abs(ki); kd = abs(kd); dt = abs(dt); speedMax = abs(speedMax); speedMin = abs(speedMin)
        motor.reset_relative_position(self.leftMotor, 0); motor.reset_relative_position(self.rightMotor, 0)
        thresholdError = abs(thresholdError); iteratorStallDetection = abs(iteratorStallDetection)
        
        self.error = self.reachTarget - gs.tilt_angles()[0]; self.lastError = 0
        self.proportional = 0; self.integral = 0; self.derivative = 0; 
        self.correction = 0; self.position = 0; self.lastPosition = 0

        # ---> Error Checking <---
        if(iteratorStallDetection == 0): print("Error - Iterator Stall Detection = 0"); return None
        if(speedMax > self.pairMaxSpeed): print("Error - speedMax out of bounds"); return None
        if(speedMax < self.pairMinSpeed): print("Error - speedMax out of bounds"); return None
        if(speedMin < self.pairMinSpeed): print("Error - speedMin out of bounds"); return None
        if(speedMin > self.pairMaxSpeed): print("Error - speedMin out of bounds"); return None
        if(speedMax < speedMin): print("Error - speedMax | speedMin"); return None
        if(thresholdError < 5): print("Error - thresholdError < 5"); return None
        if(kLeft - 0.25 > kRight): print("Error - kRight | kLeft"); return None
        if(degrees > 170): print("Error - Degrees out of bounds"); return None
        if(dt == 0): print("Error - dt = 0"); return None

        # ---> The Proportional - Integral - Derivative Controller with object stall detection <---
        while(self.error < -thresholdError or self.error > thresholdError):

            # ---> the Proportional - Integral - Derivative Controller <---
            self.error = self.reachTarget - gs.tilt_angles()[0];  
            self.proportional = self.error
            self.integral += (self.error * dt)
            self.derivative = (self.error - self.lastError) / dt
            self.lastError = (self.error)

            # ---> Limiting the integral, correction & speeds <---
            self.integral = DriveBase.limit(self, self.integral, thresholdIntegral, -thresholdIntegral)
            self.correction = ((self.proportional * kp) + (self.integral * ki) + (self.derivative * kd))

            self.speedLeft = int(DriveBase.limitAbs(self, self.correction * kLeft, speedMax, speedMin)) * (kLeft != 0)
            self.speedRight = int(DriveBase.limitAbs(self, self.correction * kRight, speedMax, speedMin)) * (kRight != 0)

            # ---> Applying the speeds <---
            motor_pair.move_tank(self.pair, self.speedLeft, self.speedRight)

            # ---> Object Stall Detection every iteratorStallDetection updates <---
            await runloop.sleep_ms(dt); self.iterator += 1
            if(self.iterator % iteratorStallDetection == 0):
                if(self.position - thresholdStallDetection < self.lastPosition):
                    break; # ---> The robot is stuck and so the function finishes <---
                self.lastPosition = self.position; self.iterator = 0
            self.position = abs(motor.relative_position(self.leftMotor)) + abs(motor.relative_position(self.rightMotor))

        # ---> Stopping (if said so) & exiting <---
        if(stop): motor_pair.stop(self.pair, stop = motor.SMART_BRAKE)
        self.lastFunction = 3; return None

# ---> The class for the upper part of the robot that is responsible for making the systems move <---
class Systems():
    def __init__(self, leftMotor: int, rightMotor: int) -> None: 
        self.leftMotor = leftMotor; self.rightMotor = rightMotor; return None
    async def rotateLeftMotor(self, degrees: int, speed: int) -> None: 
        await motor.run_for_degrees(self.leftMotor, degrees, speed); return None
    async def rotateRightMotor(self, degrees: int, speed: int) -> None: 
        await motor.run_for_degrees(self.rightMotor, degrees, speed); return None

# ---> The main program <---
async def main():
    driveBase = DriveBase(hub.port.B, hub.port.D); systems = Systems(hub.port.A, hub.port.C)

runloop.run(main())
