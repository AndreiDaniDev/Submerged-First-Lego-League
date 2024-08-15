import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

# ---> Robot Ports & Global Values <---
leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
previousIntegral = 0; previousError = 0; lastFunction = 0

# ---> Map table for lastFunction <---
'''
    lastFunction = 0 - Null
    lastFunction = 1 - Gyro Forward
    lastFunction = 2 - Gyro Backwards
    lastFunction = 3 - Turn Left
    lastFunction = 4 - Turn Right
    lastFunction = 5 - Turn Left Forwards
    lastFunction = 6 - Turn Right Forwards
    lastFunction = 7 - Turn Left Backwards
    lastFunction = 8 - Turn Right Backwards
'''

# ---> Some universal functions that we use for our functions to be easier to understand <---

# ---> Function to limit the speed / correction, regardless of its sign <---
def limitSpeed(speed: float, speedMax: int, speedMin: int):
    if(abs(speed) > speedMax): speed = speedMax * (speed / (abs(speed)))
    if(abs(speed) < speedMin): speed = speedMin * (speed / (abs(speed)))
    return speed

# ---> Function to limit a value (like the Integral) between its highest & lowest <---
def limitValue(value: float, valueMax: float, valueMin: float):
    if(value > valueMax): value = valueMax
    if(value < valueMin): value = valueMin
    return value

# ---> Function to swap two values <---
def swap(value1, value2): value3 = value1; value1 = value2; value2 = value3; return [value1, value2]

async def gyroForwards(speed: int, distance: float, target: int = 0, *, kp: float = 0.4, ki: float = 0.2, kd: float = 0.001, dt : int = 5, tresholdIntegral: float = 125, lastError: float = 0, lastIntegral: float = 0, trajectory: int = 0, stop: bool = True, setYaw: int = 0, tresholdStallDetection: int = 10, stallDetectionIterator: int = 5):
    # ---> Needed to know <------> Documentation <---

    '''
        ---> Parameters <---

        speed - the speed that is applied on the robots wheels (recommended to be in the range of [175, 925]) \n
        distance - the distance (in centimeters) that the robot will travel forwards / backwards \n
        target - the value that is used when we want to steere the robot (look in lines 63 / 64 / Technical Notebook) \n
        kp, ki, kd - constants that are used in the Proportional - Integral - Derivate Controller (more info in the Technical Notebook) \n
        dt - the waitTime between the updates, also used in the PID Controller (the shorter term) \n
        tresholdIntegral - a constant that makes the integral always be in the range of [-tresholdIntegral, tresholdIntegral] \n
        lastError, lastIntegral - used when the last function was gyroForwards to correct the error faster \n
        trajectory - used when the last function was a turn to correct the error from the turn \n
        setYaw - used when we don't want to lose the current reading of the gyroscopic sensor \n
        tresholdStallDetection - used to see if the robot is stuck or not\n
        stallDetectionIterator - how frequent we want to check if the robot is stuck or not ( can't be 0 !!! ) \n

        ---> For more information about these parameters and how to use them please take a look in the technical notebook <---
    '''

    # ---> Initialization <---
    distance = int(distance * 20.45 * 2); lastErrorLocal = lastError; integralLocal = lastIntegral; lastRelativePosition = 0
    motor.reset_relative_position(leftDriveBaseMotor, 0); motor.reset_relative_position(rightDriveBaseMotor, 0)
    summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
    gs.reset_yaw(setYaw); iterator: int = 0; global lastFunction, previousError, previousIntegral

    # ---> Choosing the right sign for the trajectory (so that it will be easier to code) <---
    if(lastFunction == 3 or lastFunction == 5 or lastFunction == 7): trajectory = -abs(trajectory)
    if(lastFunction == 4 or lastFunction == 6 or lastFunction == 8): trajectory = abs(trajectory)
    

    # ---> Solving for maxCorrection & minCorrection <---
    maxCorrection = max(1000 - abs(speed), abs(speed) - 100)
    minCorrection = min(1000 - abs(speed), abs(speed) - 100)

    # ---> The PID Controller with object-stall detection <---
    while(summyPositions < distance):

        # ---> The PID Controller itself <---
        errorLocal = trajectory - gs.tilt_angles()[0]
        proportionalLocal = (errorLocal - target)
        differentailLocal = (errorLocal - lastErrorLocal) / dt
        integralLocal += errorLocal * dt; lastErrorLocal = errorLocal
        iterator += 1 # ---> Iterator for object-stall-detection <---

        # ---> Bounding the integral to tresholdIntegral so that the robot doesn't generate a bigger error <---
        # ---> Integral windup (PID Controller Wiki) <---
        integralLocal = limitValue(integralLocal, tresholdIntegral, -tresholdIntegral)

        # ---> Limit correction so that the speed applied is in range of [100, 1000] <---
        # ---> Overshooting from known disturbances (PID Controller Wiki)
        correction = ((proportionalLocal * kp) + (integralLocal * ki) + (differentailLocal * kd))
        correction = limitSpeed(correction, maxCorrection, minCorrection)

        # ---> Moving the robot and waiting a few miliseconds <---
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed - correction), int(speed + correction))
        await runloop.sleep_ms(dt) # ---> This is done so that the robot actually has time to move <---

        # ---> Object Stall Detection <---
        if(iterator % stallDetectionIterator == 0):
            if(summyPositions - tresholdStallDetection < lastRelativePosition):
                break; # --> It means that the robot got stuck and we need to finish the function <---
            lastRelativePosition = summyPositions
        summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))

    # ---> Stopping the robot if necessary and leaving the integral & error for the next functions <---
    previousError = lastErrorLocal; previousIntegral = integralLocal; lastFunction = 1
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def TurnLeft(degrees: float, speedMax: int, speedMin: int, *, setYaw: int = 0, tresholdMinError: float = -5, tresholdMaxError: float = 5, kp: float = 0.575, dt: int = 5, stop: bool = True, tresholdStallDetection: int = 10, stallDetectionIterator: int = 5):
    # ---> Needed to know <------> Documentation <---

    '''
        ---> Parameters <---

        degrees - the target that the robot needs to reach (in degrees with one decimal) \n
        speedMax, speedMin - two pointers that make the speed always be in [speedMin, speedMax], regarding its sign \n
        setYaw - used when we don't want to lose the current reading of the gyroscopic sensor \n
        tresholdMinError, tresholdMaxError - used to see if the current error is in between the smallest / biggest marge of error \n
        kp, dt - constants for the Proportional Controller / dt - waitTime between updates \n
        stop - used for turns that are bigger than 180 degrees, but broken down into smaller ones \n
        tresholdStallDetection - used to see if the robot is stuck or not\n
        stallDetectionIterator - how frequent we want to check if the robot is stuck or not ( can't be 0 !!! ) \n

        ---> For more information about these parameters and how to use them please take a look in the technical notebook <---
    '''

    # ---> Initialization <---
    gs.reset_yaw(setYaw); target: int = -int(10 * degrees); error: int = target - gs.tilt_angles()[0]
    iterator: int = 0; summyPositions: int = 0; lastRelativePosition: int = 0
    # ---> The Proportional Controller with object stall detection <---
    while(error < tresholdMinError or error > tresholdMaxError):

        # ---> The Proportional Controller <---
        error = target - gs.tilt_angles()[0]
        speed = limitSpeed(error * kp, speedMax, speedMin)
        iterator += 1

        # ---> Applying the speed / correction and waiting <---
        motor_pair.move_tank(motor_pair.PAIR_1, -int(speed), int(speed))
        await runloop.sleep_ms(dt)

        # ---> Checking if the robot has stalled (Object Stall Detection) <---
        if(iterator % stallDetectionIterator == 0):
            if(summyPositions - tresholdStallDetection < lastRelativePosition):
                break; # --> It means that the robot got stuck and we need to finish the function <---
            lastRelativePosition = summyPositions
        summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))

    # ---> Stopping the robot (if necesarry) and exiting <---
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)
    lastFunction = 3; return

async def main():
    motor_pair.pair(motor_pair.PAIR_1, leftDriveBaseMotor, rightDriveBaseMotor)

runloop.run(main())
