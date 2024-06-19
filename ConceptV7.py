#https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#
#15_06_2024 in spike 3.4.3 & Visual Studio Code
import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

# ---> Robot Ports & Global Values <---
leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
previousIntegral = 0; previousError = 0

async def gyroForwards(speed: int, distance: float, target: int = 0, *, kp: float = 0.4, ki: float = 0.2, kd: float = 0.001, dt : int = 5, tresholdIntegral: float = 125, lastError: float = 0, lastIntegral: float = 0, trajectory: int = 0, stop: bool = True, reset: bool = True, tresholdStallDetection: int = 10):
    # ---> Needed to know <------> Documentation <---
    
    ''' 
        ---> Parameters <---
        speed - the speed that is applied on the wheel's of the robot
        distance (cm) - the distance that the robot will travel
        target - a value that makes the robot move in a circle (affected by the PID Controller)
        kp, ki, kd, dt - constants that are used in the Proportional - Integral - Derivative controller
        tresholdIntegral - used for bounding the integral so that it doesn't generate an even bigger error
        lastError - used when speeding up / slowing down, usually after a func like this
        lastIntegral - just like lastError, but for the integral
        trajectory - usually used after a turn, so that the robot loses any error from the turn
        stop - used when speeding up / slowing down
        reset - used when there is only a gyro / we don't need that error
        tresholdStallDetection - used to see if the robot is stuck or still moving
        if we dont want to use objectStallDetection this needs to be set to a negative value 
    '''

    # ---> Initialization <---

    distance = int(distance * 20.45 * 2); lastErrorLocal = lastError; integralLocal = lastIntegral; lastRelativePosition = 0
    motor.reset_relative_position(leftDriveBaseMotor, 0); motor.reset_relative_position(rightDriveBaseMotor, 0)
    summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
    if(reset): gs.reset_yaw(0); trajectory = 0 # ---> If there is a trajectory, but the yaw was reseted, we set it to 0 <---
    else: trajectory *= (abs(gs.tilt_angles()[0]) / gs.tilt_angles()[0]) # ---> So that the robot goes straight after a turn <---
    # ---> It still remains a mistery of how target is implemented corectly without using only a P Controller<---
    # ---> If we want to use target we set 0 to kd, 0 to ki and dt to 1 <---

    # ---> The PID Controller with object-stall detection <---

    while(summyPositions < distance):
        # ---> The PID Controller itself <---
        errorLocal = trajectory - gs.tilt_angles()[0]
        proportionalLocal = (errorLocal - target)
        differentailLocal = (errorLocal - lastErrorLocal) / dt
        integralLocal += errorLocal * dt; lastErrorLocal = errorLocal

        # ---> Bounding the integral to tresholdIntegral so that the robot doesn't generate a bigger error <---
        if(integralLocal > tresholdIntegral): integralLocal = tresholdIntegral
        if(integralLocal < -tresholdIntegral): integralLocal = -tresholdIntegral

        correction = (proportionalLocal * kp) + (integralLocal * ki) + (differentailLocal * kd)
        
        # ---> Moving the robot and waiting a few miliseconds <---
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed - correction), int(speed + correction))
        await runloop.sleep_ms(dt) # ---> This is done so that the robot actually has time to move <---

        # ---> Object Stall Detection <---
        summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
        if(summyPositions - tresholdStallDetection < lastRelativePosition):
            break; # --> It means that the robot got stuck and we need to finish the function <---
        lastRelativePosition = summyPositions

    # ---> Stopping the robot if necessary and leaving the integral & error for the next functions <--- 
    previousError = lastErrorLocal; previousIntegral = integralLocal
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def gyroBackwards(speed: int, distance: float, target: int = 0, *, kp: float = 0.4, ki: float = 0.2, kd: float = 0.001, dt : int = 5, tresholdIntegral: float = 125, lastError: float = 0, lastIntegral: float = 0, trajectory: int = 0, stop: bool = True, reset: bool = True, tresholdStallDetection: int = 10):
    # ---> Needed to know <------> Documentation <---

    '''
        ---> Parameters <---
        speed - the speed that is applied on the wheel's of the robot
        distance (cm) - the distance that the robot will travel
        target - a value that makes the robot move in a circle (affected by the PID Controller)
        kp, ki, kd, dt - constants that are used in the Proportional - Integral - Derivative controller
        tresholdIntegral - used for bounding the integral so that it doesn't generate an even bigger error
        lastError - used when speeding up / slowing down, usually after a func like this
        lastIntegral - just like lastError, but for the integral
        trajectory - usually used after a turn, so that the robot loses any error from the turn
        stop - used when speeding up / slowing down
        reset - used when there is only a gyro / we don't need that error
        tresholdStallDetection - used to see if the robot is stuck or still moving
            if we dont want to use objectStallDetection this needs to be set to a negative value
    '''

    # ---> Initialization <---

    distance = int(distance * 20.45 * 2); lastErrorLocal = lastError; integralLocal = lastIntegral; lastRelativePosition = 0
    motor.reset_relative_position(leftDriveBaseMotor, 0); motor.reset_relative_position(rightDriveBaseMotor, 0); speed *= -1
    summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
    if(reset): gs.reset_yaw(0); trajectory = 0 # ---> If there is a trajectory, but the yaw was reseted, we set it to 0 <---
    else: trajectory *= (abs(gs.tilt_angles()[0]) / gs.tilt_angles()[0]) # ---> So that the robot goes straight after a turn <---
    # ---> It still remains a mistery of how target is implemented corectly without using only a P Controller<---
    # ---> If we want to use target we set 0 to kd, 0 to ki and dt to 1 <---

    # ---> The PID Controller with object-stall detection <---

    while(summyPositions < distance):
        # ---> The PID Controller itself <---
        errorLocal = trajectory - gs.tilt_angles()[0]
        proportionalLocal = (errorLocal - target)
        differentailLocal = (errorLocal - lastErrorLocal) / dt
        integralLocal += errorLocal * dt; lastErrorLocal = errorLocal

        # ---> Bounding the integral to tresholdIntegral so that the robot doesn't generate a bigger error <---
        if(integralLocal > tresholdIntegral): integralLocal = tresholdIntegral
        if(integralLocal < -tresholdIntegral): integralLocal = -tresholdIntegral

        correction = -((proportionalLocal * kp) + (integralLocal * ki) + (differentailLocal * kd))
        
        # ---> Moving the robot and waiting a few miliseconds <---
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed - correction), int(speed = correction))
        await runloop.sleep_ms(dt) # ---> This is done so that the robot actually has time to move <---

        # ---> Object Stall Detection <---
        summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
        if(summyPositions - tresholdStallDetection < lastRelativePosition):
            break; # --> It means that the robot got stuck and we need to finish the function <---
        lastRelativePosition = summyPositions

    # ---> Stopping the robot if necessary and leaving the integral & error for the next functions <--- 
    previousError = lastErrorLocal; previousIntegral = integralLocal
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def turnLeft(degrees: float, speedMax: int, speedMin: int, *, kp: float = 0.5, dt: int = 2, targetOffset: int = 0, maxError: int = 5):
    # ---> Needed to know <------> Documentation <----
    
    ''' 
        ---> Parameters <--- 
        degrees - how much we want the robot to move
        speedMax, speedMin - how fast / slow we want the robot to move
        kp, dt - constants for the Proportional Controller
        targetOffset - the starter value for the gyro sensor (used for 180* turns or more)
    '''

    # ---> Initialization <---

    gs.reset_yaw(targetOffset); target = int(degrees * 10); speedMax = min(speedMax, 1000); speedMin = max(speedMin, 100)
    error = target - gs.tilt_angles()[0]; # ------>>> This will need a lot of debugging in the future <<<------

    # ---> The Proportional Controller that slows down overtime <---

    while(error < -maxError or error > maxError):
        error = target - gs.tilt_angles()[0]
        correction = (error) * kp
        
        # ---> Bounding the correction so that the robot doesn't move too fast / slow <---
        if(correction > 0):
            if(correction > speedMax): correction = speedMax
            if(correction < speedMin): correction = speedMin
        else:
            if(correction < -speedMax): correction = -speedMax
            if(correction > -speedMin): correction = -speedMin

        # ---> Applying the correction and giving it some time to move <---
        motor_pair.move_tank(motor_pair.PAIR_1, int(correction), -int(correction))
        await runloop.sleep_ms(dt)

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def turnRight(degrees: float, speedMax: int, speedMin: int, *, kp: float = 0.5, dt: int = 2, targetOffset: int = 0, maxError: int = 5):
    # ---> Needed to know <------> Documentation <----
    
    ''' 
        ---> Parameters <--- 
        degrees - how much we want the robot to move
        speedMax, speedMin - how fast / slow we want the robot to move
        kp, dt - constants for the Proportional Controller
        targetOffset - the starter value for the gyro sensor (used for 180* turns or more)
    '''

    # ---> Initialization <---

    gs.reset_yaw(targetOffset); target = int(degrees * 10); speedMax = min(speedMax, 1000); speedMin = max(speedMin, 100)
    error = target - gs.tilt_angles()[0]; degrees *= -1; # ------>>> This will need a lot of debugging in the future <<<------

    # ---> The Proportional Controller that slows down overtime <---

    while(error < -maxError or error > maxError):
        error = target - gs.tilt_angles()[0]
        correction = (error) * kp
        
        # ---> Bounding the correction so that the robot doesn't move too fast / slow <---
        if(correction > 0):
            if(correction > speedMax): correction = speedMax
            if(correction < speedMin): correction = speedMin
        else:
            if(correction < -speedMax): correction = -speedMax
            if(correction > -speedMin): correction = -speedMin

        # ---> Applying the correction and giving it some time to move <---
        motor_pair.move_tank(motor_pair.PAIR_1, -int(correction), int(correction))
        await runloop.sleep_ms(dt)

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def turnLeftOneWheel(degrees: float, speedMax: int, speedMin: int, *, kp: float = 0.5, dt: int = 2, targetOffset: int = 0, maxError: int = 5):
    # ---> Needed to know <------> Documentation <----
    
    ''' 
        ---> Parameters <--- 
        degrees - how much we want the robot to move
        speedMax, speedMin - how fast / slow we want the robot to move
        kp, dt - constants for the Proportional Controller
        targetOffset - the starter value for the gyro sensor (used for 180* turns or more)
    '''

    # ---> Initialization <---

    gs.reset_yaw(targetOffset); target = int(degrees * 10); speedMax = min(speedMax, 1000); speedMin = max(speedMin, 100)
    error = target - gs.tilt_angles()[0]; # ------>>> This will need a lot of debugging in the future <<<------

    # ---> The Proportional Controller that slows down overtime <---

    while(error < -maxError or error > maxError):
        error = target - gs.tilt_angles()[0]
        correction = (error) * kp
        
        # ---> Bounding the correction so that the robot doesn't move too fast / slow <---
        if(correction > 0):
            if(correction > speedMax): correction = speedMax
            if(correction < speedMin): correction = speedMin
        else:
            if(correction < -speedMax): correction = -speedMax
            if(correction > -speedMin): correction = -speedMin

        # ---> Applying the correction and giving it some time to move <---
        motor_pair.move_tank(motor_pair.PAIR_1, 0, -int(correction))
        await runloop.sleep_ms(dt)

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def turnLeftOtherWheel(degrees: float, speedMax: int, speedMin: int, *, kp: float = 0.5, dt: int = 2, targetOffset: int = 0, maxError: int = 5):
    # ---> Needed to know <------> Documentation <----
    
    ''' 
        ---> Parameters <--- 
        degrees - how much we want the robot to move
        speedMax, speedMin - how fast / slow we want the robot to move
        kp, dt - constants for the Proportional Controller
        targetOffset - the starter value for the gyro sensor (used for 180* turns or more)
    '''

    # ---> Initialization <---

    gs.reset_yaw(targetOffset); target = int(degrees * 10); speedMax = min(speedMax, 1000); speedMin = max(speedMin, 100)
    error = target - gs.tilt_angles()[0]; # ------>>> This will need a lot of debugging in the future <<<------

    # ---> The Proportional Controller that slows down overtime <---

    while(error < -maxError or error > maxError):
        error = target - gs.tilt_angles()[0]
        correction = (error) * kp
        
        # ---> Bounding the correction so that the robot doesn't move too fast / slow <---
        if(correction > 0):
            if(correction > speedMax): correction = speedMax
            if(correction < speedMin): correction = speedMin
        else:
            if(correction < -speedMax): correction = -speedMax
            if(correction > -speedMin): correction = -speedMin

        # ---> Applying the correction and giving it some time to move <---
        motor_pair.move_tank(motor_pair.PAIR_1, int(correction), 0)
        await runloop.sleep_ms(dt)

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def turnRightOneWheel(degrees: float, speedMax: int, speedMin: int, *, kp: float = 0.5, dt: int = 2, targetOffset: int = 0, maxError: int = 5):
    # ---> Needed to know <------> Documentation <----
    
    ''' 
        ---> Parameters <--- 
        degrees - how much we want the robot to move
        speedMax, speedMin - how fast / slow we want the robot to move
        kp, dt - constants for the Proportional Controller
        targetOffset - the starter value for the gyro sensor (used for 180* turns or more)
    '''

    # ---> Initialization <---

    gs.reset_yaw(targetOffset); target = int(degrees * 10); speedMax = min(speedMax, 1000); speedMin = max(speedMin, 100)
    error = target - gs.tilt_angles()[0]; degrees *= -1; # ------>>> This will need a lot of debugging in the future <<<------

    # ---> The Proportional Controller that slows down overtime <---

    while(error < -maxError or error > maxError):
        error = target - gs.tilt_angles()[0]
        correction = (error) * kp
        
        # ---> Bounding the correction so that the robot doesn't move too fast / slow <---
        if(correction > 0):
            if(correction > speedMax): correction = speedMax
            if(correction < speedMin): correction = speedMin
        else:
            if(correction < -speedMax): correction = -speedMax
            if(correction > -speedMin): correction = -speedMin

        # ---> Applying the correction and giving it some time to move <---
        motor_pair.move_tank(motor_pair.PAIR_1, -int(correction), 0)
        await runloop.sleep_ms(dt)

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def turnRightOtherWheel(degrees: float, speedMax: int, speedMin: int, *, kp: float = 0.5, dt: int = 2, targetOffset: int = 0, maxError: int = 5):
    # ---> Needed to know <------> Documentation <----
    
    ''' 
        ---> Parameters <--- 
        degrees - how much we want the robot to move
        speedMax, speedMin - how fast / slow we want the robot to move
        kp, dt - constants for the Proportional Controller
        targetOffset - the starter value for the gyro sensor (used for 180* turns or more)
    '''

    # ---> Initialization <---

    gs.reset_yaw(targetOffset); target = int(degrees * 10); speedMax = min(speedMax, 1000); speedMin = max(speedMin, 100)
    error = target - gs.tilt_angles()[0]; degrees *= -1; # ------>>> This will need a lot of debugging in the future <<<------

    # ---> The Proportional Controller that slows down overtime <---

    while(error < -maxError or error > maxError):
        error = target - gs.tilt_angles()[0]
        correction = (error) * kp
        
        # ---> Bounding the correction so that the robot doesn't move too fast / slow <---
        if(correction > 0):
            if(correction > speedMax): correction = speedMax
            if(correction < speedMin): correction = speedMin
        else:
            if(correction < -speedMax): correction = -speedMax
            if(correction > -speedMin): correction = -speedMin

        # ---> Applying the correction and giving it some time to move <---
        motor_pair.move_tank(motor_pair.PAIR_1, 0, int(correction))
        await runloop.sleep_ms(dt)

    motor_pair.stop(motor_pair.PAIR_1, stop = motor.SMART_BRAKE)

    return

async def main():
    motor_pair.pair(motor_pair.PAIR_1, leftDriveBaseMotor, rightDriveBaseMotor)

    # ---> Only Proportional - Integral Controller (The differential is tricky because of the dt value) <---
    await gyroForwards(750, 20, kp = 0.5, ki = 0.75, kd = 0, dt = 2)

    # ---> Proportional Controller with Steering (Target != 0) <---
    await gyroForwards(750, 25, 100, kp = 0.75, ki = 0, kd = 0, dt = 1)


runloop.run(main())
