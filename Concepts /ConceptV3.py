#https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#
#15_06_2024 in spike 3.4.3 & Visual Studio Code
import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

# ---> Robot Ports & Global Values <---
leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
previousIntegral = 0; previousError = 0

async def gyroForwards(speed: int, distance: float, target: int = 0, *, kp: float = 0.4, ki: float = 0.2, kd: float = 0.001, dt : int = 5, lastError: float = 0, lastIntegral: float = 0, trajectory: int = 0, stop: bool = True, reset: bool = True, tresholdStallDetection: int = 10):
    # ---> Needed to know <------> Documentation <---
    '''
        speed - the speed that is applied on the wheel's of the robot
        distance (cm) - the distance that the robot will travel
        target - a value that makes the robot move in a circle (affected by the PID Controller)
        kp, ki, kd, dt - constants that are used in the PID controller
        lastError - used when speeding up / slowing down, usually after a func like this
        lastIntegral - just like lastError, but for the integral
        trajectory - usually used after a turn, so that the robot loses any error from the turn
        stop - used when speeding up / slowing down
        reset - used when there is only a gyro / we don't need that error
        tresholdStallDetection - used to see if the robot is stuck or still moving
            if we dont want to use objectStallDetection this needs to be set to a negative value
    '''

    distance = int(distance * 20.45 * 2); lastErrorLocal = lastError; integralLocal = lastIntegral; lastRelativePosition = 0
    motor.reset_relative_position(leftDriveBaseMotor, 0); motor.reset_relative_position(rightDriveBaseMotor, 0)
    summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
    if(reset): gs.reset_yaw(0) 
    # ---> It still remains a mistery of how target is implemented corectly without using only a P Controller<---
    # ---> If we want to use target we set 0 to kd, 0 to ki and dt to 1 <---

    while(summyPositions < distance):
        # ---> The PID Controller itself <---
        errorLocal = trajectory - gs.tilt_angles()[0]
        proportionalLocal = (errorLocal - target)
        differentailLocal = (errorLocal - lastErrorLocal) / dt
        integralLocal += errorLocal * dt; lastErrorLocal = errorLocal
        correction = (proportionalLocal * kp) + (integralLocal * ki) + (differentailLocal * kd)
        
        # ---> Moving the robot and waiting a few miliseconds <---
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed - correction), int(speed + correction))
        await runloop.sleep_ms(dt) # ---> This is done so that the robot actually has time to move <---

        # ---> Object Stall Detection <---
        summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
        if(summyPositions - tresholdStallDetection < lastRelativePosition):
            break; # --> it means that the robot got stuck and we need to finish the function <---
        lastRelativePosition = summyPositions

    # ---> Stopping the robot if necessary and leaving the integral & error for the next functions <--- 
    previousError = lastErrorLocal; previousIntegral = integralLocal
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

    return

async def main():
    motor_pair.pair(motor_pair.PAIR_1, leftDriveBaseMotor, rightDriveBaseMotor)

    # ---> Only Proportional - Integral Controller (The differential is tricky because of the dt value) <---
    await gyroForwards(750, 20, kp = 0.5, ki = 0.75, kd = 0, dt = 2)

    # ---> Proportional Controller with Steering (Target != 0) <---
    await gyroForwards(750, 25, 100, kp = 0.75, ki = 0, kd = 0, dt = 1)


runloop.run(main())
