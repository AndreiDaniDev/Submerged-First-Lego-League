#14_06_2024 in spike 3.4.3 & Visual Studio Code
import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

# ---> Robot Ports <---
leftDriveBaseMotor = hub.port.B
rightDriveBaseMotor = hub.port.D
leftSystemMotor = hub.port.A
rightSystemMotor = hub.port.C
#leftColorSensor = NULL
#rightColorSensor = NULL

async def GyroForwards(speed: int, distance: float, target: int, *, kp: float = 0.4, ki: float = 0.2, kd: float = 0.001, lastError: float = 0, lastIntegral: float = 0, trajectory: int = 0, stop: bool = True, reset: bool = True, objectStallDetection: bool = True, tresholdStallDetection: int = 10):
    # ---> Needed to know <------> Documentation <---
    '''
        speed - the speed that is applied on the wheel's of the robot
        distance (cm) - the distance that the robot will travel
        target - a value that makes the robot move in a circle (affected by the PID Controller)
        kp, ki, kd - constants that are used in the PID controller
        lastError - used when speeding up / slowing down, usually after a func like this
        lastIntegral - just like lastError, but for the integral
        trajectory - usually used after a turn, so that the robot loses any error from the turn
        stop - used when speeding up / slowing down
        reset - used when there is only a gyro / we don't need that error
        objectStallDetection - if the robot gets stuck and this is true we exit the function
        tresholdStallDetection - used to see if the robot is stuck or still moving
    '''

    distance = int(distance * 20.45 * 2); lastErrorLocal = lastError; integralLocal = lastIntegral; lastRelativePosition = 0
    motor.reset_relative_position(leftDriveBaseMotor, 0); motor.reset_relative_position(rightDriveBaseMotor, 0)
    summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
    target = 0 # ---> It still remains a mistery of how this is implemented corectly <---
    # ---> PID Controller <---
    while(summyPositions < 2 * distance):
        
        errorLocal = trajectory - gs.tilt_angles()[0]
        proportionalLocal = (errorLocal - target) * kp
        differentailLocal = (errorLocal - lastErrorLocal) * kd
        integralLocal += errorLocal * ki; lastErrorLocal = errorLocal
        correction = proportionalLocal + differentailLocal + integralLocal
        
        # ---> Applying the speeds <---
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed - correction), int(speed + correction))
        
        # ---> Object Stall Detection <---
        await runloop.sleep_ms(5) # ---> This is done so that the robot actually has time to move <---
        summyPositions = abs(motor.relative_position(leftDriveBaseMotor)) + abs(motor.relative_position(rightDriveBaseMotor))
        if(summyPositions - tresholdStallDetection < lastRelativePosition):
            break; # --> it means that the robot got stuck <---
        lastRelativePosition = summyPositions
    
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    return
