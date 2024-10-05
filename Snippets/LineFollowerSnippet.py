import runloop, hub, motor, motor_pair, color, math, time, color_sensor
from hub import motion_sensor as gs

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase(object):

    def __init__(self, leftMotorDB: int, rightMotorDB: int, leftMotorSYS: int, rightMotorSYS: int, leftColorSensor: int, rightColorSensor: int, oneUnit: int, scale: int, brakeMethod) -> None:

        # ---> Defining the constant values that won't change in any run <---
        motor_pair.pair(motor_pair.PAIR_1, leftMotorDB, rightMotorDB)
        self.oneUnit: int = abs(oneUnit); self.scaleUnit: int = scale
        self.leftMotorDB = leftMotorDB; self.rightMotorDB = rightMotorDB
        self.leftMotorSYS = leftMotorSYS; self.rightMotorSYS = rightMotorSYS
        self.leftColorSensor = leftColorSensor; self.rightColorSensor = rightColorSensor
        self.pair: int = motor_pair.PAIR_1; self.brake = brakeMethod
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 100
        self.usualMaxSpeed: int = 1000; self.usualMinSpeed: int = 200
        self.maxAngle = 3350; self.maxOneTurnAngle = 1650
        
        return None

    async def lineFollowerSimple(self, speed: int = 250, speed2: int = 100, *, blackValue: int = 50) -> None:
        while(1): # ---> Follow the edge of the line <---
            if(color_sensor.reflection(self.rightColorSensor) < blackValue):
                motor_pair.move_tank(self.pair, speed, speed2)
            else: motor_pair.move_tank(self.pair, speed2, speed)
        return None

driveBase = DriveBase(hub.port.A, hub.port.C, hub.port.B, hub.port.D, hub.port.F, hub.port.E, 2045, 1000, motor.SMART_BRAKE)

# ---> The main program <---
async def main() -> None:
    await driveBase.lineFollowerSimple(250, 100);    
    return None

runloop.run(main())
