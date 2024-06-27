import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

'''
    leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
    leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
    leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
'''

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase:
    def __init__(self, leftMotor, rightMotor) -> None:
        pass 

# ---> The class for the upper part of the robot that makes the systems move <---
class Systems(object): 
    def __init__(self, leftMotor: int, rightMotor: int) -> None: self.leftMotor = leftMotor; self.rightMotor = rightMotor; return None
    async def lowerLeftArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.leftMotor, -degrees, speed); return None
    async def raiseLeftArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.leftMotor, degrees, speed); return None
    async def lowerRightArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.rightMotor, -degrees, speed); return None
    async def raiseRightArm(self, degrees: int, speed: int) -> None: await motor.run_for_degrees(self.rightMotor, degrees, speed); return None

async def main():
    driveBase = DriveBase(); systems = Systems(hub.port.A, hub.port.C)
runloop.run(main())
