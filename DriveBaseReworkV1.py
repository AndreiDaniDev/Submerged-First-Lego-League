import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

# ---> Global Values that represent the motor ports & variables <---

leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
leftColorSensor = hub.port.E; rightColorSensor = hub.port.F

# ---> Start of all of our functions <---
class DriveBase:
    def __init__(self) -> None:
        pass 

# ---> The class for the upper part of the robot that makes the systems move <---
class Systems(): 
    def __init__(self) -> None: pass
    async def lowerLeftArm(degrees: int, speed: int) -> None:
        await motor.run_for_degrees(leftSystemMotor, -degrees, speed); return None
    async def raiseLeftArm(degrees: int, speed: int) -> None:
        await motor.run_for_degrees(leftSystemMotor, degrees, speed); return None
    async def lowerRightArm(degrees: int, speed: int) -> None:
        await motor.run_for_degrees(rightSystemMotor, -degrees, speed); return None
    async def raiseRightArm(degrees: int, speed: int) -> None:
        await motor.run_for_degrees(rightSystemMotor, degrees, speed); return None

async def main():
    motor_pair.pair(motor_pair.PAIR_1, leftDriveBaseMotor, rightDriveBaseMotor)
    driveBase = DriveBase(); systems = Systems()

runloop.run(main())
