import runloop, hub, motor, motor_pair, color
from hub import motion_sensor as gs

'''
    # ---> Defining the hub's motors & ports <---
    leftDriveBaseMotor = hub.port.B; rightDriveBaseMotor = hub.port.D
    leftSystemMotor = hub.port.A; rightSystemMotor = hub.port.C
    leftColorSensor = hub.port.E; rightColorSensor = hub.port.F
'''

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase(object):
    def __init__( self, leftMotor: int, rightMotor: int, centimeterToDegrees: float, brakeMethod ) -> None:

        # ---> Defining the pair that is made out of [leftMotor, rightMotor] <---
        motor_pair.pair(motor_pair.PAIR_1, leftMotor, rightMotor)
        self.oneCentimeterToDegrees: float = centimeterToDegrees
        self.leftMotor = leftMotor; self.rightMotor = rightMotor
        self.pair: int = motor_pair.PAIR_1; self.brake = brakeMethod
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 125

        # self.initRun() # ---> Already used when starting run 1 <---

        return None

    def initRun( self ) -> None:
        # ---> Limiting and applying speeds <---
        self.leftSpeed: int = 0; self.rightSpeed: int = 0
        self.startSpeed: int = 0; self.endSpeed: int = 0
        self.speed: int = 0

        # ---> Limit for the correction so self.position is precise <---
        self.plusCorrection: int = 0; self.minusCorrection: int = 0
        self.thresholdCorrection: int = 0; # ---> Min(plus, minus) <---

        # ---> Object stall Detection in the PID Controller <---
        self.position: int = 0; self.lastPosition: int = 0;
        self.iterator: int = 0; self.wasStuck: bool = False

        # ---> Proportional - Integral - Derivative Controller <---
        self.error: float = 0; self.lastError: float = 0
        self.proportional: float = 0; self.integral: float = 0
        self.derivative: float = 0; self.correction: float = 0

        # ---> Faster Calculations <---
        self.previousDistance: float = 0; self.currentReachDistance: int = 0
        self.lastFunction: int = 0; self.accelerationOneDegree: float = 0
        self.reachDistance: int = 0; self.reachTarget: int = 0
        return None

    def limitAbs( self, value: float | int, valueMax: float | int, valueMin: float | int ) -> float | int:
        # ---> Limit a value so that it is in the range of [ valueMin, valueMax ] without changing its sign <---
        if( abs(value) > valueMax ): value = valueMax * ( value / abs( value ) )
        if( abs(value) < valueMin ): value = valueMin * ( value / abs( value ) )
        return value

    def limit( self, value: float | int, valueMax: float | int, valueMin: float | int ) -> float | int:
        # ---> Limit a value so that it is in the range of [ valueMin, valueMax ] (valueMin can be negative) <---
        if( value > valueMax ): value = valueMax
        if( value < valueMin ): value = valueMin
        return value

    def getSpeed( self ) -> int:
        return int( self.accelerationOneDegree * ( self.position - self.previousDistance ) )

    async def gyroForwards( self, distance: float, startSpeed: int, endSpeed: int, *, kp: float, ki: float, kd: float, dt: int, thresholdIntegral: float, thresholdStallDetection: int, iteratorStallDetection: int, stop: bool = True ) -> None:
        # ---> Documentation <---
        '''
            ---> Parameters <---
            distance - the distance ( in centimeters ) that the robot will travel \n
            startSpeed - the speed that the robot will start with ( recommended to be in the range of [175 , 1025] ) \n
            endSpeed - the speed that the robot will end with ( recommended to be in the range of [175 , 1025] ) \n
            startSpeed | endSpeed - used for accelerating and decelerating, along with having a steady speed \n

            kp, ki, kd, dt - constants for the Proportional - Integral - Derivative Controller \n
            thresholdIntegral - a value that keeps the integral in the range of [thresholdIntegral, -thresholdIntegral] \n
            thresholdStallDetection - a value that tells us if the robot is stuck or not \n
            iteratorStallDetection - a value that tells the robot how frequently we want to check if it's stuck or not \n
            stop - an On / Off value that tells the robot if we want it to stop after reaching the target \n
            stop - a value that is used when we have two following functions of the same type \n

            ---> For more information about these parameters please take a look in our technical notebook <---
        '''

        # ---> Initialization <---
        self.startSpeed = abs( startSpeed ); self.endSpeed = abs( endSpeed ); self.iterator = 0; self.iterator = 0
        thresholdIntegral = abs( thresholdIntegral ); iteratorStallDetection = abs( iteratorStallDetection )
        self.currentReachDistance = int( distance * 2 * self.oneCentimeterToDegrees )
        kp = abs( kp ); ki = abs( ki ); kd = abs( kd ); dt = abs( dt )

        # ---> Error Checking <---
        if( iteratorStallDetection == 0 ): print( "Error - iteratorStallDetection can't be 0" ); return None
        if( startSpeed > self.pairMaxSpeed ): print( "Error - startSpeed out of bounds" ); return None
        if( startSpeed < self.pairMinSpeed ): print( "Error - startSpeed out of bounds" ); return None
        if( endSpeed > self.pairMaxSpeed ): print( "Error - endSpeed out of bounds" ); return None
        if( endSpeed < self.pairMinSpeed ): print( "Error - endSpeed out of bounds" ); return None
        if( dt == 0 ): print( "Error - dt can't be 0" ); return None

        # ---> Automatization for backend ( precalculations )<---
        if( self.lastFunction == 1 ): # Gyro Forwards
            self.reachDistance += self.currentReachDistance
            if(self.wasStuck): return None
        else: # Null / Gyro Backwards / Turn Left / Turn Right
            # ---> Object Stall Detection + Distance Checking <---
            motor.reset_relative_position(self.leftMotor, 0)
            motor.reset_relative_position(self.rightMotor, 0)
            self.reachDistance = self.currentReachDistance
            self.position = 0; self.lastPosition = 0

            # ---> PID Controller <---
            self.error = 0; self.lastError = 0
            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.correction = 0

        # ---> Feed - Forward for getSpeed() ( for faster calculations ) <---
        self.accelerationOneDegree = float( self.endSpeed - self.startSpeed ) / float( self.currentReachDistance )
        self.previousDistance = self.reachDistance - self.currentReachDistance
        self.wasStuck = False

        # ---> Proportional - Integral - Derivative Controller with Object Stall detection <---
        while ( self.position <= self.reachDistance ):

            # ---> The Proportional - Integral - Derivative Controller <---
            self.error = ( self.reachTarget - gs.tilt_angles()[0] )
            self.proportional = ( self.error )
            self.integral += ( self.error * dt )
            self.derivative = ( self.error - self.lastError ) / dt
            self.lastError = ( self.error )

            # ---> Calculating the speed & threshold for the correction <---
            self.speed = int( self.startSpeed + self.getSpeed() )
            self.plusCorrection = ( self.pairMaxSpeed - abs( self.speed ) )
            self.minusCorrection = ( abs( self.speed ) - self.pairMinSpeed )
            self.thresholdCorrection = min( self.plusCorrection, self.minusCorrection )

            # ---> Limiting the correction & integral <---
            self.integral = self.limit( self.integral, thresholdIntegral, -thresholdIntegral )
            self.correction = ( ( self.proportional * kp ) + ( self.integral * ki ) + ( self.derivative * kd ))
            self.correction = self.limit( self.correction, self.thresholdCorrection, -self.thresholdCorrection )

            # ---> Calculating the speed needed and applying it <---
            self.leftSpeed = int( self.speed - self.correction )
            self.rightSpeed = int( self.speed + self.correction )
            motor_pair.move_tank( self.pair, self.leftSpeed, self.rightSpeed )

            # --> Object Stall Detection <---
            await runloop.sleep_ms( dt ); self.iterator += 1
            self.position = abs( motor.relative_position( self.leftMotor ) + motor.relative_position( self.rightMotor ) )
            if( self.iterator % iteratorStallDetection == 0 ):
                if( self.position - thresholdStallDetection < self.lastPosition ):
                    print( "Robot stuck" ); self.wasStuck = True; break; # ---> It means that the robot is stuck <---
                self.lastPosition = self.position; self.iterator = 0

        if( stop ): motor_pair.stop( self.pair, stop = self.brake )
        self.lastFunction = 1; return None

# ---> The class for the upper part of the robot that is responsible for making the systems move <---
class Systems():
    def __init__( self, leftMotor: int, rightMotor: int ) -> None:
        self.leftMotor = leftMotor; self.rightMotor = rightMotor; return None
    async def rotateLeftMotor( self, degrees: int, speed: int ) -> None:
        await motor.run_for_degrees( self.leftMotor, degrees, speed ); return None
    async def rotateRightMotor( self, degrees: int, speed: int ) -> None:
        await motor.run_for_degrees( self.rightMotor, degrees, speed ); return None

# ---> The main program <---
async def main():
    driveBase = DriveBase(hub.port.B, hub.port.D, 20.45, motor.SMART_BRAKE); systems = Systems(hub.port.A, hub.port.C)

runloop.run(main())
