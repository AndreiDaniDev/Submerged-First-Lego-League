import runloop, hub, motor, motor_pair, color, math
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
        motor_pair.pair( motor_pair.PAIR_1, leftMotor, rightMotor )
        self.oneCentimeterToDegrees: float = centimeterToDegrees
        self.leftMotor = leftMotor; self.rightMotor = rightMotor
        self.pair: int = motor_pair.PAIR_1; self.brake = brakeMethod
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 125
        # self.initRun() # ---> Already used when starting run 1 <---

        # ---> Limiting and applying speeds <---
        self.leftSpeed: int = 0; self.rightSpeed: int = 0
        self.startSpeed: int = 0; self.endSpeed: int = 0
        self.speed: int = 0

        # ---> Limit for the correction so self.position is precise <---
        self.plusCorrection: int = 0; self.minusCorrection: int = 0
        self.thresholdCorrection: int = 0; # ---> Min(plus, minus) <---

        # ---> Object stall Detection in the PID Controller <---
        self.position: int = 0; self.lastPosition: int = 0
        self.iterator: int = 0; self.wasStuck: bool = False

        # ---> Proportional - Integral - Derivative Controller <---
        self.error: float = 0; self.lastError: float = 0
        self.proportional: float = 0; self.integral: float = 0
        self.derivative: float = 0; self.correction: float = 0

        # ---> Faster Calculations <---
        self.previousDistance: float = 0; self.currentReachDistance: int = 0
        self.lastFunction: int = 0; self.reachDistance: int = 0; self.reachTarget: int = 0
        self.accelerationOneDegree: float = 0; # ---> Liniar Gyro <---
        self.ptk: int = 0; # self.powerOf2toKminus1 # ---> Dynamic Gyro <---

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
        self.position: int = 0; self.lastPosition: int = 0
        self.iterator: int = 0; self.wasStuck: bool = False

        # ---> Proportional - Integral - Derivative Controller <---
        self.error: float = 0; self.lastError: float = 0
        self.proportional: float = 0; self.integral: float = 0
        self.derivative: float = 0; self.correction: float = 0

        # ---> Faster Calculations <---
        self.previousDistance: float = 0; self.currentReachDistance: int = 0
        self.lastFunction: int = 0; self.reachDistance: int = 0; self.reachTarget: int = 0
        self.accelerationOneDegree: float = 0; # ---> Liniar Gyro <---
        self.ptk: int = 0; # self.powerOf2toKminus1 # ---> Dynamic Gyro <---
        return None

    def limit( self, value: int | float, valueMax: int | float, valueMin: int | float ) -> int | float:
        # ---> Make a value be in range of [ valueMin, valueMax ] and return it <---
        return min( max( value, valueMin ), valueMax )

    def limitAbs( self, value: int | float, valueMax: int | float, valueMin: int | float ) -> int | float:
        # ---> Make a value be in range of [ valueMin, valueMax ] based on its sign and return it <---
        return ( min( max( value, valueMin ), valueMax ) * ( value / abs( value ) ) if ( value != 0 ) else 0)

    def getSpeedLiniar( self ) -> int: # ---> Used for speeding up / slowing down / constant speed <---
        return int( self.accelerationOneDegree * ( self.position - self.previousDistance ) )

    def getPointOnCurve( self, mode: int, time: float, k: int) -> float:
        if( mode == 1 ): return time # ---> Liniar Acceleration <---
        elif( mode == 2 ): return math.pow(time, k) # ---> Exponential Acceleration <---
        elif( mode == 3 ): return 1 - math.pow(1 - time, k) # ---> Inverse Exponential Acceleration <---
        elif( mode == 4 ): return (self.ptk * math.pow(time, k)) if (time <= 0.5) else (1 - self.ptk * math.pow(1 - time, k)) # ---> Interpolation Acceleration <---
        return 0 # ---> Invalid Mode <---

    def getSpeedOnCurve( self, mode: int, step: int, k: int) -> int:
        return int( self.speedDiff * self.getPointOnCurve(mode, math.floor( step * (self.position - self.previousDistance) / self.currentReachDistance ) / step, k ) )

    async def gyroForwardsLiniar( self, distance: float, startSpeed: int, endSpeed: int, *, kp: float, ki: float, kd: float, dt: int, thresholdIntegral: float, thresholdStallDetection: int, iteratorStallDetection: int, stop: bool = True ) -> None:
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
        thresholdIntegral = abs( thresholdIntegral ); iteratorStallDetection = abs( iteratorStallDetection )
        self.startSpeed = abs( startSpeed ); self.endSpeed = abs( endSpeed ); self.iterator = 0
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
            self.previousDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
            if( self.wasStuck ): return None
        else: # Null / Gyro Backwards / Turn Left / Turn Right
            # ---> Object Stall Detection + Distance Checking <---
            motor.reset_relative_position( self.leftMotor, 0 )
            motor.reset_relative_position( self.rightMotor, 0 )
            self.reachDistance = self.currentReachDistance
            self.previousDistance = 0
            self.position = 0; self.lastPosition = 0
            self.wasStuck = False

            # ---> PID Controller <---
            self.error = 0; self.lastError = 0
            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.correction = 0

        # ---> Feed - Forward for getSpeed() ( for faster calculations ) <---
        self.accelerationOneDegree = float( self.endSpeed - self.startSpeed ) / float( self.currentReachDistance )

        # ---> Proportional - Integral - Derivative Controller with Object Stall detection <---
        while ( self.position <= self.reachDistance ):

            # ---> The Proportional - Integral - Derivative Controller <---
            self.error = ( self.reachTarget - gs.tilt_angles()[0] )
            self.proportional = ( self.error )
            self.integral += ( self.error * dt )
            self.derivative = ( self.error - self.lastError ) / dt
            self.lastError = ( self.error )

            # ---> Calculating the speed & threshold for the correction <---
            self.speed = int( self.startSpeed + self.getSpeedLiniar() )
            self.plusCorrection = ( self.pairMaxSpeed - abs( self.speed ) )
            self.minusCorrection = ( abs( self.speed ) - self.pairMinSpeed )
            self.thresholdCorrection = min( self.plusCorrection, self.minusCorrection )

            # ---> Calculating & limiting the correction & integral <---
            self.integral = self.limit( self.integral, thresholdIntegral, -thresholdIntegral )
            self.correction = (( self.proportional * kp ) + ( self.integral * ki ) + ( self.derivative * kd ))
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
                    'print( "Robot - stuck" )'; self.wasStuck = True; break; # ---> It means that the robot is stuck <---
                self.lastPosition = self.position; self.iterator = 0

        # ---> Stopping (if said so) and exiting <---
        if( stop ): motor_pair.stop( self.pair, stop = self.brake )
        self.lastFunction = 1; return None

    async def gyroForwardsDynamic( self, distance: float, startSpeed: int, endSpeed: int, *, kp: float, ki: float, kd: float, dt: int, thresholdIntegral: float, thresholdStallDetection: int, iteratorStallDetection: int, k: int, step: int, mode: int, stop: bool = True ) -> None:
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
        thresholdIntegral = abs( thresholdIntegral ); iteratorStallDetection = abs( iteratorStallDetection )
        self.startSpeed = abs( startSpeed ); self.endSpeed = abs( endSpeed ); self.iterator = 0
        self.currentReachDistance = int( distance * 2 * self.oneCentimeterToDegrees )
        kp = abs( kp ); ki = abs( ki ); kd = abs( kd ); dt = abs( dt )

        # ---> Error Checking <---
        if( iteratorStallDetection == 0 ): print( "Error - iteratorStallDetection can't be 0" ); return None
        if( startSpeed > self.pairMaxSpeed ): print( "Error - startSpeed out of bounds" ); return None
        if( startSpeed < self.pairMinSpeed ): print( "Error - startSpeed out of bounds" ); return None
        if( endSpeed > self.pairMaxSpeed ): print( "Error - endSpeed out of bounds" ); return None
        if( endSpeed < self.pairMinSpeed ): print( "Error - endSpeed out of bounds" ); return None
        if( 1 > mode or mode > 4 ): print( "Error - Mode is out of bounds" ); return None
        if( dt == 0 ): print( "Error - dt can't be 0" ); return None

        # ---> Automatization for backend ( precalculations )<---
        if( self.lastFunction == 1 ): # Gyro Forwards
            self.previousDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
            if( self.wasStuck ): return None
        else: # Null / Gyro Backwards / Turn Left / Turn Right
            # ---> Object Stall Detection + Distance Checking <---
            motor.reset_relative_position( self.leftMotor, 0 )
            motor.reset_relative_position( self.rightMotor, 0 )
            self.reachDistance = self.currentReachDistance
            self.previousDistance = 0
            self.position = 0; self.lastPosition = 0
            self.wasStuck = False

            # ---> PID Controller <---
            self.error = 0; self.lastError = 0
            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.correction = 0

        # ---> Feed Forward for getSpeedOnCurve() <---
        self.ptk = int( math.pow( 2, k - 1 ) ); self.speedDiff = ( self.endSpeed - self.startSpeed )

        # ---> Proportional - Integral - Derivative Controller with Object Stall detection <---
        while ( self.position <= self.reachDistance ):

            # ---> The Proportional - Integral - Derivative Controller <---
            self.error = ( self.reachTarget - gs.tilt_angles()[0] )
            self.proportional = ( self.error )
            self.integral += ( self.error * dt )
            self.derivative = ( self.error - self.lastError ) / dt
            self.lastError = ( self.error )

            # ---> Calculating the speed & threshold for the correction <---
            self.speed = int( self.startSpeed + self.getSpeedOnCurve(mode, step, k) )
            self.plusCorrection = ( self.pairMaxSpeed - abs( self.speed ) )
            self.minusCorrection = ( abs( self.speed ) - self.pairMinSpeed )
            self.thresholdCorrection = min( self.plusCorrection, self.minusCorrection )

            # ---> Calculating & limiting the correction & integral <---
            self.integral = self.limit( self.integral, thresholdIntegral, -thresholdIntegral )
            self.correction = (( self.proportional * kp ) + ( self.integral * ki ) + ( self.derivative * kd ))
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
                    'print( "Robot - stuck" )'; self.wasStuck = True; break; # ---> It means that the robot is stuck <---
                self.lastPosition = self.position; self.iterator = 0

        # ---> Stopping (if said so) and exiting <---
        if( stop ): motor_pair.stop( self.pair, stop = self.brake )
        self.lastFunction = 1; return None

    async def turnLeft( self, degrees: float, speedMax: int, speedMin: int, *, kp: float, ki: float, kd: float, dt: int, kLeft: float, kRight: float, thresholdError: int, thresholdIntegral: float, thresholdStallDetection: int, iteratorStallDetection: int, stop: bool = True ) -> None:
        # ---> Documentation <---
        '''
            ---> Parameters <---
            degrees - the target that the robot needs to reach (in degrees, with one decimal precision) \n
            speedMax, speedMin - two pointers that are used to limit the controllers speed \n
            kp, ki, kd, dt - constants for the Proportional - Integral - Derivative Controller \n
            kRight, kleft - constants that are multiplied with the correction when applying it \n
                        - they are values that are in the range of [-1, 1] \n
                        - the recommended values for these are: [[-1, 1], [-1, 0], [0, 1]] - shortest Distance
                                                                [[1, -1], [0, -1], [1, 0]] - longest Distance
            thresholdError - the value that is accepted by the PID Controller (can't be < 5) \n
            thresholdIntegral - the value that keeps the integral from overshooting \n
            thresholdStallDetection - the value that tells the robot if it's stuck \n
            iteratorStallDetection - how frequently we want to check if it's stuck \n
            stop - an On / Off Value that tells the robot if it should stop after reaching the desired distance \n
            ---> For more information about these parameters please take a look in our technical notebook <---
        '''

        # ---> Initialization for fixing and getting the parameters <---
        self.startSpeed = abs( speedMax ); self.endSpeed = abs( speedMin )
        kp = abs( kp ); ki = abs( ki ); kd = abs( kd ); dt = abs( dt )
        thresholdStallDetection = abs( thresholdStallDetection )
        iteratorStallDetection = abs( iteratorStallDetection )
        thresholdIntegral = abs( thresholdIntegral )
        thresholdError = abs( thresholdError )
        degrees = abs( degrees )

        # ---> Error Checking <---
        if( iteratorStallDetection == 0 ): print( "Error - iteratorStallDetection can't be 0" ); return None
        if( speedMax < speedMin): print( "Error - speedMin is greater than speedMax" ); return None
        if( speedMax > self.pairMaxSpeed ): print( "Error - speedMax out of bounds" ); return None
        if( speedMax < self.pairMinSpeed ): print( "Error - speedMax out of bounds" ); return None
        if( speedMin > self.pairMaxSpeed ): print( "Error - speedMin out of bounds" ); return None
        if( speedMin < self.pairMinSpeed ): print( "Error - speedMin out of bounds" ); return None
        if( kRight == kLeft ): print("Error - kRight mustn't be equal to kLeft"); return None
        if( kRight > 1 or kRight < -1 ): print( "Error - kRight out of bounds" ); return None
        if( kLeft > 1 or kLeft < -1 ): print( "Error - kLeft out of bounds" ); return None
        if( degrees > 167.5 ): print( "Error - degrees out of bounds" ); return None
        if( thresholdError < 5 ): print( "Error - thresholdError < 5" ); return None
        if( dt == 0 ): print( "Error - dt can't be 0" ); return None

        # ---> Getting the error left so that it's corrected by the PID Controller <---
        gs.reset_yaw( gs.tilt_angles()[0] - self.reachTarget )
        self.reachTarget = -int( degrees * 10 )

        # ---> Initialization PID Controller <---
        self.error = ( self.reachTarget - gs.tilt_angles()[0] )
        self.proportional = 0; self.integral = 0
        self.derivative = 0; self.correction = 0
        self.lastError = 0

        # ---> Initialization for Object Stall Detection <---
        if(self.wasStuck and self.lastFunction == 3):
            return None; # It was already stuck so it can't go in the same direction
        motor.reset_relative_position( self.leftMotor, 0 )
        motor.reset_relative_position( self.rightMotor, 0 )
        self.position = 0; self.lastPosition = 0
        self.iterator = 0; self.wasStuck = False

        # ---> Proportional - Integral - Derivative Controller with Object Stall Detection <---
        while(self.error > thresholdError or self.error < -thresholdError):

            # ---> The Proportional - Integral - Derivative Controller <---
            self.error = ( self.reachTarget - gs.tilt_angles()[0] )
            self.proportional = ( self.error )
            self.integral += ( self.error * dt )
            self.derivative = ( self.error - self.lastError ) / dt
            self.lastError = ( self.error )

            # ---> Calculating the correction & speeds, and limiting them & the integral <---
            self.integral = self.limit( self.integral, thresholdIntegral, -thresholdIntegral )
            self.correction = (( self.proportional * kp ) + ( self.integral * ki ) + ( self.derivative * kd ))
            self.leftSpeed = int( self.limitAbs( self.correction * kLeft, self.startSpeed, self.endSpeed) )
            self.rightSpeed = int( self.limitAbs( self.correction * kRight, self.startSpeed, self.endSpeed) )

            # ---> Applying the speeds <---
            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)

            # ---> Object Stall Detection <---
            self.position = abs( motor.relative_position( self.leftMotor ) + motor.relative_position( self.rightMotor ) )
            if( self.iterator % iteratorStallDetection == 0 ):
                if( self.position - thresholdStallDetection < self.lastPosition ):
                    'print( "Robot - stuck" )'; self.wasStuck = True; break; # ---> It means that the robot is stuck <---
                self.lastPosition = self.position; self.iterator = 0

        # ---> Stopping (if said so) and exiting <---
        if( stop ): motor_pair.stop( self.pair, stop = self.brake )
        self.lastFunction = 3; return None

    def OnGraph( self, distance: float, startSpeed: int, endSpeed: int, *, k: int, step: int, mode: int ) -> None:
        # ---> This function is for debugging purposes only <---

        '''
            ---> Parameters <--- \n

            distance - the distance (in centimeters) that will be simulated \n
            startSpeed - the speed that we will start with \n
            endSpeed - the speed that we will finish with \n

            k - exponent factor for getSpeedOnCurve (modes 2 to 4) \n
            step - the approximation that will be done for all values between 0 and 1 \n
            mode - the mode for getSpeedOnCurve - (mode 1 - liniar acceleration, mode 2 - exponential acceleration, mode 3 - inverse exponentail acceleration, mode 4 - mode 2 and 3 combined) (for more info about each acceleration mode please visit https://www.desmos.com/calculator/mf6lpbowao ) \n

            ---> For more information about these parameters and this function please take a look in out technical notebook <--- \n
        '''

        # ---> Initialization <---
        self.startSpeed = abs( startSpeed ); self.endSpeed = abs( endSpeed ); self.iterator = 0
        self.currentReachDistance = int( distance * 2 * self.oneCentimeterToDegrees )

        # ---> Error Checking <---
        if( startSpeed > self.pairMaxSpeed ): print( "Error - startSpeed out of bounds" ); return None
        if( startSpeed < self.pairMinSpeed ): print( "Error - startSpeed out of bounds" ); return None
        if( endSpeed > self.pairMaxSpeed ): print( "Error - endSpeed out of bounds" ); return None
        if( endSpeed < self.pairMinSpeed ): print( "Error - endSpeed out of bounds" ); return None
        if( 1 > mode or mode > 4 ): print( "Error - Mode is out of bounds" ); return None

        # ---> Automatization for backend ( precalculations )<---
        if( self.lastFunction == 1 ): # Gyro Forwards
            # ---> Adding distances <---
            self.previousDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
        else: # Null / Gyro Backwards / Turn Left / Turn Right
            # ---> Object Stall Detection + Distance Checking <---
            self.reachDistance = self.currentReachDistance
            self.previousDistance = 0
            self.position = 0; self.lastPosition = 0
            self.wasStuck = False

            # ---> PID Controller <---
            self.error = 0; self.lastError = 0
            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.correction = 0

        # ---> Feed Forward for getSpeedOnCurve() <---
        self.ptk = int(math.pow( 2, k - 1 )); self.speedDiff = ( self.endSpeed - self.startSpeed )

        for self.position in range(self.position, self.reachDistance + 1, 1):
            print(self.startSpeed + self.getSpeedOnCurve(mode, step, k), end = " ")

        self.lastFunction = 1; print("\n"); return None

# ---> The class for the upper part of the robot that is responsible for making the systems move <---
class Systems(object):
    def __init__( self, leftMotor: int, rightMotor: int ) -> None:
        self.leftMotor = leftMotor; self.rightMotor = rightMotor; return None
    async def rotateLeftMotor( self, degrees: int, speed: int ) -> None:
        await motor.run_for_degrees( self.leftMotor, degrees, speed ); return None
    async def rotateRightMotor( self, degrees: int, speed: int ) -> None:
        await motor.run_for_degrees( self.rightMotor, degrees, speed ); return None

# ---> The main program <---
async def main():
    driveBase = DriveBase( hub.port.B, hub.port.D, 20.45, motor.SMART_BRAKE ); systems = Systems( hub.port.A, hub.port.C )
    
runloop.run(main())
