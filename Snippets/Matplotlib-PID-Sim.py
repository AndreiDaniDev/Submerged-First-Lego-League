import numpy as np, matplotlib.pyplot as plt
import random

# ---> Code <---

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase(object):

    def __init__(self) -> None:
        self.kp: int = 0; self.ki: int = 0; self.kd: int = 0; self.dt: int = 1

        self.kscale: int = 1; self.integralLimit: int = 0

        self.angle: int = 0; self.error: int = 0; self.previousError: int = 0
        self.proportional: int = 0; self.integral: int = 0
        self.derivative: int = 0; self.controllerOutput: int = 0

        return None

    def limit(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [valueMin, valueMax] and return it <---
        return min(max(value, valueMin), valueMax)

    def limitAbs(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [valueMin, valueMax] based on its sign and return it <---
        return (min(max(value, valueMin), valueMax) * (value / abs(value)) if (value != 0) else 0)

    def PID_Controller(self, error: float, sign: int) -> None:

        '''
        ---> Parameters for PID_Controller (Proportional - Integral - Derivative Controller) <---
        error: The error that was read by the sensor (in our case the gyroscopic sensor)
        integralLimit: A value that acts like a limit so that the integral won't overshoot from regular boundaries
        sign: A value that signifies the sign of the controller's output
        scaleConstants: kp, kd and ki are written as (x / scaleConstants, where x is either kp, kd or ki (integers))
        '''

        self.error = (error)
        self.proportional = (self.error)
        self.integral += (self.error * self.dt)
        self.integral = int(self.limit(self.integral, self.integralLimit, -self.integralLimit))
        self.derivative = (self.error - self.previousError)
        self.previousError = (self.error)

        self.controllerOutput = sign * ( 
            self.proportional * self.kp +
            self.integral * self.ki +
            self.derivative * self.kd // self.dt
        ) // self.kscale

        return None

    def setConstants(self, kp: int, ki: int, kd: int, scale: int, integralLimit: int = 50000):
        self.kp = kp; self.ki = ki; self.kd = kd; self.kscale = scale; self.integralLimit = integralLimit

driveBase = DriveBase()

debugData = []; iterations: int = 50
angle: int = 0; target: int = 900 # Decidegrees

driveBase.setConstants(1750, 5, 750, 10000)

summy: int = 0

for x in range(0, iterations):
    driveBase.PID_Controller(target - angle, 1)
    angle += driveBase.controllerOutput

    debugData.append([angle, summy])

    summy += angle

plt.plot(range(iterations), debugData, label = 'Angle over time')
plt.axhline(target, color = '#222222', linestyle = '--', label = 'Target')

# ---> No more code <---

plt.legend()
plt.show()

print(debugData)
