    # s1 = Stepper(HALF_STEP, microbit.pin16, microbit.pin15, microbit.pin14, microbit.pin13, delay=5)    
    # s2 = Stepper(HALF_STEP, microbit.pin6, microbit.pin5, microbit.pin4, microbit.pin3, delay=5)   
    # #s1.step(FULL_ROTATION)
    # #s2.step(FULL_ROTATION)

    # runner = Driver()
    # runner.run([Command(s1, FULL_ROTATION, 1), Command(s2, FULL_ROTATION/2, -1)])
    # runner.run([Command(stepper, FULL_ROTATION, 1)])

import machine, math, pca9685, time
from machine import Pin

FULL_ROTATION = int(4075.7728395061727 / 8)

HALF_STEP = [
    [0, 0, 0, 1],
    [0, 0, 1, 1],
    [0, 0, 1, 0],
    [0, 1, 1, 0],
    [0, 1, 0, 0],
    [1, 1, 0, 0],
    [1, 0, 0, 0],
    [1, 0, 0, 1],
]

FULL_STEP = [
 [1, 0, 1, 0],
 [0, 1, 1, 0],
 [0, 1, 0, 1],
 [1, 0, 0, 1]
]

# Define PWM outputs for each of two available steppers.
_STEPPERS = ((11, 10, 8, 9), (12, 13, 15, 14))

class Command():
    """Tell a stepper to move X many steps in direction"""
    def __init__(self, stepper, steps, direction=1):
        self.stepper = stepper
        self.steps = steps
        self.direction = direction

class Driver():
    """Drive a set of motors, each with their own commands"""

    @staticmethod
    def run(commands):
        """Takes a list of commands and interleaves their step calls"""
        
        # Work out total steps to take
        max_steps = sum([c.steps for c in commands])

        count = 0
        while count != max_steps:
            for command in commands:
                # we want to interleave the commands
                if command.steps > 0:
                    command.stepper.step(1, command.direction)
                    command.steps -= 1
                    count += 1

class Stepper():
    def __init__(self, pca9685_obj, mode, delay=2):
        self.pca9685 = pca9685.PCA9685(__i2c, 0x40)
        self.pca9685.freq(50)
        self.mode = mode
        self.pin1 = 11
        self.pin2 = 10
        self.pin3 = 8
        self.pin4 = 9
        self.delay = delay  # Recommend 10+ for FULL_STEP, 1 is OK for HALF_STEP
        self.reset()
    
    def _pwm(self, pin, value):
        if value > 4095:
            self.pca9685.pwm(pin, 4096, 0)
        else:
            self.pca9685.pwm(pin, 0, value)

    def _pin(self, pin, value):
        if value == 1:
            self.pca9685.pwm(pin, 4096, 0)
        else:
            self.pca9685.pwm(pin, 0, 0)
    
    def step(self, count, direction=1):
        """Rotate count steps. direction = -1 means backwards"""
        for x in range(count):
            for bit in self.mode[::direction]:
                self._pin(self.pin1, bit[0])
                self._pin(self.pin2, bit[1])
                self._pin(self.pin3, bit[2])
                self._pin(self.pin4, bit[3])
                time.sleep_ms(self.delay)
        self.reset()
        
    def reset(self):
        # Reset to 0, no holding, these are geared, you can't move them
        self._pin(self.pin1, 0)
        self._pin(self.pin2, 0)
        self._pin(self.pin3, 0)
        self._pin(self.pin4, 0)

try:
    from machine import SoftI2C
    __i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=200000)
except Exception as e:
    print("SoftI2C not exist. Using old I2C", e)
    from machine import I2C
    __i2c = I2C(scl=Pin(22), sda=Pin(21), freq=200000)
    
pca9685_obj = pca9685.PCA9685(__i2c, 0x40)
stepper = Stepper(pca9685_obj, FULL_STEP, delay=5)


