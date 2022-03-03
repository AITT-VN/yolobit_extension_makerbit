import machine, math, pca9685, time
from machine import Pin

P_A1 = 11  # adapt to your wiring
P_A2 = 10 # ditto
P_B1 = 8 # ditto
P_B2 = 9 # ditto
delay = 0.005 # time to settle

# Define PWM outputs for each of two available steppers.
_STEPPERS = ((11, 10, 8, 9), (12, 13, 15, 14))

class Stepper():
    def __init__(self, pca9685_obj):
        self.pca9685 = pca9685.PCA9685(__i2c, 0x40)
        self.pca9685.freq(50)
        self.reset()

    def _pin(self, pin, value):
        if value == 1:
            self.pca9685.pwm(pin, 4096, 0)
        else:
            self.pca9685.pwm(pin, 0, 0)
    
    def setStepper(self,in1, in2, in3, in4):
        self._pin(P_A1, in1)
        self._pin(P_A2, in2)
        self._pin(P_B1, in3)
        self._pin(P_B2, in4)
        time.sleep(delay)

    def forwardStep(self):
        self.setStepper(1, 0, 1, 0)
        self.setStepper(0, 1, 1, 0)
        self.setStepper(0, 1, 0, 1)
        self.setStepper(1, 0, 0, 1)

    def backwardStep(self):
        self.setStepper(1, 0, 0, 1)
        self.setStepper(0, 1, 0, 1)
        self.setStepper(0, 1, 1, 0)
        self.setStepper(1, 0, 1, 0)
        
    def reset(self):
        # Reset to 0, no holding, these are geared, you can't move them
        self._pin(P_A1, 0)
        self._pin(P_A2, 0)
        self._pin(P_B1, 0)
        self._pin(P_B2, 0)

    def test(self):
        print('forward')
        for i in range(256):
            self.forwardStep() 
        print('backward')
        for i in range(256):
            self.backwardStep() 

try:
    from machine import SoftI2C
    __i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=200000)
except Exception as e:
    print("SoftI2C not exist. Using old I2C", e)
    from machine import I2C
    __i2c = I2C(scl=Pin(22), sda=Pin(21), freq=200000)
    
pca9685_obj = pca9685.PCA9685(__i2c, 0x40)
stepper = Stepper(pca9685_obj)


