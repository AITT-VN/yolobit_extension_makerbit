# steppers.py

# this has 2 classes
#   HBRIDGE - for using h-bridges
#   A4899 - for using a4899 stepper drivers (and similar)

#-----------------------------------------------
# imports
#-----------------------------------------------
import machine, math, pca9685, time
from machine import Pin

#-----------------------------------------------
# h-bridge stepper driver class
#-----------------------------------------------

class HBRIDGE:

    # this performs and tracks steps on a single motor with quad-h-bridge
    # this required 4 controller pins per motor

    # for bi-polar steppers, the 2 coils are A-->a and B-->b

    # for uni-polar steppers, the 4 coils are A-->G, B-->G, a-->G, and b-->G
    # where G is ground (default condition, but see invert=True)  

    # default is to use pin HIGH == h-bridge channel ON    
    # if LOW == h-bridge channel ON, set invert=True on init  

    # -----
    # modes
    # -----

    # all modes are set up as (A,B,a,b) (easier for viewing)
    # for bi-polar motors A-a and B-b are the 2 coils
    # for uni-polar motors A, B, a, and b are the 4 coils

    # full-step, single coil, 4-state
    mode1 = ((1,0,0,0),
             (0,1,0,0),
             (0,0,1,0),
             (0,0,0,1))
    
    # full-step, dual coil, 4-state
    mode2 = ((1,1,0,0),
             (0,1,1,0),
             (0,0,1,1),
             (1,0,0,1))

    # half-step, single and dual coil, 8-state
    mode3 = ((1,1,0,0),
             (0,1,0,0),
             (0,1,1,0),
             (0,0,1,0),
             (0,0,1,1),
             (0,0,0,1),
             (1,0,0,1),
             (1,0,0,0))

    # -----
    # init
    # -----

    def __init__(self,
                 pca9685_obj,
                 A,            # pin number for coil A
                 a,            # pin number for coil a
                 B,            # pin number for coil B
                 b,            # pin number for coil b
                 mode=1,       # see modes above
                 reverse=False,# reverse motor default direction
                 invert=False, # invert all mode pin states
                 sleep=False,  # start in sleep mode
                 sps=200,      # steps-per-second
                 smax=10240,   # max step count allowed
                 smin=-10240   # min step count allowed
                 ):
        self.pca9685 = pca9685.PCA9685(__i2c, 0x40)
        self.pca9685.freq(50)
        
        # set mode (delete others)
        if mode == 3:
            self.mode = self.mode3
            self.mode1,self.mode2 = None,None
        elif mode == 2:
            self.mode = self.mode2
            self.mode1,self.mode3 = None,None
        else:
            self.mode = self.mode1
            self.mode2,self.mode3 = None,None
        self.index = 0 # index of current state
        self.imax = len(self.mode)-1 # when to loop index

        # set reverse (just reverse self.mode)
        if reverse:
            # not for micropython: self.mode = self.mode[::-1]
            mode = list(self.mode)
            mode.reverse()
            self.mode = tuple(mode)

        # set invert mode and xstate (off state)
        self.xstate = (0,0,0,0)
        if invert:
            mode = []
            for state in self.mode:
                pins = []
                for pin in state:
                    if pin:
                        pins.append(0)
                    else:
                        pins.append(1)
                mode.append(tuple(pins))
            self.mode = tuple(mode)
            del mode
            self.xstate = (1,1,1,1)

        # set sleep and init state
        if sleep:
            self.isoff = True
            self.istate = self.xstate
        else:
            self.isoff = False
            self.istate = self.mode[self.index]

        # pin init
        self.p1 = A
        self.p2 = B
        self.p3 = a
        self.p4 = b

        # step tracking
        self.steps = 0 # current step count
        self.last = time.ticks_us() # ticks_us of last pset
        self.sps = int(sps or 200) # default steps per second
        self.smax = smax
        self.smin = smin
        
    def _pin(self, pin, value):
        if value == 1:
            self.pca9685.pwm(pin, 4096, 0)
        else:
            self.pca9685.pwm(pin, 0, 0)
 
    def zero(self):
        self.steps = 0

    def sleep(self):
        self.pset(self.xstate)
        self.isoff = True

    def wake(self):
        self.pset(self.mode[self.index])
        self.isoff = False

    def pset(self,state,waitfor=None):
        while waitfor and time.ticks_diff(time.ticks_us(),waitfor) < 0:
            time.sleep_us(10)
        self._pin(self.p1,self.istate[0])
        self._pin(self.p2,self.istate[1])
        self._pin(self.p3,self.istate[2])
        self._pin(self.p4,self.istate[3])
        self.last = time.ticks_us()

    def step(self,steps,sps=None,sleep=False):

        # limit steps
        #steps = min(steps,self.smax-self.steps)
        #steps = max(steps,self.smin-self.steps)
        steps = min(max(steps,self.smin-self.steps),self.smax-self.steps)

        # mode localize
        mode = self.mode
        imax = self.imax

        # local tracking
        index = self.index # most recent index
        sc = 0             # step count
        
        # accurate timing
        stime = int(round(1000000/max(1,(sps or self.sps)),0))
        #                 (   timenow -                  (lastpset + stime)) >= 0:
        if time.ticks_diff(time.ticks_us(),time.ticks_add(self.last, stime)) >= 0:
            waitfor = time.ticks_us()
        else:
            waitfor = time.ticks_add(self.last,stime)

        # wake
        if self.isoff:
            self.wake()

        # backward
        if steps < 0:
            for s in range(abs(steps)):
                if index == 0:
                    index = imax
                else:
                    index -= 1
                self.pset(mode[index],waitfor)
                waitfor = time.ticks_add(waitfor,stime)
                sc -= 1

        # forward
        else:
            for s in range(steps):
                if index == imax:
                    index = 0
                else:
                    index += 1
                self.pset(mode[index],waitfor)    
                waitfor = time.ticks_add(waitfor,stime)
                sc += 1

        # sleep
        if sleep:
            self.sleep()

        # done
        self.index = index
        self.steps += sc
        return self.steps

#-----------------------------------------------
# a4899 stepper driver class
#-----------------------------------------------

class A4899(HBRIDGE):

    # re-use as much of HBRIDGE as possible
    # keep all the function names the same

    def __init__(self,
                 step,         # pin number for step
                 direction,    # pin number for direction
                 enable=None,  # pin number for enable
                 reverse=False,# reverse motor default direction
                 sleep=False,  # start in sleep mode
                 sps=200,      # steps-per-second
                 smax=10240,   # max step count allowed
                 smin=-10240   # min step count allowed
                 ):

        # no modes (set by dip switch)
        self.mode1,self.mode2,self.mode3 = None,None,None

        # direction
        if reverse:
            self.forward = 1
            self.reverse = 0
        else:
            self.forward = 0
            self.reverse = 1

        # pin init
        self.ps = Pin(step,Pin.OUT,0)
        self.pd = Pin(direction,Pin.OUT,self.forward)
        if type(enable) == int:
            self.pe = Pin(enable,Pin.OUT,0)
        else:
            self.pe = None

        # set sleep and init state
        if sleep:
            self.sleep()
        else:
            self.wake()

        # step tracking
        self.steps = 0 # current step count
        self.last = time.ticks_us() # ticks_us of last pset
        self.sps = int(sps or 200) # default steps per second
        self.smax = smax
        self.smin = smin

    def sleep(self):
        # disable == high
        self.pe.value(1)
        self.isoff = True

    def wake(self):
        # enable == low
        self.pe.value(0)
        self.isoff = False

    def step(self,steps,sps=None,sleep=False):

        # limit steps
        #steps = min(steps,self.smax-self.steps)
        #steps = max(steps,self.smin-self.steps)
        steps = min(max(steps,self.smin-self.steps),self.smax-self.steps)

        # local tracking
        sc = 0             # step count
        
        # accurate timing
        stime = int(round(1000000/max(1,(sps or self.sps)),0))
        #                 (   timenow -                  (lastpset + stime)) >= 0:
        if time.ticks_diff(time.ticks_us(),time.ticks_add(self.last, stime)) >= 0:
            waitfor = time.ticks_us()
        else:
            waitfor = time.ticks_add(self.last,stime)

        # wake
        if self.isoff:
            self.wake()

        # set direction
        if steps < 0:
            self.pd.value(self.reverse)
            sv = -1
        else:
            self.pd.value(self.forward)
            sv = 1

        # step
        for s in range(abs(steps)):
            while waitfor and time.ticks_diff(time.ticks_us(),waitfor) < 0:
                time.sleep_us(10)
            self.last = time.ticks_us()
            waitfor = time.ticks_add(waitfor,stime)
            self.ps.value(1)
            time.sleep_us(10) # a4988 requires 1 us
            self.ps.value(0)            
            sc += sv

        # sleep
        if sleep:
            self.sleep()

        # done
        self.steps += sc
        return self.steps

    def beep(self,freq=440,time_ms=250,pause=0,sleep=False):

        # determine steps based on freq and period
        stime = int(round(500000/max(1,freq),0)) # 500000 is divide-by-2 (each step is a forward and back)
        steps = int(round(time_ms*500/stime,0)) # 500 is divide-by-2 (each step is a forward and back)

        # timing setup (see step() for details)
        if time.ticks_diff(time.ticks_us(),time.ticks_add(self.last, stime)) >= 0:
            waitfor = time.ticks_us()
        else:
            waitfor = time.ticks_add(self.last,stime)

        # wake
        if self.isoff:
            self.wake()

        # step forward and back
        for s in range(steps):

            # forward
            self.pd.value(self.forward)
            while waitfor and time.ticks_diff(time.ticks_us(),waitfor) < 0:
                time.sleep_us(10)
            self.last = time.ticks_us()
            waitfor = time.ticks_add(waitfor,stime)
            self.ps.value(1)
            time.sleep_us(10) # a4988 requires 1 us
            self.ps.value(0)

            # back
            self.pd.value(self.reverse)
            while waitfor and time.ticks_diff(time.ticks_us(),waitfor) < 0:
                time.sleep_us(10)
            self.last = time.ticks_us()
            waitfor = time.ticks_add(waitfor,stime)
            self.ps.value(1)
            time.sleep_us(10) # a4988 requires 1 us
            self.ps.value(0)

        # sleep
        if sleep:
            self.sleep()

        # pause
        if pause:
            time.sleep_ms(pause)

        # done
        return self.steps

#-----------------------------------------------
# end
#-----------------------------------------------

try:
    from machine import SoftI2C
    __i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=200000)
except Exception as e:
    print("SoftI2C not exist. Using old I2C", e)
    from machine import I2C
    __i2c = I2C(scl=Pin(22), sda=Pin(21), freq=200000)
    
pca9685_obj = pca9685.PCA9685(__i2c, 0x40)
s = HBRIDGE(pca9685_obj, 11,10,8,9, mode=3, reverse=False, invert=False, sleep=False, sps=200, smax=10240, smin=-10240 )