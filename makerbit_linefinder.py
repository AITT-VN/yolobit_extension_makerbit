from setting import *
from machine import Pin

S1_IN_S2_OUT = const(0)
S1_OUT_S2_IN = const(1)
S1_OUT_S2_OUT = const(2)
S1_IN_S2_IN = const(3)

class LineFinder:
    def __init__(self, sig_1, sig_2):
        self.sig_1 = sig_1
        self.sig_2 = sig_2
        self.left_pin = Pin(sig_1, mode=Pin.IN, pull=None)
        self.right_pin = Pin(sig_2, mode=Pin.IN, pull=None)

    def read(self):
        if self.left_pin.value() and not self.right_pin.value():
            # detect line on left side
            return S1_IN_S2_OUT
        elif not self.left_pin.value() and self.right_pin.value():
            # detect line on right side
            return S1_OUT_S2_IN
        elif not self.left_pin.value() and not self.right_pin.value():
            # detect line on none side
            return S1_OUT_S2_OUT
        elif self.left_pin.value() and self.right_pin.value():
            # detect line on both side
            return S1_IN_S2_IN
        else:
            # detect error
            return -1

line_finder = LineFinder()