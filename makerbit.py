from yolobit import *
import machine, neopixel
import time
from machine import UART
import binascii


class Car():
    def __init__(self):
        pin2.write_digital((1))
        pin3.write_digital((1))
        pin10.write_digital((1))
        pin11.write_digital((1))
        pin12.write_analog(0)
        pin16.write_analog(0)
        pin4.write_digital((0))
        pin6.write_digital((0))
        
    def forward(self,speed):

        pin2.write_digital((0))
        pin3.write_digital((1))
        pin10.write_digital((1))
        pin11.write_digital((0))
        self.setspeed(speed)
        
    def backward(self,speed):

        pin2.write_digital((1))
        pin3.write_digital((0))
        pin10.write_digital((0))
        pin11.write_digital((1))
        self.setspeed(speed)

    def right(self,speed):

        pin2.write_digital((0))
        pin3.write_digital((1))
        pin10.write_digital((0))
        pin11.write_digital((1))
        self.setspeed(speed)

    def left(self,speed):

        pin2.write_digital((1))
        pin3.write_digital((0))
        pin10.write_digital((1))
        pin11.write_digital((0))
        self.setspeed(speed)

    def stop(self):
        pin2.write_digital((1))
        pin3.write_digital((1))
        pin10.write_digital((1))
        pin11.write_digital((1))

    def setspeed(self,speed):
        pin12.write_analog(speed)
        pin16.write_analog(speed)

    def motor_left(self,state,speed):
        
        if state:
            pin2.write_digital((0))
            pin3.write_digital((1))
        else:
            pin2.write_digital((1))
            pin3.write_digital((0))
        pin12.write_analog(speed)
    
    def motor_right(self,state,speed):
        if state:
            pin10.write_digital((1))
            pin11.write_digital((0))
        else:
            pin10.write_digital((0))
            pin11.write_digital((1))
        pin16.write_analog(speed)

    def led_left(self,state):
        if state:
            pin4.write_digital((1))
        else:
            pin4.write_digital((0))

    def led_right(self,state):
        if state:
            pin6.write_digital((1))
        else:
            pin6.write_digital((0))
    
    def both_led(self,state):
        if state:
            pin4.write_digital((1))
            pin6.write_digital((1))
        else:
            pin4.write_digital((0))
            pin6.write_digital((0))

def testThread():
    global wait
    print (wait)
    while True:
        if wait:
            pin4.write_digital((0))
            pin6.write_digital((0))
            time.sleep_ms(500)
            pin4.write_digital((1))
            pin6.write_digital((1))
            time.sleep_ms(500)

def hex_to_rgb(value):
    value = value.lstrip('#')
    lv = len(value)
    return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))

Red = [255, 0, 0]

class RGBcar():
    def __init__(self):
        self.np = neopixel.NeoPixel(machine.Pin(23), 6)
        self.brightness = 50
        self.clearLed()

    def __convert_color(self, color):
        if color is None:
            color = Red
        if isinstance(color, str):
            try:
                color = hex_to_rgb(color)
            except Exception as e:
                color = Red
        rate = self.brightness/float(100)
        return (int(color[0]*rate/4), int(color[1]*rate/4), int(color[2]*rate/4))

    def singleLed(self,num,color):
        color = self.__convert_color(color)
        self.np[int(num)] = color
        self.np.write()

    def clearLed(self):
        for i in range(6):
            self.np[i] = (0, 0, 0)
        self.np.write()
    
    def all_led(self,color):
        color = self.__convert_color(color)
        for i in range(6):
            self.np[i] = color
        self.np.write()

        
class Ultrasonic_car():
    def __init__(self):
        self.echo_timeout_us = 500*2*30
        self.trigger = Pin(pin13.pin, mode=Pin.OUT, pull=None)
        self.trigger.value(0)
        self.echo = Pin(pin14.pin, mode=Pin.IN, pull=None)
        
    def _send_pulse_and_wait(self):
        self.trigger.value(0)
        time.sleep_us(5)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        try:
            pulse_time = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
            return pulse_time
        except OSError as ex:
            if ex.args[0] == 110: 
                raise OSError('Out of range')
            raise ex

    def distance_mm(self):
        pulse_time = self._send_pulse_and_wait()
        mm = pulse_time * 100 // 582
        return mm

    def distance_cm(self):
        pulse_time = self._send_pulse_and_wait()
        cms = (pulse_time / 2) / 29.1
        return cms
