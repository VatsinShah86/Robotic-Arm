import gpiozero.pins.lgpio
import lgpio

def __patched_init(self, chip=None):
    gpiozero.pins.lgpio.LGPIOFactory.__bases__[0].__init__(self)
    chip = 0
    self._handle = lgpio.gpiochip_open(chip)
    self._chip = chip
    self.pin_class = gpiozero.pins.lgpio.LGPIOPin

gpiozero.pins.lgpio.LGPIOFactory.__init__ = __patched_init

from gpiozero import AngularServo
from time import sleep

servo1 = AngularServo(18, min_angle = 0, max_angle = 180, min_pulse_width=0.0008, max_pulse_width=0.0022)
servo2 = AngularServo(20, min_angle = 0, max_angle = 180, min_pulse_width=0.0009, max_pulse_width=0.0023)
servo3 = AngularServo(21, min_angle = -90, max_angle = 90, min_pulse_width=0.0008, max_pulse_width=0.0023)
servo4 = AngularServo(26, min_angle = -90, max_angle = 90, min_pulse_width=0.0008, max_pulse_width=0.0030)

def move_servo_1(val):
    if val>180:
        val = 180
    elif val<0:
        val = 0
    servo1.angle = val

def move_servo_2(val):
    if val>180:
        val = 180
    elif val<0:
        val = 0
    servo2.angle = val

def move_servo_3(val):
    if val>90:
        val = 90
    elif val<-90:
        val = -90
    servo3.angle = val

def move_servo_4(val):
    if val>90:
        val = 90
    elif val<-90:
        val = -90
    servo4.angle = val

if __name__ == "__main__":
    try:
        while True:
            servo.angle = 0
            print("min")
            sleep(5)
            servo.angle = 90
            print("mid")
            sleep(5)
            servo.angle = 180
            print("max")
            sleep(5)

    except KeyboardInterrupt:
        print("Program Stopped")
