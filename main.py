import helper.servo as servo
from time import sleep
def servo_sweep():
    # servo values can be between 0 & 1
    val = 0
    direction = 0
    try:
        while True:
            if direction == 0:
                val+=0.1
                sleep(0.5)
                if val >= 1:
                    direction = 1
            elif direction == 1:
                val -= 0.1
                sleep(0.5)
                if val <= 0:
                    direction = 0
            else:
                val = 0
                direction = 0
            print("direction is ", direction, " value is ", val, "\n")
            servo.move_servo_1(val)
    except KeyboardInterrupt:
        servo.move_servo_1(0)
        print("Program Terminated")

def set_angle_1(ang):
    servo.move_servo_1(ang)

def set_angle_2(ang):
    servo.move_servo_2(ang)

def set_angle_3(ang):
    servo.move_servo_3(ang)

if __name__ == "__main__":
    set_angle_1(90)
    set_angle_2(90)
    set_angle_3(90)

    sleep(10)