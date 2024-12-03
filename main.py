import helper.servo as servo
from time import sleep
import numpy as np
class Robot:
    def __init__(self):
        print("created robot")
        self.FK()
        self.a1 = 38.55    # Length of the first link
        self.a2 = 120      # Length of the second link
        self.a3 = 187.75   # Length of the third link

    def FK(self):
        print("creating FK")

    def IK(self, pWx, pWy, pWz):
        # Moving pWz from frame 0 to frame 1
        pWz -= self.a1
        # Calculating cos(theta3)
        c3 = (pWx**2 + pWy**2 + pWz**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
        
        # Check for the validity of the solution
        if np.abs(c3) > 1:
            print("Point outside workspace")
            return 0 
        
        s3_pos = np.sqrt(1 - c3**2)  # sin(theta3) for the first solution
        s3_neg = -s3_pos            # sin(theta3) for the second solution

        # Calculating theta3
        theta3_1 = np.arctan2(s3_pos, c3)  # First solution for theta3
        theta3_2 = np.arctan2(s3_neg, c3)  # Second solution for theta3

        # # Choosing the theta3 that is in permissible range
        # if theta3_1 >= 0 and theta3_1 <= np.pi:
        #     theta3 = theta3_1

        # elif theta3_2 >= 0 and theta3_2 <= np.pi:
        #     theta3 = theta3_2
        # else:
        #     print("No unique theta 3 within constraints")
        #     return 0
        # Calculating theta1
        theta1_1 = np.arctan2(pWy, pWx)  # First solution for theta1
        theta1_2 = np.arctan2(-pWy, -pWx)  # Second solution for theta1

        # # Calculating c2 and s2 for theta2
        # c2_pos = (pWx**2 + pWy**2) * (self.a2 + self.a3 * c3) + pWz * self.a3 * s3_pos
        # s2_pos = pWz * (self.a2 + self.a3 * c3) - np.sqrt(pWx**2 + pWy**2) * self.a3 * s3_pos
        # c2_neg = (pWx**2 + pWy**2) * (self.a2 + self.a3 * c3) + pWz * self.a3 * s3_neg
        # s2_neg = pWz * (self.a2 + self.a3 * c3) - np.sqrt(pWx**2 + pWy**2) * self.a3 * s3_neg
        
        # # Calculating theta2
        # theta2_1 = np.arctan2(s2_pos, c2_pos)
        # theta2_2 = np.arctan2(s2_neg, c2_neg)
        # theta2_3 = np.arctan2(s2_pos, c2_neg)
        # theta2_4 = np.arctan2(s2_neg, c2_pos)
        
        # # Choosing the theta2 that is in permissible range
        # if theta2_1 >= 0 and theta2_1 <= np.pi:
        #     theta2 = theta2_1

        # elif theta2_2 >= 0 and theta2_2 <= np.pi:
        #     theta2 = theta2_2

        # elif theta2_3 >= 0 and theta2_3 <= np.pi:
        #     theta2 = theta2_3

        # elif theta2_4 >= 0 and theta2_4 <= np.pi:
        #     theta2 = theta2_4
        
        # else:
        #     print("No unique theta 2 within constraints")
        #     return 0
        theta2_1 = np.arctan2(pWz,np.sqrt(pWx**2 + pWy**2)) - np.arctan2((self.a3*np.sin(theta3_1)),(self.a2 + self.a3*np.cos(theta3_1)))
        theta2_2 = np.arctan2(pWz,np.sqrt(pWx**2 + pWy**2)) - np.arctan2((self.a3*np.sin(theta3_2)),(self.a2 + self.a3*np.cos(theta3_2)))
        # Choosing the theta1 that is in permissible range

        if theta2_1 >= 0 and theta2_1 <= np.pi and theta3_1 >= -np.pi/2 and theta3_1 <= np.pi/2:
            theta2 = theta2_1
            theta3 = theta3_1

        else:
            theta2 = theta2_2
            theta3 = theta3_2
        print(theta3_1, theta3_2)

        if abs(theta3_1) > np.pi/2 and abs(theta3_2) > np.pi/2:
            print("No unique theta 3 within constraints")
            return 0

        if abs(theta2_1) > 0 and abs(theta2_2) > np.pi:
            print("No unique theta 2 within constraints")
            return 0

        if theta1_1 >= 0 and theta1_1 <= np.pi:
            theta1 = theta1_1

        elif theta1_2 >= 0 and theta1_2 <= np.pi:
            theta1 = theta1_2
            theta2 = np.pi-theta2
            theta3 = -theta3

        else:
            print("No unique theta 1 within constraints")
            return 0
        return {
            "theta1": np.rad2deg(theta1),
            "theta2": np.rad2deg(theta2),
            "theta3": np.rad2deg(theta3)
        }


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
    set_angle_1(0)
    set_angle_2(0)
    set_angle_3(0)

    sleep(5)
    robot = Robot()
    pWx = -(robot.a2+robot.a3)/np.sqrt(2)/1.1 # X position of the end-effector
    pWy = -(robot.a2+robot.a3)/np.sqrt(2)/1.2 # Y position of the end-effector
    pWz = -60  # Z position of the end-effector
    

    solution = robot.IK(pWx, pWy, pWz)

    if solution != 0:
        print(f"theta1: {solution['theta1']}")
        print(f"theta2: {solution['theta2']}")
        print(f"theta3: {solution['theta3']}")
        
        set_angle_1(solution['theta1'])
        set_angle_2(solution['theta2'])
        set_angle_3(-solution['theta3'])
    sleep(10)