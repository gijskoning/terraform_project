# Todo add functions for arduino communication
import math

import numpy as np
from serial import Serial
import time

arduino = Serial(port='COM6', baudrate=115200, timeout=.1)


def set_servo(q, debug=False):
    q = q / math.pi * 100
    q = q.astype(int)
    q = np.clip(q, 0,100)
    s1, s2, s3 = q.astype(int)
    # write_read(f"0:{s1},1:{s2},2:{s3}", debug)
    write_read(f"0:{s1},1:{s2}", debug)


def write_read(x, debug=False):
    if debug:
        print("Sending: ", x)
    arduino.write(bytes(str(x), 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    if debug and data is not None:
        print("data:", data)
    return data


# while True:
#     num = input("Enter a number: ") # Taking input from user
#     value = write_read(num)
#     print(value) # printing the value
# Step 4: Arduino Code

# :param action: [x,z,z_rot,gripper_pos] floats

def sent_action(action, debug=False):
    """
    # :param action: [q1,q2,q3,q4] floats q4 is gripper
    """
    # [360-660]
    set_servo(action[:3], debug)


if __name__ == '__main__':
    # test code
    sent_action("test")
